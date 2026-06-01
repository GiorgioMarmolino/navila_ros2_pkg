#!/usr/bin/env python3
"""
action_to_cmdvel_node.py

Converts NaVILA actions into velocity commands (geometry_msgs/Twist)
published on /cmd_vel.

Subscribes:
    /navila/action (std_msgs/String)
        Supported formats:
            1) JSON:
                {"linear_x": 0.3, "angular_z": 0.2}

            2) Action tokens:
                "forward"
                "forward_fast"
                "backward"
                "turn_left"
                "turn_right"
                "curve_left"
                "curve_right"
                "stop"

    /sensors/lidar3d_0/scan (sensor_msgs/LaserScan)

Publishes:
    /cmd_vel (geometry_msgs/Twist)

Features:
    - Command watchdog timeout with automatic STOP
    - LiDAR-based collision prevention
    - Front obstacle slowdown and emergency stop
    - Side-aware turning reduction for narrow corridors
    - Rear obstacle protection
    - LiDAR timeout fail-safe
    - Velocity smoothing with acceleration limiting
    - Extended action token mapping + JSON passthrough
    - Throttled logging for cleaner runtime output
    - Fully configurable ROS 2 parameters
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image

import numpy as np
from cv_bridge import CvBridge


# ---------------------------------------------------------------------------
# Default parameters
# ---------------------------------------------------------------------------
DEFAULT_LINEAR_X       = 0.5    # m/s  — "forward"
DEFAULT_LINEAR_X_FAST  = 1.0    # m/s  — "forward_fast"
DEFAULT_LINEAR_X_BACK  = 0.2    # m/s  — "backward"
DEFAULT_ANGULAR_Z      = 0.35    # rad/s — rotazione sul posto
DEFAULT_CURVE_LINEAR   = 0.2    # m/s  — componente lineare in "curve_*"
DEFAULT_CURVE_ANGULAR  = 0.4    # rad/s — componente angolare in "curve_*"

DEFAULT_LIDAR_TIMEOUT  = 2.5   #s     - max time without lidar
DEFAULT_CMD_TIMEOUT    = 1.0   # s    — watchdog: stop se nessun cmd
DEFAULT_WATCHDOG_RATE  = 0.05   # s    — periodo timer watchdog (20 Hz)
DEFAULT_PUBLISH_RATE   = 0.05   # s    — periodo pubblicazione (20 Hz)

DEFAULT_MAX_ACC_LIN    = 1.0    # m/s² — max accelerazione lineare
DEFAULT_MAX_ACC_ANG    = 2.0    # rad/s² — max accelerazione angolare

DEFAULT_FRONT_STOP_DIST = 0.75  # m    — distanza frontale per stop
DEFAULT_FRONT_SLOW_DIST = 1.2   # m    — distanza frontale per inizio rallentamento
DEFAULT_SIDE_STOP_DIST  = 0.5  # m   — distanza laterale per stop
DEFAULT_REAR_STOP_DIST  = 0.65



class ActionToCmdVelNode(Node):

    def __init__(self):
        super().__init__("action_to_cmdvel_node")

        # ------------------------------------------------------------------
        # Parametri ROS 2
        # ------------------------------------------------------------------
        self.declare_parameter("action_topic",      "/navila/action")
        self.declare_parameter("cmd_vel_topic",     "/cmd_vel")
        self.declare_parameter("scan_topic",        "/sensors/lidar3d_0/scan")
        self.declare_parameter("depth_topic",       "/sensors/front_camera/depth/image_raw")

        # Velocità per ogni token
        self.declare_parameter("linear_x",          DEFAULT_LINEAR_X)
        self.declare_parameter("linear_x_fast",     DEFAULT_LINEAR_X_FAST)
        self.declare_parameter("linear_x_back",     DEFAULT_LINEAR_X_BACK)
        self.declare_parameter("angular_z",         DEFAULT_ANGULAR_Z)
        self.declare_parameter("curve_linear",      DEFAULT_CURVE_LINEAR)
        self.declare_parameter("curve_angular",     DEFAULT_CURVE_ANGULAR)

        # Watchdog / smoothing
        self.declare_parameter("cmd_timeout_sec",   DEFAULT_CMD_TIMEOUT)
        self.declare_parameter("watchdog_rate_sec", DEFAULT_WATCHDOG_RATE)
        self.declare_parameter("publish_rate_sec",  DEFAULT_PUBLISH_RATE)
        self.declare_parameter("max_acc_linear",    DEFAULT_MAX_ACC_LIN)
        self.declare_parameter("max_acc_angular",   DEFAULT_MAX_ACC_ANG)

        # Safety distances (m)
        self.declare_parameter("front_stop_dist", DEFAULT_FRONT_STOP_DIST)
        self.declare_parameter("front_slow_dist", DEFAULT_FRONT_SLOW_DIST)
        self.declare_parameter("side_stop_dist", DEFAULT_SIDE_STOP_DIST)
        self.declare_parameter("rear_stop_dist", DEFAULT_REAR_STOP_DIST)

        # self.declare_parameter("use_sim_time", True)
        
        def p(name):
            return self.get_parameter(name).value

        action_topic      = p("action_topic")
        cmd_vel_topic     = p("cmd_vel_topic")
        scan_topic        = p("scan_topic")
        depth_topic       = p("depth_topic")

        self.lin          = p("linear_x")
        self.lin_fast     = p("linear_x_fast")
        self.lin_back     = p("linear_x_back")
        self.ang          = p("angular_z")
        self.curve_lin    = p("curve_linear")
        self.curve_ang    = p("curve_angular")
        self.timeout_sec  = p("cmd_timeout_sec")
        self.max_acc_lin  = p("max_acc_linear")
        self.max_acc_ang  = p("max_acc_angular")
        watchdog_rate     = p("watchdog_rate_sec")
        publish_rate      = p("publish_rate_sec")

        self.front_stop_dist = p("front_stop_dist")
        self.front_slow_dist = p("front_slow_dist")
        self.side_stop_dist = p("side_stop_dist")
        self.rear_stop_dist = p("rear_stop_dist")

        # ------------------------------------------------------------------
        # Mappa token → (linear_x, angular_z)
        # Estendibile senza toccare la logica del callback
        # ------------------------------------------------------------------
        self._action_map: dict[str, tuple[float, float]] = {
            "forward":      ( self.lin,       0.0),
            "forward_fast": ( self.lin_fast,  0.0),
            "backward":     (-self.lin_back,  0.0),
            "turn_left":    ( 0.0,            self.ang),
            "turn_right":   ( 0.0,           -self.ang),
            "curve_left":   ( self.curve_lin, self.curve_ang),
            "curve_right":  ( self.curve_lin,-self.curve_ang),
            # alias brevi per compatibilità con nodi legacy
            "left":         ( 0.0,            self.ang),
            "right":        ( 0.0,           -self.ang),
            "stop":         ( 0.0,            0.0),
        }

        # ------------------------------------------------------------------
        # Stato interno
        # ------------------------------------------------------------------
        self._target_lin: float = 0.0   # velocità target (da azione)
        self._target_ang: float = 0.0
        self._current_lin: float = 0.0  # velocità smoothed corrente
        self._current_ang: float = 0.0
        self._last_cmd_time = self.get_clock().now()

        # ------------------------------------------------------------------
        # Security states
        # ------------------------------------------------------------------
        self._front_blocked = False
        self._left_blocked = False
        self._right_blocked = False
        self._rear_blocked = False

        self._front_min_dist = 999.0
        self._left_min_dist = 999.0
        self._right_min_dist = 999.0
        self._rear_min_dist = 999.0
        self._last_scan_time = self.get_clock().now()

        self._bridge = CvBridge()
        self._front_depth_dist = 999.0
        self._last_depth_time = self.get_clock().now()

        # ------------------------------------------------------------------
        # Subscriber / Publisher / Timer
        # ------------------------------------------------------------------
        self.sub_action = self.create_subscription(String, action_topic, self._action_cb, 10)
        self.sub_depth = self.create_subscription(Image, depth_topic, self._depth_cb, 10)
        self.sub_scan = self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)

        qos_cmd_vel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,                              # match twist_mux
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub_cmd_vel = self.create_publisher(Twist, cmd_vel_topic, qos_cmd_vel)

        # --- TIMERS ---
        self._watchdog_timer = self.create_timer(watchdog_rate, self._watchdog_cb)  # Watchdog: controlla timeout
        self._publish_timer = self.create_timer(publish_rate, self._publish_cb)     # Publish loop: applica smoothing e pubblica a rate fisso
        
        
        
        
        
        
        
        ##############################################################################
        self._debug_timer = self.create_timer(3.0, self._debug_cb)  # 1 Hz
        ##############################################################################





        self._dt = publish_rate  # usato per la rampa di accelerazione

        self.get_logger().info(
            f"action_to_cmdvel_node avviato\n"
            f"  topic in  : {action_topic}\n"
            f"  topic out : {cmd_vel_topic}\n"
            f"  timeout   : {self.timeout_sec} s\n"
            f"  max_acc   : lin={self.max_acc_lin} m/s²  "
            f"ang={self.max_acc_ang} rad/s²\n"
            f"  azioni    : {list(self._action_map.keys())}"
        )

    # ------------------------------------------------------------------
    # Action callback
    # ------------------------------------------------------------------
    def _depth_cb(self, msg: Image):
        self._last_depth_time = self.get_clock().now()
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            h, w = depth.shape[:2]

            # ROI: zona centrale-bassa del frame (ostacoli vicini al suolo)
            roi = depth[h//2:h, w//4:3*w//4]

            # Filtra valori invalidi (0, inf, nan)
            valid = roi[(roi > 0.1) & np.isfinite(roi)]
            self._front_depth_dist = float(np.min(valid)) if len(valid) > 0 else 999.0

        except Exception as e:
            self.get_logger().warn(f"Depth error: {e}", throttle_duration_sec=2.0)

    def _action_cb(self, msg: String):
        raw = msg.data.strip()

        # --- Parse as JSON -----------------------------------
        if raw.startswith("{"):
            try:
                data = json.loads(raw)
                lx = float(data.get("linear_x",  0.0))
                az = float(data.get("angular_z", 0.0))
                self._set_target(lx, az, label=f"JSON({lx:.2f},{az:.2f})")
                return
            except (json.JSONDecodeError, ValueError) as exc:
                self.get_logger().warn(
                    f"JSON malformato: '{raw}' — {exc} → STOP")
                self._set_target(0.0, 0.0, label="JSON_ERROR→stop")
                return

        # --- Token string ------------------------------------------------
        action = raw.lower()
        if action in self._action_map:
            lx, az = self._action_map[action]
            self._set_target(lx, az, label=action)
        else:
            self.get_logger().warn(
                f"Azione non riconosciuta: '{action}' → STOP")
            self._set_target(0.0, 0.0, label="unknown→stop")

    def _set_target(self, lx: float, az: float, label: str = ""):
        self._target_lin = lx
        self._target_ang = az
        self._last_cmd_time = self.get_clock().now()
        self.get_logger().debug(
            f"target set [{label}] → "
            f"lin={lx:.3f}  ang={az:.3f}")

    def _scan_cb(self, msg: LaserScan):
        self._last_scan_time = self.get_clock().now()
        ranges = list(msg.ranges)
        n = len(ranges)

        self._ranges_debug = ranges # per debug, da rimuovere o limitare in futuro se troppo pesante
        self._n_debug = n # per debug, da rimuovere

        angle_increment = msg.angle_increment
        samples_30deg = int(math.radians(30) / angle_increment)

        def safe_min(values):
            vals = [v for v in values if not math.isinf(v) and not math.isnan(v)]
            return min(vals) if vals else 999.0

        front = ranges[-samples_30deg:] + ranges[:samples_30deg]
        left  = ranges[n//4 - samples_30deg : n//4 + samples_30deg]
        right = ranges[3*n//4 - samples_30deg : 3*n//4 + samples_30deg]
        rear  = ranges[n//2 - samples_30deg : n//2 + samples_30deg]

        self._front_min_dist = safe_min(front)
        self._left_min_dist  = safe_min(left)
        self._right_min_dist = safe_min(right)
        self._rear_min_dist  = safe_min(rear)

        self._front_blocked = self._front_min_dist < self.front_slow_dist
        self._left_blocked  = self._left_min_dist  < self.side_stop_dist
        self._right_blocked = self._right_min_dist < self.side_stop_dist
        self._rear_blocked  = self._rear_min_dist  < self.rear_stop_dist

    # ------------------------------------------------------------------
    # Watchdog callback
    # ------------------------------------------------------------------

    def _watchdog_cb(self):
        dt = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if dt > self.timeout_sec:
            if self._target_lin != 0.0 or self._target_ang != 0.0:
                self.get_logger().info(
                    f"Watchdog: nessun comando da {dt:.2f}s → STOP")
            self._target_lin = 0.0
            self._target_ang = 0.0

    # ------------------------------------------------------------------
    # Publish callback (smoothing + pubblicazione)
    # ------------------------------------------------------------------

    # def _publish_cb(self):
    #     # Rampa di accelerazione (clamp della variazione per timestep)
    #     # self._current_lin = self._ramp(
    #     #     self._current_lin, self._target_lin,
    #     #     self.max_acc_lin * self._dt)

    #     # self._current_ang = self._ramp(
    #     #     self._current_ang, self._target_ang,
    #     #     self.max_acc_ang * self._dt)

    #     # no ramp:
    #     self._current_lin = self._target_lin
    #     self._current_ang = self._target_ang

    #     twist = Twist()
    #     twist.linear.x  = self._current_lin
    #     twist.angular.z = self._current_ang
    #     self.pub_cmd_vel.publish(twist)

    #     self.get_logger().debug(
    #         f"cmd_vel → lin={self._current_lin:.3f}  "
    #         f"ang={self._current_ang:.3f}")





    ################################################################################
    def _debug_cb(self):
        self.get_logger().info(
            f"[SAFETY] "
            f"front_lidar={self._front_min_dist:.2f}m ({'BLOCK' if self._front_blocked else 'ok'})  "
            f"front_depth={self._front_depth_dist:.2f}m ({'BLOCK' if self._front_depth_dist < self.front_slow_dist else 'ok'})  "
            f"left={self._left_min_dist:.2f}m ({'BLOCK' if self._left_blocked else 'ok'})  "
            f"right={self._right_min_dist:.2f}m ({'BLOCK' if self._right_blocked else 'ok'})  "
            f"rear={self._rear_min_dist:.2f}m ({'BLOCK' if self._rear_blocked else 'ok'})  "
            f"| target=({self._target_lin:.2f}, {self._target_ang:.2f})"
        )
        # Aggiungi temporaneamente per capire l'orientamento
        self.get_logger().info(
            f"[LIDAR RAW] "
            f"idx0={self._ranges_debug[0]:.2f}m  "
            f"idx_n4={self._ranges_debug[self._n_debug//4]:.2f}m  "
            f"idx_n2={self._ranges_debug[self._n_debug//2]:.2f}m  "
            f"idx_3n4={self._ranges_debug[3*self._n_debug//4]:.2f}m"
        )
    ################################################################################









    def _publish_cb(self):
        # --- Scan timeout check ---
        scan_dt = (self.get_clock().now() - self._last_scan_time).nanoseconds / 1e9

        if scan_dt > DEFAULT_LIDAR_TIMEOUT:

            self.get_logger().warn("LIDAR TIMEOUT -> STOP", throttle_duration_sec=1.0)

            lin = 0.0
            ang = 0.0
        else:
            lin = self._target_lin
            ang = self._target_ang

            if lin > 0.0:                       # Front obstacle protection
                d = min(self._front_min_dist, self._front_depth_dist) # using depth camera
                if d < self.front_stop_dist:    # Hard stop
                    self.get_logger().warn("FRONT OBSTACLE -> STOP", throttle_duration_sec=1.0 )
                    lin = 0.0
                elif d < self.front_slow_dist:  # Progressive slowdown
                    scale = (
                        (d - self.front_stop_dist) / (self.front_slow_dist - self.front_stop_dist))
                    scale = max(0.0, min(scale, 1.0))
                    lin *= scale

            if lin < 0.0 and self._rear_blocked:# Rear obstacle protection
                self.get_logger().warn("REAR OBSTACLE -> STOP", throttle_duration_sec=1.0)
                lin = 0.0

            # Reduce turning aggressiveness in narrow spaces
            if lin > 0.0 and ang > 0.0 and self._left_blocked:
                self.get_logger().warn("LEFT SIDE CLOSE -> REDUCING TURN", throttle_duration_sec=1.0)
                ang *= 0.4

            if lin > 0.0 and ang < 0.0 and self._right_blocked:
                self.get_logger().warn("RIGHT SIDE CLOSE -> REDUCING TURN", throttle_duration_sec=1.0)
                ang *= 0.4
        
        # Acceleration ramp
        self._current_lin = self._ramp(self._current_lin, lin, self.max_acc_lin * self._dt)
        self._current_ang = self._ramp(self._current_ang, ang, self.max_acc_ang * self._dt)

        # Publish cmd_vel
        twist = Twist()
        twist.linear.x = self._current_lin
        twist.angular.z = self._current_ang
        self.pub_cmd_vel.publish(twist)
        self.get_logger().debug(
            f"cmd_vel -> "
            f"lin={self._current_lin:.3f} "
            f"ang={self._current_ang:.3f} "
            f"front={self._front_min_dist:.2f}m"
        )

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------

    @staticmethod
    def _ramp(current: float, target: float, max_delta: float) -> float:
        """Avvicina current a target di al massimo max_delta."""
        delta = target - current
        delta = math.copysign(min(abs(delta), max_delta), delta)
        return current + delta


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = ActionToCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutdown: invio STOP finale")
        stop = Twist()
        node.pub_cmd_vel.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()