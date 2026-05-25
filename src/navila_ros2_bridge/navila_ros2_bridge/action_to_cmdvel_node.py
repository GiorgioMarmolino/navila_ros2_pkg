#!/usr/bin/env python3
"""
action_to_cmdvel_node.py
Converte le azioni di NaVILA in comandi di velocità (geometry_msgs/Twist)
pubblicati su /cmd_vel.

Subscribes:
    /navila/action  (std_msgs/String)
        Accetta due formati:
          1) JSON  →  {"linear_x": 0.3, "angular_z": 0.2}
          2) Token →  "forward" | "forward_fast" | "backward"
                      "turn_left" | "turn_right"
                      "curve_left" | "curve_right"
                      "stop"

Publishes:
    /cmd_vel  (geometry_msgs/Twist)

Features:
    - Watchdog timer: STOP automatico dopo <cmd_timeout_sec> senza comandi
    - Velocity smoothing: rampa di accelerazione (acceleration limiting)
    - Action schema ricco con token estesi + JSON pass-through
    - Logging throttled (debug per frame, info/warn per eventi)
    - Parametri ROS 2 configurabili
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


# ---------------------------------------------------------------------------
# Default parameters
# ---------------------------------------------------------------------------
DEFAULT_LINEAR_X       = 0.5    # m/s  — "forward"
DEFAULT_LINEAR_X_FAST  = 1.0    # m/s  — "forward_fast"
DEFAULT_LINEAR_X_BACK  = 0.2    # m/s  — "backward"
DEFAULT_ANGULAR_Z      = 0.5    # rad/s — rotazione sul posto
DEFAULT_CURVE_LINEAR   = 0.2    # m/s  — componente lineare in "curve_*"
DEFAULT_CURVE_ANGULAR  = 0.4    # rad/s — componente angolare in "curve_*"

DEFAULT_CMD_TIMEOUT    = 0.9    # s    — watchdog: stop se nessun cmd
DEFAULT_WATCHDOG_RATE  = 0.05   # s    — periodo timer watchdog (20 Hz)
DEFAULT_PUBLISH_RATE   = 0.05   # s    — periodo pubblicazione (20 Hz)

DEFAULT_MAX_ACC_LIN    = 1.0    # m/s² — max accelerazione lineare
DEFAULT_MAX_ACC_ANG    = 2.0    # rad/s² — max accelerazione angolare


class ActionToCmdVelNode(Node):

    def __init__(self):
        super().__init__("action_to_cmdvel_node")

        # ------------------------------------------------------------------
        # Parametri ROS 2
        # ------------------------------------------------------------------
        self.declare_parameter("action_topic",      "/navila/action")
        self.declare_parameter("cmd_vel_topic",     "/cmd_vel")

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

        def p(name):
            return self.get_parameter(name).value

        action_topic      = p("action_topic")
        cmd_vel_topic     = p("cmd_vel_topic")
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
        # Subscriber / Publisher / Timer
        # ------------------------------------------------------------------
        self.sub_action = self.create_subscription(
            String, action_topic, self._action_cb, 10)

        qos_cmd_vel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,                              # match twist_mux
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist, cmd_vel_topic, qos_cmd_vel)

        # Watchdog: controlla timeout
        self._watchdog_timer = self.create_timer(
            watchdog_rate, self._watchdog_cb)

        # Publish loop: applica smoothing e pubblica a rate fisso
        self._publish_timer = self.create_timer(
            publish_rate, self._publish_cb)

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
    # Callback azione
    # ------------------------------------------------------------------

    def _action_cb(self, msg: String):
        raw = msg.data.strip()

        # --- Prova a parsare come JSON -----------------------------------
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

    def _publish_cb(self):
        # Rampa di accelerazione (clamp della variazione per timestep)
        self._current_lin = self._ramp(
            self._current_lin, self._target_lin,
            self.max_acc_lin * self._dt)

        self._current_ang = self._ramp(
            self._current_ang, self._target_ang,
            self.max_acc_ang * self._dt)

        twist = Twist()
        twist.linear.x  = self._current_lin
        twist.angular.z = self._current_ang
        self.pub_cmd_vel.publish(twist)

        self.get_logger().debug(
            f"cmd_vel → lin={self._current_lin:.3f}  "
            f"ang={self._current_ang:.3f}")

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