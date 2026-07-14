#!/usr/bin/env python3
"""
navila_node.py

ROS 2 node implementing the NaVILA Vision-Language-Action inference loop
for autonomous robot navigation driven by natural-language instructions.

The node is pure ROS plumbing: it decodes camera frames, handles timing and
threading, and drives a ROS-agnostic policy object (NaViLAAgent) that owns the
model, frame history, inference, output parsing and the action queue. The node
never touches policy state directly.

Architecture
------------
Event-driven, step-synchronous control loop replicating the official NaVILA
inference pipeline (one decision → one primitive → observe → next decision):

    /goal_instruction (String)
              │
              ▼
    /zed/.../compressed ──► [ navila_node ] ──► /navila/action (String)
                                  ▲                       │
                                  │              [ action_node ]
                                  │                       │
                          /navila/primitive_status ◄──────┘
                           (String: done | aborted)

    On mission completion the node emits /navila/complete (Bool).

The frame history advances by exactly one frame per completed primitive, held
inside the agent. The node only reports the outcome of the previous primitive
(done/aborted/start) so the agent can promote or discard the decision frame.

Subscribes
----------
    <image_topic>     (sensor_msgs/CompressedImage)
        Camera observations. Default: /zed/rgb/color/rect/image/compressed
    <goal_topic>      (std_msgs/String)
        Natural-language instruction. A new goal resets the agent and arms
        the loop. Default: /goal_instruction
    <reset_topic>     (std_msgs/Empty)
        Resets the agent and disarms the loop without a new goal.
        Default: /navila/reset
    <status_topic>    (std_msgs/String)
        Primitive completion signal from action_node: 'done' | 'aborted'.
        Default: /navila/primitive_status

Publishes
---------
    <action_topic>    (std_msgs/String)
        Primitive command '<action> <value> <unit>', e.g. 'forward 25 cm',
        'turn_left 15 deg'. Default: /navila/action
    <complete_topic>  (std_msgs/Bool)
        Published (data=True) when the agent decides 'stop' (mission complete).
        Default: /navila/complete

Parameters
----------
    model_path              str     NaVILA checkpoint dir.
                                    Default: $NAVILA_MODEL_PATH or '/models'
    num_video_frames        int     Frames per inference step (7 hist + 1 curr).
                                    Reconciled against the checkpoint at load
                                    time (checkpoint is authoritative). Default: 8
    max_history_frames      int     Max depth of the agent frame history. Default: 512
    frame_wait_timeout_sec  float   Max wait for a fresh frame after a primitive
                                    completes. Default: 1.0
    frame_settle_sec        float   Settling margin after motion before sampling
                                    the observation frame. Default: 0.0
    is_frame_rgb            bool    Channel order of the decoded native frame.
                                    False → native is BGR (cv2.imdecode default),
                                    converted to RGB before inference; True → the
                                    source already yields RGB. Default: False
    debug_dir               str     Root dir for per-run debug frames.
                                    Default: /home/ros_ws/tmp/navila_debug
    image_topic / goal_topic / action_topic / reset_topic /
    status_topic / complete_topic   str   See Subscribes/Publishes above.

Color handling
--------------
    _decode_frame() only decodes CompressedImage → native BGR (no conversion).
    Consumers convert on demand: _to_rgb() feeds the agent (model expects RGB),
    _to_bgr() feeds cv2.imwrite in the debug path. Both honour is_frame_rgb and
    _to_rgb always returns a fresh array, so the model image and the debug image
    are independent.

Inference pipeline
------------------
    1. On 'done'/'aborted' (or a new goal), _kick_drive() launches _drive_thread.
    2. _drive_thread() waits for a fresh frame (up to frame_wait_timeout_sec),
       decodes it, converts to RGB and calls agent.step(rgb, prev_status).
    3. The agent replays a queued primitive when the queue is non-empty,
       otherwise runs inference, parses the output and expands the magnitude
       into unit primitives (25 cm / 15 deg), publishing the first and queuing
       the rest. History promotion/discard is driven by prev_status.
    4. On 'stop' the loop disarms and /navila/complete is published.

Debug
-----
    Each step writes an annotated PNG (action label, chunk, instruction,
    direction arrow) to <debug_dir>/run_<timestamp>/frame_<idx>.png. A new run
    directory is created whenever the instruction changes.

Notes
-----
    - The model loads in a background thread so ROS spin is never blocked.
    - The frame-freshness gate uses time.monotonic() and a local frame counter,
      never header.stamp, making it robust to the dual-machine ZED/inference
      setup where clocks may differ.
"""

import time

import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Empty, Bool

import cv2
import numpy as np

from navila_ros2_bridge.third_party.navila_agent import NaViLAAgent

class NaViLANode(Node):

    def __init__(self):
        super().__init__("navila_super_node")

        #------------------------------------------------------------------------
        # ============================ Paramters ================================
        #------------------------------------------------------------------------
        d = lambda name, default: self.declare_parameter(name, default)

        d("model_path",                os.environ.get("NAVILA_MODEL_PATH", "/models"))
        d("num_video_frames",          8)
        d("max_history_frames",        512) 
        d("frame_wait_timeout_sec",    1.0) 
        d("frame_settle_sec",          0.0)        
        d("is_frame_rgb",              False)

        d("image_topic",               "/zed/rgb/color/rect/image/compressed")
        d("goal_topic",                "/goal_instruction")

        d("action_topic",              "/navila/action")
        d("reset_topic",               "/navila/reset")
        d("status_topic",              "/navila/primitive_status")
        d("complete_topic",            "/navila/complete")

        d("debug_dir",                  "/home/ros_ws/tmp/navila_debug")

        #------------------------------------------------------------------------
        p = lambda n: self.get_parameter(n).value

        model_path                  =   p("model_path")
        self._num_video_frames      =   p("num_video_frames")
        max_history_frames          =   p("max_history_frames")
        self._frame_wait_timeout    =   p("frame_wait_timeout_sec")
        self._frame_settle          =   p("frame_settle_sec")
        image_topic                 =   p("image_topic")
        goal_topic                  =   p("goal_topic")
        action_topic                =   p("action_topic")
        reset_topic                 =   p("reset_topic")
        status_topic                =   p("status_topic")

        self._frame_is_rgb          =   p("is_frame_rgb")
        self._debug_dir             =   p("debug_dir")
        complete_topic              =   p("complete_topic")

        self.agent = NaViLAAgent(
                model_path          =  model_path,
                num_video_frames    =  self._num_video_frames,
                max_history_frames  =  max_history_frames,
        )

        os.makedirs(self._debug_dir, exist_ok=True)

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self._last_image_msg        =   None 
        self._frame_seq             =   0
        self._motion_done_seq       =   0
        self._motion_done_mono      =   time.monotonic()

        self._active                =   False
        self._cycle_active          =   False

        self._prev_status           =   "start"
        self._lock                  =   threading.Lock()

        self._debug_run = self._debug_dir
        self._debug_idx = 0
        self._debug_last_instr = None

        # ------------------------------------------------------------------
        # Subscribers
        # ----------------------------------------------------------------]]--
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub_image      = self.create_subscription(CompressedImage, image_topic,    self._image_cb,     qos_sensor)
        self.sub_goal       = self.create_subscription(String,          goal_topic,     self._goal_cb,      10)
        self.sub_reset      = self.create_subscription(Empty,           reset_topic,    self._reset_cb,     10)
        self.sub_status     = self.create_subscription(String,          status_topic,   self._primitive_cb, 10)        

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self.pub_action     = self.create_publisher(String, action_topic,   10)
        self.pub_complete   = self.create_publisher(Bool,   complete_topic, 10)


        # ------------------------------------------------------------------
        # Inference timer
        # ------------------------------------------------------------------
        self.timer = self.create_timer(0.5, self._kick_drive)

        # ------------------------------------------------------------------
        # Startup log
        # ------------------------------------------------------------------
        self.get_logger().info(
            f"\n{'='*60}\n"
            f"  NaViLANode starting\n"
            f"  model_path        : {model_path}\n"
            f"  num_video_frames  : {self._num_video_frames} (may be overridden by checkpoint)\n"
            f"  max_history_frames: {max_history_frames}\n"
            f"  input_color_order : {self._frame_is_rgb}\n"
            f"  frame_wait_timeout: {self._frame_wait_timeout}s\n"
            f"  frame_settle      : {self._frame_settle}s\n"
            f"  image_topic       : {image_topic}\n"
            f"  goal_topic        : {goal_topic}\n"
            f"  action_topic      : {action_topic}\n"
            f"  status_topic      : {status_topic}\n"
            f"  reset_topic       : {reset_topic}\n"
            f"{'='*60}"
        )
        # ------------------------------------------------------------------
        # Load models in a background thread (non-blocking for ROS 2 spin)
        # ------------------------------------------------------------------
        self.get_logger().info(f"Loading NaVILA from: {model_path}")
        threading.Thread(
            target=self._load_model_thread,
            args=(model_path,),
            daemon=True,
        ).start()

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------
    def _image_cb(self, msg: CompressedImage):
        with self._lock:
            self._last_image_msg = msg
            self._frame_seq += 1

    def _decode_frame(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)   # solo decode → BGR nativo
            if frame is None:
                raise ValueError("cv2.imdecode returned None — corrupted frame?")
            return frame                                     # nessuna conversione qui
        except Exception as exc:
            self.get_logger().warn(f"Image decode error: {exc}")
            return None

    def _primitive_cb(self, msg: String):
        status = msg.data.strip().lower()
        with self._lock:
            if status in ("done", "aborted"):
                self._prev_status = status
                self._motion_done_seq = self._frame_seq
                self._motion_done_mono = time.monotonic()
            self._cycle_active = False
        self._kick_drive()

    def _goal_cb(self, msg: String):
        with self._lock:
            self.agent.reset(msg.data)
            self._prev_status = "start"
            self._active = True
            self._cycle_active = False
            self._motion_done_seq = self._frame_seq
            self._motion_done_mono = time.monotonic()
        self.get_logger().info(f"New goal: '{msg.data}' (loop armed)")
        self._kick_drive()

    def _reset_cb(self, msg):
        with self._lock:
            self.agent.reset("")
            self._active        = False
            self._cycle_active  = False
            self._prev_status   = "start"
        self.get_logger().info("NaVILA reset (loop disarmed).")

    def _publish_complete(self):
        msg = Bool()
        msg.data = True
        self.pub_complete.publish(msg)

    # ------------------------------------------------------------------
    # Model loading (background thread)
    # ------------------------------------------------------------------

    def _load_model_thread(self, model_path: str):
        try:
            self.get_logger().info("Loading NaVILA model...")
            self.agent.load_model()                       # l'agent possiede tutto
            self._num_video_frames = self.agent.num_video_frames
            self.get_logger().info(
                f"NaVILA ready — num_video_frames={self._num_video_frames}, parser: REGEX")

            self.get_logger().info("Waiting for camera frame...")
            while rclpy.ok():
                with self._lock:
                    has_frame = self._last_image_msg is not None
                if has_frame:
                    break
                self.get_logger().info("Waiting for camera frame...", throttle_duration_sec=5.0)
                time.sleep(0.5)
            self.get_logger().info("Camera frame received")

            while rclpy.ok():
                if self.agent.goal:
                    break
                self.get_logger().info(
                    f"Waiting for goal on '{self.get_parameter('goal_topic').value}'...",
                    throttle_duration_sec=5.0)
                time.sleep(0.5)
            self.get_logger().info(f"Goal received: '{self.agent.goal}' — inference loop start ✓")

        except Exception as exc:
            self.get_logger().error(f"Failed to load NaVILA model: {exc}")

    # ------------------------------------------------------------------
    # Inference callback 09 / 06 / 2026 - versione con padding + debug
    # ------------------------------------------------------------------
    def _kick_drive(self):
        with self._lock:
            if self._cycle_active:
                return
            if not self.agent.ready:
                return
            if not self._active:
                self.get_logger().info(
                    "Waiting for goal instruction...",
                    throttle_duration_sec=5.0)
                return
            if self._last_image_msg is None:
                self.get_logger().info(
                    "Waiting for camera frame...",
                    throttle_duration_sec=5.0)
                return
            self._cycle_active = True
        threading.Thread(target=self._drive_thread, daemon=True).start()

    def _drive_thread(self):
        try:
            with self._lock:
                prev_status = self._prev_status
                after_seq, after_mono = self._motion_done_seq, self._motion_done_mono
                settle, timeout = self._frame_settle, self._frame_wait_timeout

            image_msg, stale = self._wait_fresh_frame(after_seq, after_mono, settle, timeout)
            if stale:
                self.get_logger().warn("Nessun frame fresco entro il timeout — uso l'ultimo.",
                                        throttle_duration_sec=5.0)
            curr = self._decode_frame(image_msg)
            if curr is None:
                with self._lock:
                    self._cycle_active = False
                return

            r = self.agent.step(self._to_rgb(curr), prev_status)
            self._save_debug_frame(curr, self.agent.goal, [r["cmd"]])

            if r["is_stop"]:
                with self._lock:
                    self._active = False
                    self._cycle_active = False
                self.get_logger().info(f"raw='{r['raw']}' → STOP (hist:{r['hist_len']})")
                self._publish_complete()
                msg = String
                msg.data = "stop"
                self.pub_action.publish(msg)
                return

            tag = "queue" if r["from_queue"] else "inference"
            if tag == "inference":
                self.get_logger().info(f"NaViLA raw: {r['raw']}")
            self.get_logger().info(
                f"[{tag}] cmd='{r['cmd']}' ×{r['n_total']} "
                f"(coda:{len(self.agent._queue)}, hist:{r['hist_len']})")
            out = String(); out.data = r["cmd"]
            self.pub_action.publish(out)
            # _cycle_active resta True: sarà lo status callback a resettarlo (event-driven)

        except Exception as exc:
            self.get_logger().error(f"Drive error: {exc}")
            with self._lock:
                self._cycle_active = False

    #------------------------------------------------------------------------
    # ============================ Helpers ==================================
    #------------------------------------------------------------------------
    def _to_rgb(self, frame):
        return frame if self._frame_is_rgb else cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def _to_bgr(self, frame):
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if self._frame_is_rgb else frame

    def _wait_fresh_frame(self, after_seq, after_mono, settle_sec, timeout_sec, poll_sec=0.02):
        """Wait for the first frame received AFTER after_seq and AFTER (after_mono + settle_sec).

        Uses a local frame counter (_frame_seq) and time.monotonic() exclusively —
        never header.stamp — making it robust to the dual-machine setup where the
        ZED camera (robot PC) and the inference PC have unsynchronized clocks.

        With QoS depth=1/KEEP_LAST, _last_image_msg always holds the most recent
        frame: once the settle period has elapsed, the available frame is already
        post-settling by construction.

        Args:
            after_seq   : value of _frame_seq at the last done/aborted event.
                        The returned frame must have seq > after_seq.
            after_mono  : time.monotonic() timestamp of the last done/aborted event.
            settle_sec  : settling margin after motion completion (s). Allows the
                        camera to stabilize before sampling the observation.
            timeout_sec : maximum total wait time (s). If exceeded, returns the
                        latest available frame with stale=True.
            poll_sec    : polling interval (s). Default 0.02 (50 Hz).

        Returns:
            (msg, stale): msg   — most recent CompressedImage available, or None
                                if no frame has ever been received.
                        stale — True if the timeout expired before a fresh frame
                                arrived, False if the frame is genuinely new
                                relative to the previous motion primitive.
        """
        deadline     = time.monotonic() + float(timeout_sec)
        settle_until = after_mono + float(settle_sec)
        while rclpy.ok():
            with self._lock:
                msg = self._last_image_msg
                seq = self._frame_seq
            if msg is not None and seq > after_seq and time.monotonic() >= settle_until:
                return msg, False
            if time.monotonic() >= deadline:
                return msg, True
            time.sleep(poll_sec)
        return None, True

    
    

    #------------------------------------------------------------------------
    # ============================ DEBUG ====================================
    #------------------------------------------------------------------------
    def _save_debug_frame(self, frame, instruction, actions):
        if isinstance(actions, str):
            actions = [actions]
        if instruction != self._debug_last_instr:
            ts = time.strftime("%Y%m%d_%H%M%S")
            self._debug_run = os.path.join(self._debug_dir, f"run_{ts}")
            os.makedirs(self._debug_run, exist_ok=True)
            self._debug_idx = 0
            self._debug_last_instr = instruction
        img = self._to_bgr(frame).copy()
        executed = actions[0] if actions else "-"
        verb = executed.split()[0] if executed != "-" else "-"   # "forward 25 cm" → "forward"
        chunk = " ".join(actions)
        h, w = img.shape[:2]
        cv2.rectangle(img, (0, 0), (w, 72), (0, 0, 0), -1)
        cv2.putText(img, f"step {self._debug_idx:04d}   EXEC: {executed.upper()}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(img, f"chunk: {chunk}",
                    (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img, instruction[:70],
                    (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        # freccetta direzione dell'azione eseguita
        cx, cy = w - 60, 40
        if verb == "forward":
            cv2.arrowedLine(img, (cx, cy + 12), (cx, cy - 12), (0, 255, 0), 3, tipLength=0.4)
        elif verb in ("left", "turn_left"):
            cv2.arrowedLine(img, (cx + 12, cy), (cx - 12, cy), (0, 255, 0), 3, tipLength=0.4)
        elif verb in ("right", "turn_right"):
            cv2.arrowedLine(img, (cx - 12, cy), (cx + 12, cy), (0, 255, 0), 3, tipLength=0.4)
        elif verb == "stop":
            cv2.circle(img, (cx, cy), 12, (0, 0, 255), -1)
        cv2.imwrite(os.path.join(self._debug_run, f"frame_{self._debug_idx:05d}.png"), img)
        self._debug_idx += 1
    #------------------------------------------------------------------------enddebug





# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = NaViLANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
