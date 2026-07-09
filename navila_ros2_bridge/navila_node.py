#!/usr/bin/env python3
"""
navila_node.py

ROS 2 node implementing the NaVILA Vision-Language-Action inference loop
for autonomous robot navigation driven by natural-language instructions.

Architecture
------------
The node implements an event-driven, step-synchronous control loop that
replicates the official NaVILA inference pipeline:

    /goal_instruction (String)
              │
              ▼
    /zed/.../compressed ──► [ navila_node ] ──► /navila/action (String)
                                  ▲                       │
                                  │              [ action_node ]
                                  │                       │
                          /navila/primitive_status ◄──────┘
                           (String: done | aborted)

One model decision → execute one primitive → observe result → next decision.
This replicates the step-synchronous nature of the NaVILA policy, where the
history advances by exactly one frame per completed primitive.

Subscribes
----------
    <image_topic>               (sensor_msgs/CompressedImage)
        Camera observations. Default: /zed/rgb/color/rect/image/compressed

    <goal_topic>                (std_msgs/String)
        Natural-language navigation instruction. Receiving a new goal arms
        the loop and clears the frame history.
        Default: /goal_instruction

    <reset_topic>               (std_msgs/Empty)
        Disarms the loop and clears the frame history without sending a new goal.
        Default: /navila/reset

    <status_topic>              (std_msgs/String)
        Primitive completion signal from action_node.
        Accepted values: 'done' | 'aborted'
        Default: /navila/primitive_status

Publishes
---------
    <action_topic>              (std_msgs/String)
        Primitive command in the format '<action> <value> <unit>'.
        Examples: 'forward 25 cm', 'turn_left 15 deg', 'stop'
        Default: /navila/action

Parameters
----------
    model_path              str     Path to the NaVILA checkpoint directory.
                                    Default: $NAVILA_MODEL_PATH or '/models'
    num_video_frames        int     Number of frames per inference step
                                    (7 historical + 1 current). Must match
                                    the checkpoint config. Default: 8
    max_history_frames      int     Maximum depth of the frame history deque.
                                    Default: 512
    frame_wait_timeout_sec  float   Maximum time to wait for a fresh frame
                                    after a primitive completes. Default: 1.0
    frame_settle_sec        float   Settling margin after motion before
                                    grabbing the observation frame. Default: 0.0
    input_color_order       str     Channel order after cv2.imdecode:
                                    'bgr' (convert to RGB) or 'rgb' (keep).
                                    Note: cv2.imdecode always returns BGR
                                    regardless of the topic encoding.
                                    Default: 'bgr'
    image_topic             str     See Subscribes above.
    goal_topic              str     See Subscribes above.
    reset_topic             str     See Subscribes above.
    action_topic            str     See Publishes above.
    status_topic            str     See Subscribes above.

Inference pipeline
------------------
    1. On 'done'/'aborted' from action_node, _kick_drive() is called.
    2. _drive_thread() grabs a fresh frame (waiting up to frame_wait_timeout_sec).
    3. If the action queue is non-empty, the next queued primitive is replayed
       without running inference (replicating NaVILA's queue_actions).
    4. If the queue is empty, run_navila_inference() is called with the current
       frame history sampled via _sample_history() (faithful replica of
       sample_and_pad_images from the official repo: black-frame padding,
       endpoint=False linspace, integer indices).
    5. parse_navila_output() extracts action, value and unit using the official
       regex patterns and quantizes to the canonical magnitudes (25 cm / 15 deg).
    6. _expand_primitives() splits the magnitude into N unit primitives;
       the first is published immediately, the rest are queued.
    7. On 'aborted', the queue is cleared and the decision frame is discarded
       (motion did not occur, so it must not enter the history).

Debug
-----
    Each inference step saves a JPEG frame with action overlay to:
        /home/ros_ws/debug_frames/<timestamp>_<action>.jpg
    Useful to verify the image the model receives and the decision it produces.

Notes
-----
    - The model is loaded in a background thread so ROS spin is never blocked.
    - num_video_frames is read from model.config at load time and overrides
      the ROS parameter if different (the checkpoint is authoritative).
    - The frame history gate uses time.monotonic() and a local frame counter,
      never header.stamp, making it robust to the dual-machine ZED/inference
      setup where clocks may differ.
"""

# =============================================================================
# Mock deepspeed — required only for training, not inference.
# Avoids import errors on environments without the full CUDA dev toolkit.
# =============================================================================
import sys
import time
from pathlib import Path
from unittest.mock import MagicMock

_mock_ds = MagicMock()
_mock_ds.__spec__ = "deepspeed"
_mock_ds.__version__ = "0.0.0"
for _mod in [
    "deepspeed",
    "deepspeed.comm",
    "deepspeed.runtime",
    "deepspeed.runtime.zero",
    "deepspeed.runtime.zero.partition_parameters",
    "deepspeed.runtime.activation_checkpointing",
    "deepspeed.runtime.activation_checkpointing.checkpointing",
]:
    sys.modules[_mod] = _mock_ds
# =============================================================================

import os
import re
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Empty

import cv2
from PIL import Image as PILImage
import numpy as np

from collections import deque

# Official patterns
_OFFICIAL_PATTERNS = {
    "stop":       re.compile(r"\bstop\b", re.IGNORECASE),
    "forward":    re.compile(r"\bis move forward\b", re.IGNORECASE),
    "turn_left":  re.compile(r"\bis turn left\b", re.IGNORECASE),
    "turn_right": re.compile(r"\bis turn right\b", re.IGNORECASE),
}

def parse_navila_output(text: str):
    """Ritorna (action, value, unit) come da repo ufficiale.
    action ∈ {stop, forward, turn_left, turn_right}."""
    action = None
    for name, pat in _OFFICIAL_PATTERNS.items():
        if pat.search(text):
            action = name
            break
    if action is None:
        action = "stop"   # default ufficiale

    if action == "forward":
        m = re.search(r"move forward (\d+) cm", text)
        d = int(m.group(1)) if m else 25
        if d % 25 != 0:
            d = min([25, 50, 75], key=lambda x: abs(x - d))
        return "forward", d, "cm"
    if action == "turn_left":
        m = re.search(r"turn left (\d+) degree", text)
        g = int(m.group(1)) if m else 15
        if g % 15 != 0:
            g = min([15, 30, 45], key=lambda x: abs(x - g))
        return "turn_left", g, "deg"
    if action == "turn_right":
        m = re.search(r"turn right (\d+) degree", text)
        g = int(m.group(1)) if m else 15
        if g % 15 != 0:
            g = min([15, 30, 45], key=lambda x: abs(x - g))
        return "turn_right", g, "deg"
    return "stop", 0, ""


# =============================================================================
# NaVILA model loader
# =============================================================================

def load_navila_model(model_path: str):
    import torch
    from llava.model.builder import load_pretrained_model
    from llava.mm_utils import get_model_name_from_path
    from huggingface_hub import snapshot_download

    HF_MODEL_ID = "a8cheng/navila-llama3-8b-8f"

    torch.cuda.empty_cache()

    # DS_ACCELERATOR=cpu affects only DeepSpeed (mocked); PyTorch uses CUDA normally.
    os.environ["DS_SKIP_CUDA_CHECK"] = "1"
    os.environ["DS_ACCELERATOR"]     = "cpu"

    if not os.path.exists(os.path.join(model_path, "config.json")):
        print(f"[NaVILA] Downloading model from HuggingFace: {HF_MODEL_ID}")
        snapshot_download(
            repo_id=HF_MODEL_ID,
            local_dir=model_path,
            local_dir_use_symlinks=False,
        )
        print(f"[NaVILA] Model saved to: {model_path}")
    else:
        print(f"[NaVILA] Model found at: {model_path}")

    model_name = get_model_name_from_path(model_path)
    tokenizer, model, image_processor, context_len = load_pretrained_model(
        model_path=model_path,
        model_base=None,
        model_name=model_name,
        device_map="auto",
        offload_folder="offload",
    )

    if tokenizer.pad_token is None:
        tokenizer.pad_token = tokenizer.eos_token

    model.eval()

    llm = getattr(model, "llm", model)
    print("attn impl:", llm.config._attn_implementation)

    print(f"[NaVILA] Model device: {next(model.parameters()).device}")
    print("[NaVILA] Model loaded successfully.")
    return model, tokenizer, image_processor


# =============================================================================
# NaVILA inference
# =============================================================================

def run_navila_inference(
    model,
    tokenizer,
    image_processor,
    frames_rgb: list,
    goal: str,
    num_video_frames: int,
) -> str:
    """
    Un passo di inferenza NaVILA su una sequenza di frame (memoria + osservazione corrente).
    Ritorna il testo grezzo del modello (es. "the next action is to move forward 75 cm").
    """
    import torch
    from llava.mm_utils import process_images, tokenizer_image_token, KeywordsStoppingCriteria
    from llava.constants import IMAGE_TOKEN_INDEX
    from llava.conversation import conv_templates, SeparatorStyle

    # Lista di PIL → tensore impilato (N, C, H, W). N DEVE essere == num_video_frames.
    pil_images = [PILImage.fromarray(f) for f in frames_rgb]
    image_tensor = process_images(pil_images, image_processor, model.config)
    image_tensor = image_tensor.to(dtype=torch.float16, device="cuda")

    conv = conv_templates["llama_3"].copy()
    image_token = "<image>\n"

    # Prompt ufficiale NaVILA: storico + osservazione corrente.
    qs = (
        f"Imagine you are a robot programmed for navigation tasks. You have been given a video "
        f"of historical observations {image_token * (num_video_frames - 1)}, and current observation <image>\n. "
        f"Your assigned task is: \"{goal}\" "
        f"Analyze this series of images to decide your next action, which could be turning left or right by a specific "
        f"degree, moving forward a certain distance, or stop if the task is completed."
    )

    conv.append_message(conv.roles[0], qs)
    conv.append_message(conv.roles[1], None)
    prompt_text = conv.get_prompt()

    input_ids = tokenizer_image_token(
        prompt_text, tokenizer, IMAGE_TOKEN_INDEX, return_tensors="pt"
    ).unsqueeze(0).to("cuda")

    stop_str = conv.sep if conv.sep_style != SeparatorStyle.TWO else conv.sep2
    stopping_criteria = KeywordsStoppingCriteria([stop_str], tokenizer, input_ids)

    with torch.inference_mode():
        output_ids = model.generate(
            input_ids,
            images=image_tensor,
            do_sample=False,
            temperature=0.0,
            num_beams=1,
            max_new_tokens=32,
            use_cache=True,
            pad_token_id=tokenizer.eos_token_id,
            stopping_criteria=[stopping_criteria],
        )

    raw_output = tokenizer.decode(output_ids[0], skip_special_tokens=True).strip().lower()
    return raw_output


# =============================================================================
# ROS 2 Node
# =============================================================================

class NaViLANode(Node):

    def __init__(self):
        super().__init__("navila_super_node")

        # ------------------------------------------------------------------
        # ROS 2 parameters
        # ------------------------------------------------------------------
        self.declare_parameter("model_path", os.environ.get("NAVILA_MODEL_PATH", "/models"))
        self.declare_parameter("num_video_frames", 8) # default N=8 frames
        self.declare_parameter("max_history_frames", 512) 
        self.declare_parameter("frame_wait_timeout_sec", 1.0) 
        self.declare_parameter("frame_settle_sec", 0.0)        
        self.declare_parameter("input_color_order", "bgr")

        self.declare_parameter("image_topic",  "/zed/rgb/color/rect/image/compressed")
        self.declare_parameter("goal_topic",   "/goal_instruction")

        self.declare_parameter("action_topic", "/navila/action")
        self.declare_parameter("reset_topic",  "/navila/reset")
        self.declare_parameter("status_topic",   "/navila/primitive_status")

        def p(name):
            return self.get_parameter(name).value

        model_path        = p("model_path")
        self._num_video_frames = p("num_video_frames")
        max_history_frames = p("max_history_frames")
        self._frame_wait_timeout = p("frame_wait_timeout_sec")
        self._frame_settle       = p("frame_settle_sec")


        self._input_color_order = str(p("input_color_order")).strip().lower()
        if self._input_color_order not in ("bgr", "rgb"):
            self.get_logger().warn(f"input_color_order='{self._input_color_order}' not validido → use 'bgr'")
            self._input_color_order = "bgr"
        self.get_logger().info(f"input_color_order = {self._input_color_order}")

        image_topic       = p("image_topic")
        goal_topic        = p("goal_topic")
        action_topic      = p("action_topic")
        reset_topic       = p("reset_topic")
        status_topic        = p("status_topic")

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self._last_image_msg = None 
        self._frame_history = deque(maxlen=max_history_frames)
        self._frame_seq        = 0
        self._motion_done_seq  = 0
        self._motion_done_mono = time.monotonic()

        self._last_decision_frame   = None
        self._queue                 = []
        self._active                = False
        self._cycle_active          = False

        self.last_goal     = ""
        self.model         = None
        self.tokenizer     = None
        self.image_proc    = None
        self._model_ready  = False
        self._lock         = threading.Lock()

        # ------------------------------------------------------------------
        # Subscribers
        # ----------------------------------------------------------------]]--
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub_image = self.create_subscription(
            CompressedImage,
            image_topic,
            self._image_cb,
            qos_sensor)

        self.sub_goal  = self.create_subscription(
            String, 
            goal_topic, 
            self._goal_cb, 10)
        
        self.sub_reset = self.create_subscription(
            Empty,
            reset_topic,
            self._reset_cb, 10)
        
        self.sub_status = self.create_subscription(
            String, 
            status_topic, 
            self._primitive_status_cb, 10)

        

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self.pub_action = self.create_publisher(String, action_topic, 10)
        self.pub_status = self.create_publisher(String, status_topic, 10)


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
            f"  input_color_order : {self._input_color_order}\n"
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

    def _process_image(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)   # BGR diretto
            if frame is None:
                raise ValueError("cv2.imdecode returned None — corrupted frame?")
            if self._input_color_order == "bgr":
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return frame
        except Exception as exc:
            self.get_logger().warn(f"Image conversion error: {exc}")
            return None

    def _primitive_status_cb(self, msg: String):
        status = msg.data.strip().lower()
        with self._lock:
            if status in ("done", "aborted"):
                self._motion_done_seq = self._frame_seq
                self._motion_done_mono = time.monotonic()
            if status == "aborted":
                self._queue = []                  # coda invalidata dall'ostacolo
                self._last_decision_frame = None  # moto non avvenuto → fuori dallo storico
                self.get_logger().warn("Primitiva ABORTED → coda svuotata, frame scartato")
            # su 'done' non tocco coda né _last_decision_frame:
            #   il frame verrà promosso nello storico dal prossimo _drive_thread
            self._cycle_active = False
        self._kick_drive()

    def _goal_cb(self, msg: String):
        with self._lock:
            self.last_goal = msg.data
            self._frame_history.clear()
            self._last_decision_frame = None
            self._queue = []
            self._active = True
            self._cycle_active = False
            self._motion_done_seq  = self._frame_seq
            self._motion_done_mono = time.monotonic()
        self.get_logger().info(f"New goal: '{msg.data}' (loop armed)")
        self._kick_drive()

    def _reset_cb(self, msg: Empty):
        with self._lock:
            self.last_goal = ""
            self._active = False
            self._cycle_active = False
            self._frame_history.clear()
            self._last_decision_frame = None
            self._queue = []
        self.get_logger().info("NaVILA reset (loop disarmed).")

    # ------------------------------------------------------------------
    # Model loading (background thread)
    # ------------------------------------------------------------------

    def _load_model_thread(self, model_path: str):
        try:
            # <--- NaVILA --->
            self.get_logger().info("Loading NaVILA model...")
            model, tokenizer, image_proc = load_navila_model(model_path)
            self.get_logger().info("NaVILA model loaded successfully.")

            cfg_nvf = getattr(model.config, "num_video_frames", None)
            # --- Ready ---
            with self._lock:
                self.model        = model
                self.tokenizer    = tokenizer
                self.image_proc   = image_proc
                
                if cfg_nvf is None:
                    self.get_logger().warn(f"model.config.num_video_frames missing — keep ROS param ({self._num_video_frames}).")
                elif cfg_nvf != self._num_video_frames:
                    self.get_logger().warn(f"num_video_frames: param={self._num_video_frames} - checkpoint={cfg_nvf} → using checkpoint.")
                    self._num_video_frames = cfg_nvf
                else:
                    self._num_video_frames = cfg_nvf
                self.get_logger().info(f"num_video_frames correct = {self._num_video_frames}")
                self._model_ready = True

            self.get_logger().info(f"NaVILA ready — action parser: REGEX PARSER")
            # Waiting for topics
            self.get_logger().info("Waiting for camera frame...")
            while rclpy.ok():
                with self._lock:
                    has_frame = self._last_image_msg is not None
                if has_frame:
                    break
                self.get_logger().info("Waiting for camera frame...", throttle_duration_sec=5.0)
                time.sleep(0.5)
            self.get_logger().info("Camera frame received")
            # Wait for goal instruction
            while rclpy.ok():
                with self._lock:
                    has_goal = bool(self.last_goal)
                if has_goal:
                    break
                self.get_logger().info(
                    "Waiting for goal instruction on "
                    f"'{self.get_parameter('goal_topic').value}'...",
                    throttle_duration_sec=5.0)
                time.sleep(0.5)
            self.get_logger().info(f"Goal received: '{self.last_goal}' — starting inference loop ✓")

        except Exception as exc:
            self.get_logger().error(f"Failed to load NaVILA model: {exc}")

    # ------------------------------------------------------------------
    # Inference callback 09 / 06 / 2026 - versione con padding + debug
    # ------------------------------------------------------------------
    def _kick_drive(self):
        with self._lock:
            if self._cycle_active:
                return
            if not self._model_ready:
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
                if self._last_decision_frame is not None:
                    self._frame_history.append(self._last_decision_frame)
                    self._last_decision_frame = None
                goal      = self.last_goal
                model, tok, iproc = self.model, self.tokenizer, self.image_proc
                queued = self._queue.pop(0) if self._queue else None
                after_seq  = self._motion_done_seq
                after_mono = self._motion_done_mono
                settle     = self._frame_settle
                timeout    = self._frame_wait_timeout

            image_msg, stale = self._wait_fresh_frame(after_seq, after_mono, settle, timeout)
            if stale:
                self.get_logger().warn(
                    "Nessun frame fresco entro il timeout — uso l'ultimo disponibile.",
                    throttle_duration_sec=5.0)

            curr = self._process_image(image_msg)
            if curr is None:
                with self._lock:
                    self._cycle_active = False
                return

            if queued is not None:
                cmd = queued
                with self._lock:
                    self._last_decision_frame = curr
                self.get_logger().info(
                    f"[queue] {cmd}  (resto coda:{len(self._queue)}, hist:{len(self._frame_history)})")
                self._save_debug_frame(curr, cmd, "[queued]", goal)
                out = String(); out.data = cmd
                self.pub_action.publish(out)
                return 
             
            # Inference
            frames = self._sample_history(list(self._frame_history) + [curr], self._num_video_frames)

            self.get_logger().info(f"[inference] goal='{goal}'  hist={len(self._frame_history)} frames")

            raw_output = run_navila_inference(model, tok, iproc, frames, goal, self._num_video_frames)
            action, value, unit = parse_navila_output(raw_output)
            cmd, n_total = self._expand_primitives(action, value)

            self.get_logger().info(f"[inference] raw='{raw_output}' → action='{action}' "f"value={value}{unit} → cmd='{cmd}' ×{n_total}")
            #- - - - - - - - - -

            if action == "stop":
                with self._lock:
                    self._last_decision_frame = curr
                    self._active = False
                    self._cycle_active = False
                self.get_logger().info(f"raw='{raw_output}' → STOP")
                self.get_logger().info(
                    f"\n{'='*60}\n"
                    f"  MISSION COMPLETE\n"
                    f"  goal   : '{goal}'\n"
                    f"  history: {len(self._frame_history)} frames\n"
                    f"{'='*60}"
                )
                self._save_debug_frame(curr, "stop", raw_output, goal)
                done_msg = String(); done_msg.data = "complete"
                self.pub_status.publish(done_msg)
                done_msg = String(); done_msg.data = "complete"
                self.pub_status_out.publish(done_msg)
                return

            with self._lock:
                self._last_decision_frame = curr
                self._queue = [cmd] * (n_total - 1)  

            self.get_logger().info(
                f"raw='{raw_output}' → {cmd} ×{n_total}  "
                f"(accodate:{n_total - 1}, hist:{len(self._frame_history)})")
            self._save_debug_frame(curr, cmd, raw_output, goal)
            out = String(); out.data = cmd
            self.pub_action.publish(out)
            

        except Exception as exc:
            self.get_logger().error(f"Drive error: {exc}")
            with self._lock:
                self._cycle_active = False

# =============================================================================
# HELPER METHODS
# =============================================================================
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

    @staticmethod
    def _sample_history(history, num_frames, pad_h=512, pad_w=512):
        """Faithful replica of sample_and_pad_images from the official NaVILA repo.

        Pads the frame list with black frames at the front when the history is
        shorter than num_frames, then samples num_frames-1 uniformly spaced
        indices (endpoint=False, integer linspace) and appends the latest frame.

        This exactly replicates the official evaluation loop in navila_trainer.py:
            - Black PIL/numpy padding at the front (not the back).
            - np.linspace with endpoint=False and dtype=int for index selection.
            - The current (most recent) frame is always the last element.

        Args:
            history   : list of RGB numpy frames, oldest → newest.
                        The last element is the current observation.
            num_frames: total number of frames to return. Must equal
                        model.config.num_video_frames (typically 8).
            pad_h     : height of black padding frames (px). Default 512.
            pad_w     : width of black padding frames (px). Default 512.

        Returns:
            List of num_frames RGB numpy arrays: [hist_0, ..., hist_N-2, current].
        """
        frames = list(history)
        while len(frames) < num_frames:
            frames.insert(0, np.zeros((pad_h, pad_w, 3), dtype=np.uint8))
        latest = frames[-1]
        idxs = np.linspace(0, len(frames) - 1, num=num_frames - 1, endpoint=False, dtype=int)
        return [frames[i] for i in idxs] + [latest]

    @staticmethod
    def _expand_primitives(action, value):
        """(action, value) → (cmd_primitiva, n_totale_primitive), come da repo.
        forward: step da 25 cm; turn: step da 15°."""
        if action == "forward":
            n = max(1, int(value) // 25)
            return "forward 25 cm", n
        if action == "turn_left":
            n = max(1, int(value) // 15)
            return "turn_left 15 deg", n
        if action == "turn_right":
            n = max(1, int(value) // 15)
            return "turn_right 15 deg", n
        return "stop", 0   # stop
    

# =============================================================================
# Debug
    def _save_debug_frame(self, frame_rgb: np.ndarray, action: str, raw_output: str, goal: str):
        """Save the inference frame with action overlay for debugging."""
        try:
            debug_dir = Path("/home/ros_ws/debug_frames")
            debug_dir.mkdir(exist_ok=True)

            # Converti RGB → BGR per OpenCV
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            # Overlay testo
            timestamp = time.strftime("%H:%M:%S")
            texts = [
                f"ACTION: {action}",
                f"GOAL: {goal[:50]}",
                f"RAW: {raw_output[:60]}",
                f"TIME: {timestamp}",
                f"HISTORY: {len(self._frame_history)} frames",
            ]

            # Sfondo semitrasparente per leggibilità
            overlay = frame_bgr.copy()
            cv2.rectangle(overlay, (0, 0), (frame_bgr.shape[1], 30 + len(texts) * 28), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, frame_bgr, 0.4, 0, frame_bgr)

            # Scrivi testo
            for i, text in enumerate(texts):
                color = (0, 255, 0) if i == 0 else (255, 255, 255)  # action in verde
                cv2.putText(frame_bgr, text, (10, 25 + i * 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

            # Salva con timestamp + action come nome file
            filename = debug_dir / f"{int(time.time()*1000)}_{action}.jpg"
            cv2.imwrite(str(filename), frame_bgr)

        except Exception as e:
            self.get_logger().warn(f"Debug frame save error: {e}", throttle_duration_sec=5.0)

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
