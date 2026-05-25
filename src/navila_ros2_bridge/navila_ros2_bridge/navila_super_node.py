#!/usr/bin/env python3
"""
navila_node.py
ROS 2 bridge between the NaVILA VLA model and a wheeled robot (e.g. Husky).

Subscribes:
    /camera/image_raw   (sensor_msgs/Image)
    /goal_instruction   (std_msgs/String)
    /odom               (nav_msgs/Odometry)

Publishes:
    /navila/action      (std_msgs/String)
        One of: forward | forward_fast | backward |
                turn_left | turn_right | curve_left | curve_right | stop

Action resolution pipeline (cascading fallback):
    NaVILA raw text
        └─► Phi-3-mini classifier   (if loaded)
                └─► regex parser    (fallback / always available)
                        └─► "stop"  (conservative default)
"""

# =============================================================================
# Mock deepspeed — required only for training, not inference.
# Avoids import errors on environments without the full CUDA dev toolkit.
# =============================================================================
import sys
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
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as PILImage


# =============================================================================
# Action vocabulary — must match action_to_cmdvel_node._action_map keys
# =============================================================================

VALID_ACTIONS = [
    "forward", "forward_fast", "backward",
    "turn_left", "turn_right",
    "curve_left", "curve_right",
    "stop",
]


# =============================================================================
# Regex-based action parser
# =============================================================================

# Each entry: (regex_pattern, weight)
# Patterns are evaluated with re.search() against the lowercased NaVILA output.
# The action with the highest cumulative score wins.
# Word boundaries (\b) prevent false positives like "alright" → "right".

_ACTION_PATTERNS: dict[str, list[tuple[str, int]]] = {
    # --- fast forward (checked before plain forward) ---
    "forward_fast": [
        (r"\bfast\b",               3),
        (r"\bquickly\b",            3),
        (r"\bspeed up\b",           3),
        (r"\bfull speed\b",         4),
        (r"\brapidly\b",            2),
        (r"\brun\b",                2),
        (r"\bhurry\b",              2),
        (r"\bacceler\w*\b",         2),   # accelerate / acceleration
    ],
    # --- backward ---
    "backward": [
        (r"\bbackward[s]?\b",       3),
        (r"\bback up\b",            3),
        (r"\breverse\b",            3),
        (r"\bretreat\b",            2),
        (r"\bback\b",               1),   # weak — "back" alone is ambiguous
    ],
    # --- curve left/right (higher weight than plain turn) ---
    "curve_left": [
        (r"\bcurve left\b",         4),
        (r"\bear left\b",           4),
        (r"\bveer left\b",          4),
        (r"\bslightly left\b",      4),
        (r"\bbear to the left\b",   4),
        (r"\bdiagonal.*left\b",     3),
    ],
    "curve_right": [
        (r"\bcurve right\b",        4),
        (r"\bear right\b",          4),
        (r"\bveer right\b",         4),
        (r"\bslightly right\b",     4),
        (r"\bbear to the right\b",  4),
        (r"\bdiagonal.*right\b",    3),
    ],
    # --- in-place turns ---
    "turn_left": [
        (r"\bturn left\b",          3),
        (r"\brotate left\b",        3),
        (r"\bspin left\b",          3),
        (r"\bleft\b",               1),
    ],
    "turn_right": [
        (r"\bturn right\b",         3),
        (r"\brotate right\b",       3),
        (r"\bspin right\b",         3),
        (r"\bright\b",              1),
    ],
    # --- forward ---
    "forward": [
        (r"\bforward\b",            2),
        (r"\bstraight\b",           2),
        (r"\bproceed\b",            1),
        (r"\bcontinue\b",           1),
        (r"\badvance\b",            1),
        (r"\bmove ahead\b",         2),
    ],
    # --- stop ---
    "stop": [
        (r"\bstop\b",               3),
        (r"\bhalt\b",               3),
        (r"\bwait\b",               2),
        (r"\bdo not move\b",        3),
        (r"\bstay\b",               1),
        (r"\bfreeze\b",             3),
        (r"\bstand still\b",        3),
    ],
}

# Pre-compile all patterns for performance
_COMPILED_PATTERNS: dict[str, list[tuple[re.Pattern, int]]] = {
    action: [(re.compile(pat), w) for pat, w in signals]
    for action, signals in _ACTION_PATTERNS.items()
}

# Forward signal patterns reused for forward_fast guard
_FORWARD_SIGNALS = [p for p, _ in _COMPILED_PATTERNS["forward"]]


def parse_navila_output(text: str) -> str:
    """
    Map free-form NaVILA output text to a valid action token using
    weighted regex scoring.

    Args:
        text: raw lowercased string produced by NaVILA

    Returns:
        One of VALID_ACTIONS; defaults to "stop" if no pattern matches.
    """
    scores: dict[str, int] = {action: 0 for action in _COMPILED_PATTERNS}

    for action, signals in _COMPILED_PATTERNS.items():
        for pattern, weight in signals:
            if pattern.search(text):
                scores[action] += weight

    # Guard: forward_fast requires at least one plain-forward signal.
    # Without it, words like "run" or "fast" could match unrelated sentences.
    if scores["forward_fast"] > 0:
        has_forward = any(p.search(text) for p in _FORWARD_SIGNALS)
        if not has_forward:
            scores["forward_fast"] = 0

    best_action = max(scores, key=lambda a: scores[a])
    return best_action if scores[best_action] > 0 else "stop"


# =============================================================================
# Phi-3-mini classifier (optional — loaded only if use_phi3=True)
# =============================================================================

_PHI3_SYSTEM_PROMPT = f"""\
You are a robot navigation action classifier.
Given a navigation instruction, respond with exactly one word from this list:
{", ".join(VALID_ACTIONS)}

Definitions:
- forward:      move straight ahead at normal speed
- forward_fast: move straight ahead quickly
- backward:     move backwards / reverse
- turn_left:    rotate left in place (no forward motion)
- turn_right:   rotate right in place (no forward motion)
- curve_left:   move forward while steering left
- curve_right:  move forward while steering right
- stop:         do not move

Respond with ONLY the action word. No explanation. No punctuation.\
"""


class Phi3Classifier:
    """
    Lightweight Phi-3-mini-based action classifier.
    Used as the primary parser when available; regex is always the fallback.
    """

    def __init__(self, device: str = "auto", use_4bit: bool = False, model_path: str = "/models/phi3mini"):
        import torch
        from transformers import (
            AutoTokenizer,
            AutoModelForCausalLM,
            #BitsAndBytesConfig,
        )
        from huggingface_hub import snapshot_download

        model_id = "microsoft/Phi-3-mini-4k-instruct"

        

        # --- download if not found ---
        if not os.path.exists(os.path.join(model_path, "config.json")):
            print(f"[Phi3Classifier] Downloading {model_id} to {model_path} ...")
            snapshot_download(
                repo_id=model_id,
                local_dir=model_path,
            )
            print(f"[Phi3Classifier] Model saved to: {model_path}")
        else:
            print(f"[Phi3Classifier] Model found at: {model_path}")
        # --- --- --- --- --- --- --- ---

        print(f"[Phi3Classifier] Loading {model_id}...")

        # print(f"[Phi3Classifier] Loading {model_id} (4bit={use_4bit}) ...")

        # quant_cfg = None
        # if use_4bit:
        #     quant_cfg = BitsAndBytesConfig(
        #         load_in_4bit=True,
        #         bnb_4bit_compute_dtype=torch.float16,
        #         bnb_4bit_use_double_quant=True,
        #     )

        self.tokenizer = AutoTokenizer.from_pretrained(model_id)
        self.model = AutoModelForCausalLM.from_pretrained(
            model_id,
            #quantization_config=quant_cfg,
            device_map=device,
            trust_remote_code=True,
            torch_dtype=torch.float16,
        )
        self.model.eval()
        print("[Phi3Classifier] Ready.")

    def classify(
        self,
        navila_output: str,
        fallback_fn: Optional[Callable[[str], str]] = None,
    ) -> str:
        """
        Classify raw NaVILA output into a valid action token.

        Args:
            navila_output: raw text from NaVILA
            fallback_fn:   called when Phi-3 produces an out-of-vocabulary token

        Returns:
            A valid action string from VALID_ACTIONS.
        """
        import torch

        messages = [
            {"role": "system", "content": _PHI3_SYSTEM_PROMPT},
            {"role": "user",   "content": navila_output},
        ]

        input_ids = self.tokenizer.apply_chat_template(
            messages,
            add_generation_prompt=True,
            return_tensors="pt",
        ).to(self.model.device)

        with torch.inference_mode():
            output_ids = self.model.generate(
                input_ids,
                max_new_tokens=8,
                do_sample=False,
                temperature=None,
                top_p=None,
                pad_token_id=self.tokenizer.eos_token_id,
            )

        new_tokens = output_ids[0][input_ids.shape[-1]:]
        result = self.tokenizer.decode(new_tokens, skip_special_tokens=True)
        result = result.strip().lower().split()[0] if result.strip() else ""

        if result in VALID_ACTIONS:
            return result

        # Out-of-vocabulary → fallback
        if fallback_fn is not None:
            return fallback_fn(navila_output)

        return "stop"


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
    frame_rgb: np.ndarray,
    goal: str,
) -> tuple[str, str]:
    """
    Run one NaVILA inference step.

    Args:
        frame_rgb:  numpy array (H, W, 3) in RGB
        goal:       natural-language navigation instruction

    Returns:
        (action, raw_output) — action is a VALID_ACTIONS token;
        raw_output is the unprocessed model text (useful for logging/debug).
    """
    import torch
    from llava.mm_utils import process_images, tokenizer_image_token
    from llava.constants import IMAGE_TOKEN_INDEX, DEFAULT_IMAGE_TOKEN
    from llava.conversation import conv_templates, SeparatorStyle
    from llava.mm_utils import KeywordsStoppingCriteria

    pil_img = PILImage.fromarray(frame_rgb)
    image_tensor = process_images([pil_img], image_processor, model.config)
    image_tensor = image_tensor.to(dtype=torch.float16, device="cuda")

    conv = conv_templates["llama_3"].copy()
    image_token = "<image>\n"

    qs = (
        f"Imagine you are a robot programmed for navigation tasks. "
        f"You have been given a current observation <image>\n. "
        f'Your assigned task is: "{goal}" '
        f"Analyze this image to decide your next action, which could be turning left or right by a specific "
        f"degree, moving forward a certain distance, or stop if the task is completed."
    )

    conv.append_message(conv.roles[0], qs)
    conv.append_message(conv.roles[1], None)
    prompt_text = conv.get_prompt()

    # print(f"Prompt: {prompt_text}")

    input_ids = tokenizer_image_token(
        prompt_text, tokenizer, IMAGE_TOKEN_INDEX, return_tensors="pt"
    ).unsqueeze(0).to("cuda")

    attention_mask = torch.ones_like(input_ids)

    stop_str = conv.sep if conv.sep_style != SeparatorStyle.TWO else conv.sep2
    stopping_criteria = KeywordsStoppingCriteria([stop_str], tokenizer, input_ids)


    with torch.inference_mode():
        output_ids = model.generate(
            input_ids,
            attention_mask=attention_mask,
            images=[image_tensor],
            do_sample=False,
            temperature=1.0,
            top_p=1.0,
            num_beams=1,
            max_new_tokens=64,
            use_cache=True,
            pad_token_id=tokenizer.eos_token_id,
            stopping_criteria=[stopping_criteria],
        )

    raw_output = tokenizer.decode(output_ids[0], skip_special_tokens=True).strip().lower()

    # Action resolution is handled by the caller (NaViLANode._inference_cb)
    # so that the classifier / fallback chain is applied there.
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
        self.declare_parameter("phi3_model_path", "/models/phi3mini")
        self.declare_parameter("inference_rate_hz", 2.0)

        self.declare_parameter("image_topic",  "/sensors/front_camera/color/image_raw")
        self.declare_parameter("goal_topic",   "/goal_instruction")
        self.declare_parameter("odom_topic",   "/platform/odom")
        self.declare_parameter("action_topic", "/navila/action")

        self.declare_parameter("use_phi3",     False)   # enable Phi-3 classifier
        self.declare_parameter("phi3_4bit",    True)    # quantize Phi-3 to 4-bit

        def p(name):
            return self.get_parameter(name).value

        model_path        = p("model_path")
        inference_rate_hz = p("inference_rate_hz")
        image_topic       = p("image_topic")
        goal_topic        = p("goal_topic")
        odom_topic        = p("odom_topic")
        action_topic      = p("action_topic")
        self._use_phi3    = p("use_phi3")
        self._phi3_4bit   = p("phi3_4bit")

        self._inference_running = False

        # self.declare_parameter("use_sim_time", True)

        self.get_logger().info(f"use_phi3 = {self.get_parameter('use_phi3').value}")

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self.bridge        = CvBridge()
        self.last_frame    = None       # numpy RGB
        self.last_goal     = ""
        self.last_odom     = None
        self.model         = None
        self.tokenizer     = None
        self.image_proc    = None
        self.classifier: Optional[Phi3Classifier] = None
        self._model_ready  = False
        self._lock         = threading.Lock()

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub_image = self.create_subscription(
            Image, 
            image_topic, 
            self._image_cb, 
            qos_sensor)

        self.sub_goal  = self.create_subscription(
            String, 
            goal_topic, 
            self._goal_cb, 10)

        self.sub_odom  = self.create_subscription(
            Odometry, 
            odom_topic, 
            self._odom_cb, 
            qos_sensor)

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self.pub_action = self.create_publisher(String, action_topic, 10)

        # ------------------------------------------------------------------
        # Inference timer
        # ------------------------------------------------------------------
        self.timer = self.create_timer(1.0 / inference_rate_hz, self._inference_cb)

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

    def _image_cb(self, msg: Image):
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            with self._lock:
                self.last_frame = frame_rgb
        except Exception as exc:
            self.get_logger().warn(f"Image conversion error: {exc}")

    def _goal_cb(self, msg: String):
        with self._lock:
            self.last_goal = msg.data
        self.get_logger().info(f"New goal received: '{msg.data}'")

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self.last_odom = msg

    # ------------------------------------------------------------------
    # Model loading (background thread)
    # ------------------------------------------------------------------

    def _load_model_thread(self, model_path: str):
        try:
            # <--- NaVILA --->
            self.get_logger().info("Loading NaVILA model...")
            model, tokenizer, image_proc = load_navila_model(model_path)
            self.get_logger().info("NaVILA model loaded successfully.")


            # <--- Phi3 --->
            classifier = None
            if self._use_phi3:
                self.get_logger().info(
                f"use_phi3=True — loading Phi-3-mini "
                f"(4bit={self._phi3_4bit})..."
                )
                try:
                    classifier = Phi3Classifier(
                        device="auto",
                        use_4bit=self._phi3_4bit,
                        model_path=self.get_parameter("phi3_model_path").value,
                    )
                    self.get_logger().info("Phi-3-mini classifier loaded successfully.")
                except Exception as exc:
                    self.get_logger().error(
                        f"Phi-3 classifier FAILED: {exc}\n"
                        f"Falling back to regex parser."
                    )
            else:
                self.get_logger().info("use_phi3=False — skipping Phi-3 classifier.")

            # --- Ready ---
            with self._lock:
                self.model        = model
                self.tokenizer    = tokenizer
                self.image_proc   = image_proc
                self.classifier   = classifier
                self._model_ready = True

            mode = "Phi-3 + regex fallback" if classifier else "regex parser"
            self.get_logger().info(
                f"NaVILA ready — action parser: {mode}"
            )

        except Exception as exc:
            self.get_logger().error(f"Failed to load NaVILA model: {exc}")

    # ------------------------------------------------------------------
    # Inference callback
    # ------------------------------------------------------------------

    def _inference_cb(self):

        if self._inference_running: # if previous inference is still running, skip this cycle
            return
    
        with self._lock:
            ready      = self._model_ready
            frame      = self.last_frame
            goal       = self.last_goal
            model      = self.model
            tok        = self.tokenizer
            iproc      = self.image_proc
            classifier = self.classifier

        if not ready:
            self.get_logger().info(
                "Waiting for model to load...", throttle_duration_sec=30.0)
            return
        if frame is None:
            self.get_logger().info(
                "Waiting for camera frame...", throttle_duration_sec=30.0)
            return
        if not goal:
            self.get_logger().info(
                "Waiting for goal instruction...", throttle_duration_sec=30.0)
            return

        #--- Run inference in a separate thread to avoid blocking the ROS 2 timer ---
        threading.Thread(
            target=self._run_inference_thread,
            args=(model, tok, iproc, frame, goal, classifier),
            daemon=True,
        ).start()


    def _run_inference_thread(self, model, tok, iproc, frame, goal, classifier):
        self._inference_running = True
        try:
            raw_output = run_navila_inference(model, tok, iproc, frame, goal)

            # --- Action resolution: Phi-3 → regex → stop ---
            if classifier is not None:
                action = classifier.classify(raw_output, fallback_fn=parse_navila_output)
            else:
                action = parse_navila_output(raw_output)

            self.get_logger().info(f"raw='{raw_output}' → action='{action}'  (goal: '{goal}')")

            msg = String()
            msg.data = action
            self.pub_action.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"Inference error: {exc}")
        finally:
            self._inference_running = False


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