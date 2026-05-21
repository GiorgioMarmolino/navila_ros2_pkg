#!/usr/bin/env python3
"""
navila_node.py
ROS 2 bridge between NaVILA VLA model and Husky robot.

Subscribes:
    /camera/image_raw   (sensor_msgs/Image)
    /goal_instruction   (std_msgs/String)
    /odom               (nav_msgs/Odometry)

Publishes:
    /navila/action      (std_msgs/String)  — discrete action: forward/left/right/stop
"""



# =============================================================================
import sys
from unittest.mock import MagicMock

# Mock deepspeed completo — non serve per inferenza ma solo per training (non effettuato)
mock_ds = MagicMock()
mock_ds.__spec__ = "deepspeed"
mock_ds.__version__ = "0.0.0"
sys.modules['deepspeed'] = mock_ds
sys.modules['deepspeed.comm'] = mock_ds
sys.modules['deepspeed.runtime'] = mock_ds
sys.modules['deepspeed.runtime.zero'] = mock_ds
sys.modules['deepspeed.runtime.zero.partition_parameters'] = mock_ds
sys.modules['deepspeed.runtime.activation_checkpointing'] = mock_ds
sys.modules['deepspeed.runtime.activation_checkpointing.checkpointing'] = mock_ds
# =============================================================================


import os
import threading

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
# Model loader
# =============================================================================

def load_navila_model(model_path: str):
    import torch
    from transformers import BitsAndBytesConfig
    from llava.model.builder import load_pretrained_model
    from llava.mm_utils import get_model_name_from_path
    from huggingface_hub import snapshot_download

    HF_MODEL_ID = "a8cheng/navila-llama3-8b-8f"


    torch.cuda.empty_cache()
    
    #-----------------------------------------------------------------------------
    # Il modulo deepspeed non funziona correttamente con cudnn8-runtime perché mancano alcune librerie di sviluppo CUDA che deepspeed richiede anche a runtime.Il modulo deepspeed non funziona correttamente con cudnn8-runtime perché mancano alcune librerie di sviluppo CUDA che deepspeed richiede anche a runtime. DS_ACCELERATOR riguarda solo deepspeed, non PyTorch. Il modello NaVILA per l'inferenza usa PyTorch direttamente sulla GPU, deepspeed non è coinvolto.
    os.environ["DS_SKIP_CUDA_CHECK"] = "1"
    os.environ["DS_ACCELERATOR"] = "cpu" 
    #-----------------------------------------------------------------------------
    

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
        # quantization_config=bnb_config,

        device_map="auto", # cpu-cuda-auto-balanced-sequential
        offload_folder="offload",
        # max_memory={0: "6GiB", "cpu": "12GiB"},
    )

    model.eval()

    print(next(model.parameters()).device)
    print("[NaVILA] Model loaded successfully.")
    return model, tokenizer, image_processor


# =============================================================================
# Action mapping
# =============================================================================

# NaVILA outputs a discrete action token — map it to a string
ACTION_MAP = {
    0: "stop",
    1: "forward",
    2: "left",
    3: "right",
    4: "stop",   # fallback
}


def run_navila_inference(model, tokenizer, image_processor, frame_rgb, goal: str) -> str:
    """
    Esegue l'inferenza NaVILA dato un frame RGB e un goal testuale.

    Args:
        frame_rgb: numpy array (H, W, 3) in RGB
        goal:      stringa con l'istruzione di navigazione

    Returns:
        Stringa azione: "forward" | "left" | "right" | "stop"
    """
    import torch
    from llava.mm_utils import process_images, tokenizer_image_token
    from llava.constants import IMAGE_TOKEN_INDEX, DEFAULT_IMAGE_TOKEN
    from llava.conversation import conv_templates

    # Prepara immagine
    pil_img = PILImage.fromarray(frame_rgb)
    image_tensor = process_images([pil_img], image_processor, model.config)
    image_tensor = image_tensor.to(dtype=torch.float16, device="cuda")

    # Prepara prompt
    conv = conv_templates["llama_3"].copy()
    prompt = f"{DEFAULT_IMAGE_TOKEN}\n{goal}"
    conv.append_message(conv.roles[0], prompt)
    conv.append_message(conv.roles[1], None)
    prompt_text = conv.get_prompt()

    input_ids = tokenizer_image_token(
        prompt_text, tokenizer, IMAGE_TOKEN_INDEX, return_tensors="pt"
    ).unsqueeze(0).to("cuda")

    with torch.inference_mode():
        output_ids = model.generate(
            input_ids,
            images=image_tensor,
            image_sizes=[pil_img.size],
            do_sample=False,
            max_new_tokens=16,
        )

    output_text = tokenizer.decode(output_ids[0], skip_special_tokens=True).strip().lower()

    # Mappa output testuale → azione discreta
    if "forward" in output_text or "straight" in output_text:
        return "forward"
    elif "left" in output_text or "turn left" in output_text:
        return "left"
    elif "right" in output_text or "turn right" in output_text:
        return "right"
    else:
        return "stop"


# =============================================================================
# ROS 2 Node
# =============================================================================

class NaViLANode(Node):

    def __init__(self):
        super().__init__("navila_node")

        # ------------------------------------------------------------------
        # Parametri ROS 2 (sovrascrivibili da CLI o launch file)
        # ------------------------------------------------------------------
        self.declare_parameter("model_path",
                               os.environ.get("NAVILA_MODEL_PATH", "/models"))
        self.declare_parameter("inference_rate_hz", 2.0)   # inferenze al secondo
        self.declare_parameter("image_topic",  "/camera/image_raw")
        self.declare_parameter("goal_topic",   "/goal_instruction")
        self.declare_parameter("odom_topic",   "/odom")
        self.declare_parameter("action_topic", "/navila/action")

        model_path        = self.get_parameter("model_path").value
        inference_rate_hz = self.get_parameter("inference_rate_hz").value
        image_topic       = self.get_parameter("image_topic").value
        goal_topic        = self.get_parameter("goal_topic").value
        odom_topic        = self.get_parameter("odom_topic").value
        action_topic      = self.get_parameter("action_topic").value

        # ------------------------------------------------------------------
        # Stato interno
        # ------------------------------------------------------------------
        self.bridge       = CvBridge()
        self.last_frame   = None          # numpy RGB
        self.last_goal    = ""            # stringa goal
        self.last_odom    = None          # Odometry msg (disponibile per usi futuri)
        self.model        = None
        self.tokenizer    = None
        self.image_proc   = None
        self._lock        = threading.Lock()
        self._model_ready = False

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

        self.sub_goal = self.create_subscription(
            String, 
            goal_topic, 
            self._goal_cb, 
            10)

        self.sub_odom = self.create_subscription(
            Odometry, 
            odom_topic, 
            self._odom_cb, 
            qos_sensor)

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self.pub_action = self.create_publisher(String, action_topic, 10)

        # ------------------------------------------------------------------
        # Timer inferenza
        # ------------------------------------------------------------------
        period = 1.0 / inference_rate_hz
        self.timer = self.create_timer(period, self._inference_cb)

        # ------------------------------------------------------------------
        # Caricamento modello in thread separato (non blocca ROS 2)
        # ------------------------------------------------------------------
        self.get_logger().info(f"Loading NaVILA model from: {model_path}")
        threading.Thread(
            target=self._load_model_thread,
            args=(model_path,),
            daemon=True,
        ).start()

    # ------------------------------------------------------------------
    # Callbacks subscriber
    # ------------------------------------------------------------------

    def _image_cb(self, msg: Image):
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            with self._lock:
                self.last_frame = frame_rgb
        except Exception as e:
            self.get_logger().warn(f"Image conversion error: {e}")

    def _goal_cb(self, msg: String):
        with self._lock:
            self.last_goal = msg.data
        self.get_logger().info(f"New goal received: '{msg.data}'")

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self.last_odom = msg

    # ------------------------------------------------------------------
    # Caricamento modello (thread)
    # ------------------------------------------------------------------

    def _load_model_thread(self, model_path: str):
        try:
            model, tokenizer, image_proc = load_navila_model(model_path)
            with self._lock:
                self.model       = model
                self.tokenizer   = tokenizer
                self.image_proc  = image_proc
                self._model_ready = True
            self.get_logger().info("NaVILA model ready — inference active.")
        except Exception as e:
            self.get_logger().error(f"Failed to load NaVILA model: {e}")

    # ------------------------------------------------------------------
    # Timer inferenza
    # ------------------------------------------------------------------

    def _inference_cb(self):
        with self._lock:
            ready  = self._model_ready
            frame  = self.last_frame
            goal   = self.last_goal
            model  = self.model
            tok    = self.tokenizer
            iproc  = self.image_proc

        # Attendi che modello, frame e goal siano disponibili
        if not ready:
            self.get_logger().info("Waiting for model to load...", throttle_duration_sec=5.0)
            return
        if frame is None:
            self.get_logger().info("Waiting for camera frame...", throttle_duration_sec=5.0)
            return
        if not goal:
            self.get_logger().info("Waiting for goal instruction...", throttle_duration_sec=5.0)
            return

        # Inferenza
        try:
            action = run_navila_inference(model, tok, iproc, frame, goal)
            self.get_logger().info(f"Action: {action}  (goal: '{goal}')")

            msg = String()
            msg.data = action
            self.pub_action.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")


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