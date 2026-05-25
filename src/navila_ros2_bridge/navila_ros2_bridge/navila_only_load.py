#!/usr/bin/env python3
"""
minimal_load_node.py
Carica solo il modello NaVILA senza fare nulla altro.
Serve per isolare se è il processo Python con il modello carico a bloccare Gazebo.
"""

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

import os
import threading

import rclpy
from rclpy.node import Node


def load_navila_model(model_path: str):
    import torch
    from llava.model.builder import load_pretrained_model
    from llava.mm_utils import get_model_name_from_path

    torch.cuda.empty_cache()
    os.environ["DS_SKIP_CUDA_CHECK"] = "1"
    os.environ["DS_ACCELERATOR"]     = "cpu"

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
    return model, tokenizer, image_processor


class MinimalLoadNode(Node):

        def __init__(self):
            super().__init__("minimal_load_node")

            self.declare_parameter("model_path", os.environ.get("NAVILA_MODEL_PATH", "/models"))
            model_path = self.get_parameter("model_path").value

            # Subscriber odom
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            qos_sensor = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            from nav_msgs.msg import Odometry
            self.sub_odom = self.create_subscription(
                Odometry, '/platform/odom', lambda msg: None, qos_sensor)

            self.get_logger().info("Subscriber odom attivo — il robot si muove?")

            threading.Thread(target=self._load, args=(model_path,), daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = MinimalLoadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()