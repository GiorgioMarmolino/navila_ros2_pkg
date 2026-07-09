#!/usr/bin/env python3
"""NaViLA policy agent — ROS-agnostic. Owns model + inference + history/queue."""

# --- Mock deepspeed (deve precedere qualsiasi import di llava) ---------------
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
# -----------------------------------------------------------------------------

import os
import re
import logging
from collections import deque

import numpy as np
from PIL import Image as PILImage

log = logging.getLogger("navila_agent")

_OFFICIAL_PATTERNS = {
    "stop":       re.compile(r"\bstop\b", re.IGNORECASE),
    "forward":    re.compile(r"\bis move forward\b", re.IGNORECASE),
    "turn_left":  re.compile(r"\bis turn left\b", re.IGNORECASE),
    "turn_right": re.compile(r"\bis turn right\b", re.IGNORECASE),
}


class NaViLAAgent:
    def __init__(self, model_path, num_video_frames=8, max_history_frames=512):
        self.model_path         =       model_path
        self.num_video_frames   =       num_video_frames
        self.goal               =       ""
        self.ready              =       False
        self.model              =       self.tokenizer = self.image_proc = None
        self._history           =       deque(maxlen=max_history_frames)
        self._queue             =       []
        self._pending = None          # frame deciso ma non ancora promosso

    # ------------------------------------------------------------------ load
    def load_model(self):
        import torch
        from llava.model.builder import load_pretrained_model
        from llava.mm_utils import get_model_name_from_path
        from huggingface_hub import snapshot_download

        HF_MODEL_ID = "a8cheng/navila-llama3-8b-8f"
        torch.cuda.empty_cache()
        os.environ["DS_SKIP_CUDA_CHECK"] = "1"
        os.environ["DS_ACCELERATOR"] = "cpu"

        if not os.path.exists(os.path.join(self.model_path, "config.json")):
            log.info(f"Downloading model from HF: {HF_MODEL_ID}")
            snapshot_download(repo_id=HF_MODEL_ID, local_dir=self.model_path,
                              local_dir_use_symlinks=False)
        else:
            log.info(f"Model found at: {self.model_path}")

        model_name = get_model_name_from_path(self.model_path)
        tokenizer, model, image_proc, _ = load_pretrained_model(
            model_path=self.model_path, model_base=None, model_name=model_name,
            device_map="auto", offload_folder="offload",
        )
        if tokenizer.pad_token is None:
            tokenizer.pad_token = tokenizer.eos_token
        model.eval()

        # checkpoint autoritativo su num_video_frames
        cfg_nvf = getattr(model.config, "num_video_frames", None)
        if cfg_nvf is None:
            log.warning(f"config.num_video_frames missing — keep {self.num_video_frames}")
        elif cfg_nvf != self.num_video_frames:
            log.warning(f"num_video_frames: param={self.num_video_frames} "
                        f"checkpoint={cfg_nvf} → using checkpoint")
            self.num_video_frames = cfg_nvf

        self.model, self.tokenizer, self.image_proc = model, tokenizer, image_proc
        self.ready = True          # ultimo: garantisce visibilità dei campi sopra
        log.info(f"NaVILA ready — num_video_frames={self.num_video_frames}")

    # ------------------------------------------------------------------ reset
    def reset(self, goal=""):
        self.goal = goal
        self._history.clear()
        self._queue.clear()
        self._pending = None

    # ------------------------------------------------------------------ step
    def step(self, frame_rgb, prev_status="done"):
        if not self.ready:
            raise RuntimeError("Model not loaded — call load_model() first.")

        # 1) esito primitiva precedente
        if prev_status == "aborted":
            self._pending = None          # moto non avvenuto → fuori dallo storico
            self._queue.clear()
        elif prev_status == "done":
            if self._pending is not None:
                self._history.append(self._pending)
                self._pending = None
        # "start": reset() ha già ripulito

        hist_len = len(self._history)

        # 2) replay coda senza inferenza (queue_actions ufficiale)
        if self._queue:
            cmd = self._queue.pop(0)
            self._pending = frame_rgb
            return dict(cmd=cmd, from_queue=True, is_stop=False, raw="",
                        action=None, value=0, unit="", n_total=0, hist_len=hist_len)

        # 3) inferenza
        frames = self._sample_history(list(self._history) + [frame_rgb],
                                      self.num_video_frames)
        raw = self._infer(frames)
        action, value, unit = self._parse_output(raw)
        self._pending = frame_rgb

        if action == "stop":
            return dict(cmd="stop", from_queue=False, is_stop=True, raw=raw,
                        action=action, value=value, unit=unit,
                        n_total=0, hist_len=hist_len)

        cmd, n_total = self._expand_primitives(action, value)
        self._queue = [cmd] * (n_total - 1)
        return dict(cmd=cmd, from_queue=False, is_stop=False, raw=raw,
                    action=action, value=value, unit=unit,
                    n_total=n_total, hist_len=hist_len)

    # ------------------------------------------------------------------ infer
    def _infer(self, frames_rgb):
        import torch
        from llava.mm_utils import (process_images, tokenizer_image_token,
                                    KeywordsStoppingCriteria)
        from llava.constants import IMAGE_TOKEN_INDEX
        from llava.conversation import conv_templates, SeparatorStyle

        pil = [PILImage.fromarray(f) for f in frames_rgb]
        image_tensor = process_images(pil, self.image_proc, self.model.config)
        image_tensor = image_tensor.to(dtype=torch.float16, device="cuda")

        conv = conv_templates["llama_3"].copy()
        image_token = "<image>\n"
        qs = (
            f"Imagine you are a robot programmed for navigation tasks. You have been given a video "
            f"of historical observations {image_token * (self.num_video_frames - 1)}, and current observation <image>\n. "
            f"Your assigned task is: \"{self.goal}\" "
            f"Analyze this series of images to decide your next action, which could be turning left or right by a specific "
            f"degree, moving forward a certain distance, or stop if the task is completed."
        )
        conv.append_message(conv.roles[0], qs)
        conv.append_message(conv.roles[1], None)
        prompt_text = conv.get_prompt()

        input_ids = tokenizer_image_token(
            prompt_text, self.tokenizer, IMAGE_TOKEN_INDEX, return_tensors="pt"
        ).unsqueeze(0).to("cuda")

        stop_str = conv.sep if conv.sep_style != SeparatorStyle.TWO else conv.sep2
        stopping = KeywordsStoppingCriteria([stop_str], self.tokenizer, input_ids)

        with torch.inference_mode():
            out_ids = self.model.generate(
                input_ids, images=image_tensor, do_sample=False, temperature=0.0,
                num_beams=1, max_new_tokens=32, use_cache=True,
                pad_token_id=self.tokenizer.eos_token_id,
                stopping_criteria=[stopping],
            )
        return self.tokenizer.decode(out_ids[0], skip_special_tokens=True).strip().lower()

    # ------------------------------------------------------------------ static
    @staticmethod
    def _parse_output(text):
        action = None
        for name, pat in _OFFICIAL_PATTERNS.items():
            if pat.search(text):
                action = name
                break
        if action is None:
            action = "stop"
        if action == "forward":
            m = re.search(r"move forward (\d+) cm", text)
            d = int(m.group(1)) if m else 25
            if d % 25 != 0:
                d = min([25, 50, 75], key=lambda x: abs(x - d))
            return "forward", d, "cm"
        if action in ("turn_left", "turn_right"):
            side = "left" if action == "turn_left" else "right"
            m = re.search(rf"turn {side} (\d+) degree", text)
            g = int(m.group(1)) if m else 15
            if g % 15 != 0:
                g = min([15, 30, 45], key=lambda x: abs(x - g))
            return action, g, "deg"
        return "stop", 0, ""

    @staticmethod
    def _sample_history(history, num_frames, pad_h=512, pad_w=512):
        frames = list(history)
        while len(frames) < num_frames:
            frames.insert(0, np.zeros((pad_h, pad_w, 3), dtype=np.uint8))
        latest = frames[-1]
        idxs = np.linspace(0, len(frames) - 1, num=num_frames - 1,
                           endpoint=False, dtype=int)
        return [frames[i] for i in idxs] + [latest]

    @staticmethod
    def _expand_primitives(action, value):
        if action == "forward":
            return "forward 25 cm", max(1, int(value) // 25)
        if action == "turn_left":
            return "turn_left 15 deg", max(1, int(value) // 15)
        if action == "turn_right":
            return "turn_right 15 deg", max(1, int(value) // 15)
        return "stop", 0