#!/usr/bin/env python3
"""
Test offline di NaVILA su sequenze di immagini reali.
Riusa la stessa run_navila_inference del nodo, senza ROS.

Uso:
    python3 test_navila_offline.py /percorso/sequenza "Go to the door at the end"

La cartella deve contenere immagini ordinabili per nome (es. 01.jpg, 02.jpg, ...),
ordine oldest -> newest (ultima = osservazione corrente).
"""

import sys
import glob
import os
import numpy as np
import cv2

# importa la TUA funzione dal nodo (adatta il nome del modulo/pacchetto)
from navila_ros2_bridge.navila_node import load_navila_model, run_navila_inference, parse_navila_output

NUM_VIDEO_FRAMES = 8


def load_sequence(folder):
    paths = sorted(glob.glob(os.path.join(folder, "*.jpg")) +
                   glob.glob(os.path.join(folder, "*.png")))
    if not paths:
        raise SystemExit(f"Nessuna immagine in {folder}")
    frames = []
    for p in paths:
        bgr = cv2.imread(p)
        if bgr is None:
            print(f"  skip (illeggibile): {p}")
            continue
        frames.append(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))  # come nel nodo
    print(f"Caricate {len(frames)} immagini da {folder}")
    return frames


def sample_history(history, num_frames):
    """Identico a _sample_history del nodo."""
    n = len(history)
    if n == 0:
        return []
    idxs = np.linspace(0, n - 1, num_frames)
    return [history[int(round(float(i)))] for i in idxs]


def main():
    if len(sys.argv) < 3:
        raise SystemExit('Uso: python3 test_navila_offline.py <cartella> "<goal>"')

    folder = sys.argv[1]
    goal = sys.argv[2]

    print("Carico il modello NaVILA...")
    model, tokenizer, image_proc = load_navila_model(
        os.environ.get("NAVILA_MODEL_PATH", "/models"))

    frames_all = load_sequence(folder)

    # Simulo lo stesso comportamento del nodo: la memoria cresce un frame per step,
    # e a ogni step campiono NUM_VIDEO_FRAMES. Cosi vedo l'evoluzione step-by-step,
    # incluso il transitorio iniziale.
    print(f'\nGOAL: "{goal}"\n' + "=" * 60)
    history = []
    for step, frame in enumerate(frames_all):
        history.append(frame)
        frames = sample_history(history, NUM_VIDEO_FRAMES)
        raw = run_navila_inference(model, tokenizer, image_proc,
                                   frames, goal, NUM_VIDEO_FRAMES)
        action = parse_navila_output(raw)
        print(f"step {step:2d} | history={len(history):2d} | "
              f"action={action:12s} | raw='{raw}'")

    print("=" * 60)


if __name__ == "__main__":
    main()