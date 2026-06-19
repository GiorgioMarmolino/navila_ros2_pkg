#!/usr/bin/env python3
"""
Test offline di NaVILA su sequenze di immagini reali.
Riusa le STESSE funzioni del nodo (run_navila_inference, parse_navila_output,
_sample_history, _expand_primitives) → il test non può divergere dal runtime.

Uso:
    python3 test_navila_offline.py /percorso/sequenza "Go to the door at the end"

La cartella deve contenere immagini ordinabili per nome (01.jpg, 02.jpg, ...),
ordine oldest -> newest (ultima = osservazione corrente).
"""

import sys
import glob
import os
import cv2

# Import diretto dal nodo: stesse funzioni del runtime → fedeltà garantita.
from navila_ros2_bridge.navila_super_node import (
    NaViLANode,
    load_navila_model,
    run_navila_inference,
    parse_navila_output,
)


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


def main():
    if len(sys.argv) < 3:
        raise SystemExit('Uso: python3 test_navila_offline.py <cartella> "<goal>"')

    folder = sys.argv[1]
    goal = sys.argv[2]

    print("Carico il modello NaVILA...")
    model, tokenizer, image_proc = load_navila_model(
        os.environ.get("NAVILA_MODEL_PATH", "/models"))

    # num_video_frames autoritativo dal checkpoint, come fa il nodo.
    num_video_frames = getattr(model.config, "num_video_frames", 8)
    print(f"num_video_frames = {num_video_frames}")

    frames_all = load_sequence(folder)

    # Stesso comportamento del nodo: la memoria cresce di un frame per step e a
    # ogni step si campiona/padda con _sample_history (frame neri in testa se
    # hist < N). Così vedo l'evoluzione step-by-step, incluso il transitorio iniziale.
    print(f'\nGOAL: "{goal}"\n' + "=" * 78)
    history = []
    for step, frame in enumerate(frames_all):
        # nel nodo: _sample_history(list(_frame_history) + [curr], N)
        frames = NaViLANode._sample_history(history + [frame], num_video_frames)
        raw = run_navila_inference(model, tokenizer, image_proc,
                                   frames, goal, num_video_frames)
        action, value, unit = parse_navila_output(raw)
        cmd, n_total = NaViLANode._expand_primitives(action, value)

        print(f"step {step:2d} | hist={len(history):2d} | "
              f"action={action:10s} {value:>3}{unit:<3} | "
              f"cmd='{cmd}' x{n_total} | raw='{raw}'")

        # La storia avanza col frame corrente (come la promozione di _last_decision_frame).
        history.append(frame)

    print("=" * 78)


if __name__ == "__main__":
    main()