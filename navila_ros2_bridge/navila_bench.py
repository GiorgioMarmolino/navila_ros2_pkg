#!/usr/bin/env python3
"""
bench_navila.py — Misura il tempo di inferenza di NaVILA.

NaVILA campiona SEMPRE un numero fisso di frame (_sample_history ->
num_video_frames, tipicamente 8): l'input del modello ha dimensione costante,
quindi la latenza NON cresce con la lunghezza dell'episodio. Qui la media e'
una metrica ben definita.

Misuro direttamente agent._infer() — il vero forward del modello (process_images
+ generate) — costruendo i frame con lo stesso agent._sample_history() usato in
deploy. Cosi' evito la contaminazione del ramo "replay coda" di step() (che NON
fa inferenza) e controllo esattamente cosa entra nel modello.

Modalita':
  * default            : N chiamate _infer() con storico realistico -> media/percentili
  * --demonstrate-constant : varia la profondita' dello storico (0,8,32,128,512)
                         e cronometra _infer() a ogni profondita'. Atteso: PIATTO
                         (dimostra che la finestra fissa rende la latenza costante).

Timing corretto: torch.cuda.Event + synchronize. _infer() usa gia'
torch.inference_mode() e do_sample=False (greedy) -> output deterministico, poco
rumore rispetto a Uni-NaVid.

USO (dentro il container/env del nodo, col modello gia' scaricato):
    python bench_navila.py --iters 100 --warmup 10 --out navila_pcpilot.csv
    python bench_navila.py --demonstrate-constant --out navila_flat.csv
"""

import argparse
import os
import statistics
import time

import numpy as np

# Stesso import del nodo: adatta il path al punto in cui posizioni il file.
from navila_ros2_bridge.third_party.navila_agent import NaViLAAgent

try:
    import torch
    _CUDA = torch.cuda.is_available()
except ImportError:
    torch = None
    _CUDA = False


def time_call(fn) -> float:
    if _CUDA:
        torch.cuda.synchronize()
        start = torch.cuda.Event(enable_timing=True)
        end = torch.cuda.Event(enable_timing=True)
        start.record()
        fn()
        end.record()
        torch.cuda.synchronize()
        return start.elapsed_time(end)
    t0 = time.perf_counter()
    fn()
    return (time.perf_counter() - t0) * 1000.0


def summarize(samples):
    s = sorted(samples)
    n = len(s)

    def pct(p):
        if n == 1:
            return s[0]
        k = (n - 1) * p
        lo = int(k)
        hi = min(lo + 1, n - 1)
        return s[lo] + (s[hi] - s[lo]) * (k - lo)

    mean = statistics.fmean(s)
    return {
        "n": n,
        "mean_ms": round(mean, 2),
        "std_ms": round(statistics.pstdev(s), 2) if n > 1 else 0.0,
        "median_ms": round(statistics.median(s), 2),
        "p90_ms": round(pct(0.90), 2),
        "p99_ms": round(pct(0.99), 2),
        "min_ms": round(s[0], 2),
        "max_ms": round(s[-1], 2),
        "hz": round(1000.0 / mean, 2) if mean > 0 else float("inf"),
    }


def make_frames(agent, history_depth, h, w):
    """Costruisce l'input di _infer() esattamente come step(): storico + corrente
    passati a _sample_history, che ritorna sempre num_video_frames frame."""
    history = [np.random.randint(0, 255, (h, w, 3), dtype=np.uint8)
               for _ in range(history_depth)]
    current = np.random.randint(0, 255, (h, w, 3), dtype=np.uint8)
    return agent._sample_history(history + [current], agent.num_video_frames)


def main():
    ap = argparse.ArgumentParser(description="Benchmark tempo di inferenza NaVILA.")
    ap.add_argument("--model-path", default=os.environ.get("NAVILA_MODEL_PATH", "/models"))
    ap.add_argument("--num-video-frames", type=int, default=8,
                    help="riconciliato col checkpoint dopo il load (checkpoint autoritativo)")
    ap.add_argument("--max-history-frames", type=int, default=512)
    ap.add_argument("--iters", type=int, default=100, help="chiamate _infer() cronometrate")
    ap.add_argument("--warmup", type=int, default=10, help="chiamate di warmup scartate")
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--instruction", default="reach the blue chair")
    ap.add_argument("--demonstrate-constant", action="store_true",
                    help="varia la profondita' dello storico e mostra che la latenza e' piatta")
    ap.add_argument("--out", default="", help="salva i risultati in CSV")
    args = ap.parse_args()

    print("=" * 64)
    print("NaVILA inference benchmark")
    if _CUDA:
        print(f"  GPU               : {torch.cuda.get_device_name(0)}")
        print(f"  torch / CUDA      : {torch.__version__} / {torch.version.cuda}")
    else:
        print("  ATTENZIONE: CUDA non disponibile -> timing su CPU (perf_counter)")
    print("=" * 64)

    print(f"Carico l'agent da: {args.model_path}")
    agent = NaViLAAgent(
        model_path=args.model_path,
        num_video_frames=args.num_video_frames,
        max_history_frames=args.max_history_frames,
    )
    agent.load_model()                       # NaVILA carica esplicitamente (a differenza di Uni-NaVid)
    agent.reset(args.instruction)            # _infer() usa self.goal
    nvf = agent.num_video_frames
    print(f"num_video_frames (dal checkpoint): {nvf}")

    h, w = args.height, args.width

    if args.demonstrate_constant:
        depths = [0, nvf, 4 * nvf, 16 * nvf, args.max_history_frames]
        print("\nDIMOSTRAZIONE COSTANZA — latenza vs profondita' storico")
        print(f"{'hist_depth':>10} | {'mean_ms':>9} | {'p90_ms':>9} | {'Hz':>7}")
        print("-" * 44)
        rows = []
        # warmup unico
        f0 = make_frames(agent, nvf, h, w)
        for _ in range(args.warmup):
            agent._infer(f0)
        if _CUDA:
            torch.cuda.synchronize()
            torch.cuda.reset_peak_memory_stats()
        for depth in depths:
            frames = make_frames(agent, depth, h, w)   # sempre nvf frame in uscita
            lat = [time_call(lambda: agent._infer(frames)) for _ in range(max(10, args.iters // 5))]
            st = summarize(lat)
            rows.append((depth, st))
            print(f"{depth:>10} | {st['mean_ms']:>9} | {st['p90_ms']:>9} | {st['hz']:>7}")
        print("\nAtteso: colonna mean_ms sostanzialmente PIATTA (finestra fissa).")
        if args.out:
            with open(args.out, "w") as fp:
                fp.write("hist_depth,mean_ms,std_ms,p90_ms,p99_ms,hz\n")
                for depth, st in rows:
                    fp.write(f"{depth},{st['mean_ms']},{st['std_ms']},{st['p90_ms']},{st['p99_ms']},{st['hz']}\n")
            print(f"Salvato in {args.out}")
        return

    # --- modalita' default: media su storico realistico (finestra piena) ---
    frames = make_frames(agent, nvf, h, w)

    print("Warmup...")
    for _ in range(args.warmup):
        agent._infer(frames)
    if _CUDA:
        torch.cuda.synchronize()
        torch.cuda.reset_peak_memory_stats()

    print("Misura in corso...")
    lat = [time_call(lambda: agent._infer(frames)) for _ in range(args.iters)]
    st = summarize(lat)

    print("\n" + "=" * 64)
    print("RISULTATO (NaVILA — latenza costante, finestra fissa)")
    print(f"  media             : {st['mean_ms']} ms  ({st['hz']} Hz)")
    print(f"  std / mediana     : {st['std_ms']} / {st['median_ms']} ms")
    print(f"  p90 / p99         : {st['p90_ms']} / {st['p99_ms']} ms")
    print(f"  min / max         : {st['min_ms']} / {st['max_ms']} ms")
    if _CUDA:
        print(f"  picco memoria     : {torch.cuda.max_memory_allocated()/1024**3:.2f} GB")
    print("=" * 64)

    if args.out:
        with open(args.out, "w") as fp:
            fp.write("iter,latency_ms\n")
            for i, v in enumerate(lat):
                fp.write(f"{i},{v:.3f}\n")
        print(f"Salvato in {args.out} (colonne: iter,latency_ms)")


if __name__ == "__main__":
    main()