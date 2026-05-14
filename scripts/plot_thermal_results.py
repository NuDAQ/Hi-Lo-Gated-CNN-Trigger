#!/usr/bin/env python3
"""
plot_thermal_results.py

Reads chunk_wave.csv and cnn_results.txt produced by tb_thermal.sv and
generates one waveform plot per captured chunk, styled after the hilo-trigger
analysis plots (gate windows, multiplicity panel, threshold lines).

Each plot shows:
  - Rows 0-3 : 4 ADC channel waveforms (in σ units), coincidence gate shading,
               ±THRESH lines, and a vertical L0 trigger marker at sample 128
               (the boundary between pre-trigger and post-trigger halves).
  - Row 4    : Per-sample channel multiplicity with BIN_THR line.
  - Title    : chunk type (thermal noise / false trigger), CNN score.

Inputs (from hw/sim/thermal_data/):
  chunk_wave.csv    — waveforms logged by tb_thermal.sv
  cnn_results.txt   — per-chunk trigger time and CNN score

Outputs (to hw/sim/thermal_data/plots/):
  chunk_0_thermal.png
  chunk_1_thermal.png
  chunk_2_thermal.png

Usage:
  python3 scripts/plot_thermal_results.py \
      [--data-dir hw/sim/thermal_data] \
      [--thresh 192] [--hilo-window 5] [--coinc-window 16] [--bin-thr 1] \
      [--sigma-scale 64.0]
"""

import argparse
import math
import pathlib

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D


# ---------------------------------------------------------------------------
# HiLo Boolean Emulation (same as plot_sanity_results.py)
# ---------------------------------------------------------------------------

def emulate_hilo(data: np.ndarray,
                 thresh: int,
                 hilo_window: int,
                 coinc_window: int,
                 bin_thr: int):
    N = len(data)
    hilo_window  = min(int(hilo_window), 16)
    coinc_window = min(int(coinc_window), 32)   # clamped to hardware max of 32

    gate4 = np.zeros((N, 4), dtype=bool)
    for ch in range(4):
        col    = data[:, ch].astype(np.int32)
        ot_hi  = col >  thresh
        ot_lo  = col < -thresh
        gate_hi = np.zeros(N, dtype=bool)
        gate_lo = np.zeros(N, dtype=bool)
        for i in range(N):
            start = max(0, i - hilo_window + 1)
            gate_hi[i] = ot_hi[start:i+1].any()
            gate_lo[i] = ot_lo[start:i+1].any()
        gate4[:, ch] = gate_hi & gate_lo

    coinc4 = np.zeros((N, 4), dtype=bool)
    for ch in range(4):
        for i in range(N):
            start = max(0, i - coinc_window + 1)
            coinc4[i, ch] = gate4[start:i+1, ch].any()

    mult     = coinc4.sum(axis=1).astype(np.int32)
    pre_trig = mult >= bin_thr
    return gate4, coinc4, mult, pre_trig


# ---------------------------------------------------------------------------
# Per-Chunk Plot
# ---------------------------------------------------------------------------

def sigmoid(x: float) -> float:
    return 1.0 / (1.0 + math.exp(-x))


def plot_chunk(chunk_id: int,
               data: np.ndarray,
               cnn_fired: bool,
               cnn_score: float,
               cnn_prob: float,
               thresh: int,
               hilo_window: int,
               coinc_window: int,
               bin_thr: int,
               sigma_scale: float,
               out_path: pathlib.Path):
    """
    5-panel plot for one 256-sample chunk (thermal noise false trigger).
    """
    N   = len(data)        # 256
    t   = np.arange(N)
    pre_boundary = 128     # sample index where post-trigger starts

    gate4, coinc4, mult, pre_trig = emulate_hilo(
        data, thresh, hilo_window, coinc_window, bin_thr)

    thresh_sigma  = thresh / sigma_scale
    ch_colors     = ["C0", "C1", "C2", "C3"]

    fig, axes = plt.subplots(5, 1, figsize=(14, 10),
                              gridspec_kw={"height_ratios": [2, 2, 2, 2, 1.2]},
                              sharex=True)
    fig.subplots_adjust(hspace=0.08, top=0.90, bottom=0.07, left=0.08, right=0.97)

    # ---- Channel panels ----
    for ch in range(4):
        ax  = axes[ch]
        sig = data[:, ch] / sigma_scale

        ax.plot(t, sig, lw=0.9, color=ch_colors[ch], zorder=3, label=f"Ch {ch}")
        ax.axhline( thresh_sigma, color="darkred", lw=0.8, ls="--", alpha=0.8)
        ax.axhline(-thresh_sigma, color="darkred", lw=0.8, ls="--", alpha=0.8)

        # Coincidence gate shading
        ax.fill_between(t, ax.get_ylim()[0], ax.get_ylim()[1],
                        where=coinc4[:, ch],
                        alpha=0.18, color="green", zorder=1)

        # Pre/post boundary (where L0 fired — start of post-trigger)
        ax.axvline(pre_boundary, color="red", lw=1.2, ls=":",
                   alpha=0.85, zorder=4,
                   label="L0 boundary" if ch == 0 else "")

        ax.set_ylabel(f"Ch {ch} (σ)", fontsize=9)
        ax.tick_params(labelsize=8)
        ax.grid(axis="x", lw=0.3, alpha=0.5)

        if ch == 0:
            custom_handles = [
                Line2D([0], [0], color=ch_colors[ch], lw=1.2,
                       label=f"Ch {ch}"),
                Line2D([0], [0], color="darkred", lw=0.8, ls="--",
                       label=f"±{thresh_sigma:.1f}σ"),
                mpatches.Patch(color="green", alpha=0.3,
                               label="Coinc gate"),
                Line2D([0], [0], color="red", lw=1.2, ls=":",
                       label="L0 boundary (pre|post)"),
            ]
            ax.legend(handles=custom_handles, fontsize=7,
                      loc="upper right", ncol=4, framealpha=0.6)

    # ---- Multiplicity panel ----
    ax_m = axes[4]
    ax_m.step(t, mult, where="post", lw=1.2, color="purple", label="Multiplicity")
    ax_m.axhline(bin_thr, color="black", lw=0.9, ls="--",
                 alpha=0.8, label=f"BIN_THR={bin_thr}")
    ax_m.fill_between(t, 0, mult, where=pre_trig,
                      alpha=0.25, color="red", label="PRE_TRIG active")
    ax_m.axvline(pre_boundary, color="red", lw=1.2, ls=":", alpha=0.85)
    ax_m.set_ylim(-0.1, 4.5)
    ax_m.set_ylabel("Channels", fontsize=9)
    ax_m.set_xlabel("Sample index (256-sample chunk: 0–127 pre, 128–255 post)",
                    fontsize=9)
    ax_m.legend(fontsize=7, loc="upper right", ncol=3, framealpha=0.6)
    ax_m.tick_params(labelsize=8)
    ax_m.grid(axis="x", lw=0.3, alpha=0.5)

    # ---- CNN annotation ----
    score_str  = (f"CNN prob = {cnn_prob:.4f}  (score = {cnn_score:.4f})"
                  if cnn_fired else "CNN: no output (timeout)")
    result_str = "THERMAL NOISE (expected prob < 0.5)"
    result_col = "steelblue"

    fig.text(0.97, 0.97,
             f"{result_str}\n{score_str}",
             ha="right", va="top", fontsize=10,
             color=result_col,
             bbox=dict(boxstyle="round,pad=0.3",
                       facecolor="lightyellow", edgecolor=result_col, alpha=0.85))

    # ---- Python emulation note ----
    emul_first = int(np.argmax(pre_trig)) if pre_trig.any() else -1
    emul_str   = (f"Python emulation: L0 @ sample {emul_first}"
                  if emul_first >= 0 else "Python emulation: no trigger")

    fig.suptitle(
        f"Chunk {chunk_id} [THERMAL NOISE / False Trigger]   "
        f"THRESH={thresh} ({thresh_sigma:.1f}σ)   "
        f"HILO_WIN={hilo_window}   COINC_WIN={coinc_window}   "
        f"BIN_THR={bin_thr}\n"
        f"{emul_str}",
        fontsize=10
    )

    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"  Saved: {out_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Plot thermal-noise chunk waveforms from tb_thermal.sv")
    parser.add_argument("--data-dir",     default="hw/sim/thermal_data")
    parser.add_argument("--thresh",       type=int,   default=192,
                        help="ADC counts threshold (default: 192 = 3σ×64)")
    parser.add_argument("--hilo-window",  type=int,   default=5)
    parser.add_argument("--coinc-window", type=int,   default=30)
    parser.add_argument("--bin-thr",      type=int,   default=2)
    parser.add_argument("--sigma-scale",  type=float, default=64.0)
    args = parser.parse_args()

    data_dir = pathlib.Path(args.data_dir)
    plot_dir = data_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # Load waveform CSV
    # ------------------------------------------------------------------
    wave_path = data_dir / "chunk_wave.csv"
    if not wave_path.exists():
        print(f"ERROR: {wave_path} not found. Run tb_thermal.sv first.")
        raise SystemExit(1)

    wave_df = pd.read_csv(wave_path, comment="#",
                          names=["chunk_id", "sample_idx", "ch0", "ch1", "ch2", "ch3"])
    print(f"Loaded waveform: {len(wave_df)} rows, {wave_df['chunk_id'].nunique()} chunks")

    # ------------------------------------------------------------------
    # Load CNN results
    # ------------------------------------------------------------------
    results_path = data_dir / "cnn_results.txt"
    if not results_path.exists():
        print(f"ERROR: {results_path} not found.")
        raise SystemExit(1)

    results_df = pd.read_csv(results_path, comment="#",
                             names=["chunk_id", "l0_time_ns", "cnn_fired",
                                    "cnn_raw_hex", "cnn_score_float", "overflow"])
    print(f"Loaded results: {len(results_df)} chunks")
    print(results_df.to_string(index=False))

    # ------------------------------------------------------------------
    # Per-chunk plot
    # ------------------------------------------------------------------
    for chunk_id in sorted(wave_df["chunk_id"].unique()):
        ev_wave = wave_df[wave_df["chunk_id"] == chunk_id].sort_values("sample_idx")
        data = ev_wave[["ch0", "ch1", "ch2", "ch3"]].values.astype(np.int32)

        if len(data) != 256:
            print(f"  WARNING: chunk {chunk_id} has {len(data)} samples "
                  f"(expected 256). Skipping.")
            continue

        row = results_df[results_df["chunk_id"] == chunk_id]
        if row.empty:
            print(f"  WARNING: no results for chunk {chunk_id}. Skipping.")
            continue
        row = row.iloc[0]

        cnn_fired = bool(int(row["cnn_fired"]))
        cnn_score = float(row["cnn_score_float"])
        cnn_prob  = sigmoid(cnn_score) if cnn_fired else 0.0

        out_path = plot_dir / f"chunk_{chunk_id}_thermal.png"
        print(f"\nPlotting chunk {chunk_id} ...")
        plot_chunk(
            chunk_id     = chunk_id,
            data         = data,
            cnn_fired    = cnn_fired,
            cnn_score    = cnn_score,
            cnn_prob     = cnn_prob,
            thresh       = args.thresh,
            hilo_window  = args.hilo_window,
            coinc_window = args.coinc_window,
            bin_thr      = args.bin_thr,
            sigma_scale  = args.sigma_scale,
            out_path     = out_path,
        )

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    n_total = len(results_df)
    n_cnn   = int(results_df["cnn_fired"].sum())
    print(f"\n{'='*50}")
    print(f"  Plots  : {plot_dir}")
    print(f"  Chunks : {n_total}  ({n_cnn} with CNN output)")
    for _, r in results_df.iterrows():
        if int(r["cnn_fired"]):
            score = float(r["cnn_score_float"])
            prob  = sigmoid(score)
            verdict = "<0.5 — correct (noise)" if prob < 0.5 else "≥0.5 — unexpected"
            print(f"  chunk {int(r['chunk_id'])}: prob={prob:.4f}  ({verdict})")
        else:
            print(f"  chunk {int(r['chunk_id'])}: no CNN output (timeout)")
    print(f"{'='*50}\n")


if __name__ == "__main__":
    main()
