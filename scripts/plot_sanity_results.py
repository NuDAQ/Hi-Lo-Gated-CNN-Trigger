#!/usr/bin/env python3
"""
plot_sanity_results.py

Reads the simulation logs produced by tb_sanity.sv and generates waveform
plots similar to the hilo-trigger analysis (gate windows, multiplicity panel,
threshold lines) for each of the 4 sanity-check events.

Inputs  (from hw/sim/sanity_data/):
  sanity_wave.csv    — ADC waveforms logged during simulation
  sanity_results.txt — per-event trigger / CNN result
  sanity_golden.txt  — expected labels from prepare_sanity_chunks.py

Outputs (to hw/sim/sanity_data/plots/):
  event_0_sig.png
  event_1_sig.png
  event_2_sig.png
  event_3_noise.png

The Hi-Lo trigger logic is emulated in Python (boolean RTL emulation) to
overlay the gate window and multiplicity panels, matching the style of the
hilo-trigger repository's plot_rtl_emulation.py.

Usage:
  python3 scripts/plot_sanity_results.py [--data-dir hw/sim/sanity_data]
                                          [--thresh 300]
                                          [--hilo-window 10]
                                          [--coinc-window 20]
                                          [--bin-thr 1]
                                          [--sigma-scale 64]

Run from the project root on Ubuntu.
"""

import argparse
import pathlib

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D


# ---------------------------------------------------------------------------
# Hi-Lo RTL Boolean Emulation
# ---------------------------------------------------------------------------

def emulate_hilo(data: np.ndarray,
                 thresh: int,
                 hilo_window: int,
                 coinc_window: int,
                 bin_thr: int):
    """
    Python boolean emulation of the PRE_TRIGGER RTL logic.

    Mirrors the three-stage pipeline in Pre_trigger.vhd and
    Pre_trigger_1ch.vhd (continuous sample stream, no batch boundaries).

    Parameters
    ----------
    data         : (N, 4) int32 array, signed 12-bit ADC values
    thresh       : threshold in ADC counts
    hilo_window  : HILO_WINDOW parameter (samples, clamped to ≤16)
    coinc_window : COINC_WINDOW parameter (samples, clamped to ≤32)
    bin_thr      : BIN_THR parameter (1–4)

    Returns
    -------
    gate4    : (N, 4) bool  — per-channel Hi-Lo gate
    coinc4   : (N, 4) bool  — per-channel coincidence-smeared gate
    mult     : (N,)   int   — active-channel count per sample
    pre_trig : (N,)   bool  — True where mult >= bin_thr
    """
    N = len(data)
    hilo_window  = min(int(hilo_window),  16)
    coinc_window = min(int(coinc_window), 32)

    # ------------------------------------------------------------------
    # Stage 1: per-channel bipolar threshold + HILO_WINDOW sliding gate
    # ------------------------------------------------------------------
    gate4 = np.zeros((N, 4), dtype=bool)
    for ch in range(4):
        col    = data[:, ch].astype(np.int32)
        ot_hi  = col >  thresh
        ot_lo  = col < -thresh

        # Causal sliding OR: gate_hi[i] = any ot_hi in [i-W+1 .. i]
        gate_hi = np.zeros(N, dtype=bool)
        gate_lo = np.zeros(N, dtype=bool)
        for i in range(N):
            start = max(0, i - hilo_window + 1)
            gate_hi[i] = ot_hi[start : i + 1].any()
            gate_lo[i] = ot_lo[start : i + 1].any()

        gate4[:, ch] = gate_hi & gate_lo   # bipolar AND

    # ------------------------------------------------------------------
    # Stage 2: per-channel coincidence smear (COINC_WINDOW)
    # ------------------------------------------------------------------
    coinc4 = np.zeros((N, 4), dtype=bool)
    for ch in range(4):
        for i in range(N):
            start = max(0, i - coinc_window + 1)
            coinc4[i, ch] = gate4[start : i + 1, ch].any()

    # ------------------------------------------------------------------
    # Stage 3: multiplicity threshold
    # ------------------------------------------------------------------
    mult     = coinc4.sum(axis=1).astype(np.int32)
    pre_trig = mult >= bin_thr

    return gate4, coinc4, mult, pre_trig


# ---------------------------------------------------------------------------
# Per-Event Plot
# ---------------------------------------------------------------------------

def plot_event(ev_id: int,
               ev_type: str,
               data: np.ndarray,
               gate4: np.ndarray,
               coinc4: np.ndarray,
               mult: np.ndarray,
               pre_trig: np.ndarray,
               l0_fired: bool,
               l0_sample: float,
               cnn_fired: bool,
               cnn_score: float,
               thresh: int,
               bin_thr: int,
               sigma_scale: float,
               pass_flag: bool,
               out_path: pathlib.Path):
    """
    Generate a 5-panel waveform figure for one event, styled after the
    hilo-trigger analysis plots.

    Layout:
      Row 0–3 : One panel per ADC channel
      Row 4   : Per-sample channel multiplicity
    """
    N = len(data)
    t = np.arange(N)

    fig, axes = plt.subplots(5, 1, figsize=(14, 10),
                              gridspec_kw={"height_ratios": [2, 2, 2, 2, 1.2]},
                              sharex=True)
    fig.subplots_adjust(hspace=0.08, top=0.93, bottom=0.07, left=0.08, right=0.97)

    ch_colors = ["C0", "C1", "C2", "C3"]
    thresh_sigma = thresh / sigma_scale

    # ---- Channel panels (rows 0..3) ----
    for ch in range(4):
        ax  = axes[ch]
        sig = data[:, ch] / sigma_scale   # convert to σ units

        # ADC waveform
        ax.plot(t, sig, lw=0.9, color=ch_colors[ch], zorder=3,
                label=f"Ch {ch}")

        # ±THRESH lines
        ax.axhline( thresh_sigma, color="darkred", lw=0.9, ls="--",
                   alpha=0.8, zorder=2)
        ax.axhline(-thresh_sigma, color="darkred", lw=0.9, ls="--",
                   alpha=0.8, zorder=2, label=f"±THRESH ({thresh_sigma:.1f}σ)")

        # Coincidence gate shading (green, like hilo-trigger analysis)
        gate_regions = coinc4[:, ch].astype(float)
        ax.fill_between(t, ax.get_ylim()[0], ax.get_ylim()[1],
                        where=coinc4[:, ch],
                        alpha=0.18, color="green", zorder=1,
                        label="Coinc gate")

        # L0_PRE_TRIG marker (vertical dashed red line)
        if l0_fired and l0_sample >= 0:
            ax.axvline(l0_sample, color="red", lw=1.2, ls=":",
                       alpha=0.9, zorder=4, label="L0_PRE_TRIG" if ch == 0 else "")

        ax.set_ylabel(f"Ch {ch} (σ)", fontsize=9)
        ax.tick_params(labelsize=8)
        ax.grid(axis="x", lw=0.3, alpha=0.5)

        # Compact legend only on ch0
        if ch == 0:
            custom_handles = [
                Line2D([0], [0], color=ch_colors[ch], lw=1.2,       label=f"Ch {ch}"),
                Line2D([0], [0], color="darkred",     lw=1.0, ls="--", label=f"±{thresh_sigma:.1f}σ"),
                mpatches.Patch(color="green",          alpha=0.3,    label="Coinc gate"),
                Line2D([0], [0], color="red",          lw=1.2, ls=":", label="L0 trigger"),
            ]
            ax.legend(handles=custom_handles, fontsize=7,
                      loc="upper right", ncol=4, framealpha=0.6)

    # ---- Multiplicity panel (row 4) ----
    ax_m = axes[4]
    ax_m.step(t, mult, where="post", lw=1.2, color="purple", label="Multiplicity")
    ax_m.axhline(bin_thr, color="black", lw=1.0, ls="--",
                 alpha=0.8, label=f"BIN_THR={bin_thr}")
    ax_m.fill_between(t, 0, mult, where=pre_trig,
                      alpha=0.25, color="red", label="PRE_TRIG active")
    if l0_fired and l0_sample >= 0:
        ax_m.axvline(l0_sample, color="red", lw=1.2, ls=":", alpha=0.9)
    ax_m.set_ylim(-0.1, 4.5)
    ax_m.set_ylabel("Channels", fontsize=9)
    ax_m.set_xlabel("Sample index (within 256-sample event chunk)", fontsize=9)
    ax_m.legend(fontsize=7, loc="upper right", ncol=3, framealpha=0.6)
    ax_m.tick_params(labelsize=8)
    ax_m.grid(axis="x", lw=0.3, alpha=0.5)

    # ---- CNN score annotation ----
    score_str = f"CNN score = {cnn_score:.4f} ({'fired' if cnn_fired else 'no output'})"
    result_str = "✓ PASS" if pass_flag else "✗ FAIL"
    result_color = "green" if pass_flag else "red"

    fig.text(0.97, 0.97,
             f"{result_str}\n{score_str}",
             ha="right", va="top", fontsize=10,
             color=result_color,
             bbox=dict(boxstyle="round,pad=0.3",
                       facecolor="lightyellow", edgecolor=result_color, alpha=0.85))

    # ---- Main title ----
    sim_l0_str = (f"RTL L0 fired @ sample ≈{l0_sample:.0f}"
                  if l0_fired else "RTL L0 did NOT fire")
    emul_first = int(np.argmax(pre_trig)) if pre_trig.any() else -1
    emul_str   = (f"Python emulation: L0 @ sample {emul_first}"
                  if emul_first >= 0 else "Python emulation: no trigger")

    fig.suptitle(
        f"Event {ev_id} [{ev_type.upper()}]   "
        f"THRESH={thresh} ({thresh_sigma:.1f}σ)   "
        f"HILO_WIN={int(args.hilo_window)}   COINC_WIN={int(args.coinc_window)}   "
        f"BIN_THR={bin_thr}\n"
        f"{sim_l0_str}   |   {emul_str}",
        fontsize=10
    )

    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"  Saved: {out_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    global args   # needed by plot_event for title

    parser = argparse.ArgumentParser(
        description="Plot sanity-check waveforms from tb_sanity.sv output")
    parser.add_argument("--data-dir", default="hw/sim/sanity_data",
                        help="Directory containing sanity_wave.csv and sanity_results.txt")
    parser.add_argument("--thresh",       type=int,   default=300)
    parser.add_argument("--hilo-window",  type=int,   default=10)
    parser.add_argument("--coinc-window", type=int,   default=20)
    parser.add_argument("--bin-thr",      type=int,   default=1)
    parser.add_argument("--sigma-scale",  type=float, default=64.0,
                        help="ADC counts per σ (default 64, since 1σ = 64 counts in ap_fixed<12,6>)")
    args = parser.parse_args()

    data_dir = pathlib.Path(args.data_dir)
    plot_dir = data_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # Load waveform CSV
    # ------------------------------------------------------------------
    wave_path = data_dir / "sanity_wave.csv"
    if not wave_path.exists():
        print(f"ERROR: {wave_path} not found. Run tb_sanity.sv first.")
        raise SystemExit(1)

    wave_df = pd.read_csv(wave_path, comment="#",
                          names=["ev_id", "sample_idx", "ch0", "ch1", "ch2", "ch3"])
    print(f"Loaded waveform: {len(wave_df)} rows, {wave_df['ev_id'].nunique()} events")

    # ------------------------------------------------------------------
    # Load results
    # ------------------------------------------------------------------
    results_path = data_dir / "sanity_results.txt"
    if not results_path.exists():
        print(f"ERROR: {results_path} not found. Run tb_sanity.sv first.")
        raise SystemExit(1)

    results_df = pd.read_csv(results_path, comment="#",
                              names=["ev_id", "type", "l0_fired", "l0_time_ns",
                                     "ev_start_ns", "cnn_fired", "cnn_raw_hex",
                                     "cnn_score_float", "pass"])
    print(f"Loaded results: {len(results_df)} events")
    print(results_df.to_string(index=False))

    # ------------------------------------------------------------------
    # Load golden reference (optional)
    # ------------------------------------------------------------------
    golden_path = data_dir / "sanity_golden.txt"
    golden = {}
    if golden_path.exists():
        with open(golden_path) as f:
            for line in f:
                if line.startswith("#"):
                    continue
                parts = line.split()
                if len(parts) >= 3:
                    try:
                        golden[int(parts[0])] = parts[1]   # ev_id → type
                    except ValueError:
                        pass

    # ------------------------------------------------------------------
    # Per-event plot
    # ------------------------------------------------------------------
    for ev_id in sorted(wave_df["ev_id"].unique()):
        ev_wave = wave_df[wave_df["ev_id"] == ev_id].sort_values("sample_idx")
        data = ev_wave[["ch0", "ch1", "ch2", "ch3"]].values.astype(np.int32)

        if len(data) != 256:
            print(f"  WARNING: event {ev_id} has {len(data)} samples (expected 256). Skipping.")
            continue

        # Look up results for this event
        row = results_df[results_df["ev_id"] == ev_id]
        if row.empty:
            print(f"  WARNING: no results row for event {ev_id}. Skipping.")
            continue
        row = row.iloc[0]

        ev_type   = str(row["type"]).strip()
        l0_fired  = bool(int(row["l0_fired"]))
        cnn_fired = bool(int(row["cnn_fired"]))
        cnn_score = float(row["cnn_score_float"])
        pass_flag = bool(int(row["pass"]))

        # Convert L0 trigger time to sample index within the 256-sample event
        l0_sample = -1.0
        if l0_fired:
            ev_start_ns = float(row["ev_start_ns"])
            l0_time_ns  = float(row["l0_time_ns"])
            # Each ADC clock (32 ns) processes 32 samples.
            # L0 fires ~2 ADC cycles after the triggering batch (pipeline delay).
            adc_clk_ns  = 32.0   # ADC clock period in ns
            batch_elapsed = (l0_time_ns - ev_start_ns) / adc_clk_ns
            # Subtract pipeline delay (~2 cycles) and convert to samples
            l0_sample = max(0.0, (batch_elapsed - 2.0) * 32.0)

        # Python RTL emulation
        gate4, coinc4, mult, pre_trig = emulate_hilo(
            data,
            thresh       = args.thresh,
            hilo_window  = args.hilo_window,
            coinc_window = args.coinc_window,
            bin_thr      = args.bin_thr
        )

        emul_fires = pre_trig.any()
        if emul_fires != l0_fired:
            print(f"  NOTE ev{ev_id}: Python emulation={'fires' if emul_fires else 'no'}, "
                  f"RTL={'fires' if l0_fired else 'no'} — may differ due to batch-boundary effects.")

        fname = f"event_{ev_id}_{ev_type}.png"
        out_path = plot_dir / fname

        print(f"\nPlotting event {ev_id} [{ev_type}] ...")
        plot_event(
            ev_id       = ev_id,
            ev_type     = ev_type,
            data        = data,
            gate4       = gate4,
            coinc4      = coinc4,
            mult        = mult,
            pre_trig    = pre_trig,
            l0_fired    = l0_fired,
            l0_sample   = l0_sample,
            cnn_fired   = cnn_fired,
            cnn_score   = cnn_score,
            thresh      = args.thresh,
            bin_thr     = args.bin_thr,
            sigma_scale = args.sigma_scale,
            pass_flag   = pass_flag,
            out_path    = out_path,
        )

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    n_pass = int(results_df["pass"].sum())
    n_total = len(results_df)
    print(f"\n{'='*50}")
    print(f"  Plots saved to: {plot_dir}")
    print(f"  Test result:    {n_pass}/{n_total} PASSED")
    if n_pass == n_total:
        print("  ALL PASS ✓")
    else:
        fail_rows = results_df[results_df["pass"] == 0]
        for _, r in fail_rows.iterrows():
            print(f"  FAIL: event {int(r['ev_id'])} [{r['type']}]  "
                  f"l0_fired={int(r['l0_fired'])}  cnn_score={float(r['cnn_score_float']):.4f}")
    print(f"{'='*50}\n")


if __name__ == "__main__":
    main()
