#!/usr/bin/env python3
"""
prepare_sanity_chunks.py

Selects 3 signal events (label=1) and 1 noise event (label=0) from the
cnn-core-wrapper test dataset and writes them to hw/sim/sanity_data/ for
tb_sanity.sv.

Selection criterion for signal events:
  - Any channel must show a BIPOLAR pulse: both a sample > +THRESH and a
    sample < -THRESH within the 256-sample window (Hi-Lo triggerable).

Selection criterion for noise:
  - Peak absolute value < THRESH/2 across all channels (won't trigger Hi-Lo).

Each output hex file: 256 lines of 64-bit hex.
  Line k: [ch3(16-bit)] [ch2(16-bit)] [ch1(16-bit)] [ch0(16-bit)]
  The 16-bit values are sign-extended 12-bit ap_fixed<12,6> integers.
  The lower 12 bits are the raw ADC value fed to the Hi-Lo trigger.

Usage:
  python3 scripts/prepare_sanity_chunks.py [--project-root .] [--thresh 300]

Run from the project root on Ubuntu.
"""

import argparse
import pathlib
import shutil
import numpy as np


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def find_checkout(bender_root: pathlib.Path, name_prefix: str) -> pathlib.Path:
    """Return the first .bender checkout directory matching name_prefix-*."""
    checkouts = bender_root / ".bender" / "git" / "checkouts"
    matches = sorted(checkouts.glob(f"{name_prefix}-*"))
    if not matches:
        raise FileNotFoundError(
            f"No checkout for '{name_prefix}' in {checkouts}. Run 'bender update'.")
    return matches[0]


def load_labels_hex(path: pathlib.Path) -> np.ndarray:
    """Load 1000 32-bit labels from a hex file (one per line)."""
    labels = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                labels.append(int(line, 16))
    return np.array(labels, dtype=np.int32)


def load_hex_chunk(path: pathlib.Path) -> np.ndarray:
    """
    Load 256-line hex file into a (256, 4) int16 array.
    Each line: 64-bit word with [ch3][ch2][ch1][ch0], each 16-bit sign-extended.
    Lower 12 bits of each 16-bit slot = signed 12-bit ADC value.
    """
    samples = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            word = int(line, 16)
            row = []
            for ch in range(4):
                raw16 = (word >> (ch * 16)) & 0xFFFF
                raw12 = raw16 & 0x0FFF
                # Sign-extend 12-bit → Python int
                if raw12 & 0x800:
                    raw12 -= 0x1000
                row.append(raw12)
            samples.append(row)  # [ch0, ch1, ch2, ch3]
    return np.array(samples, dtype=np.int32)  # (256, 4)


def is_bipolar(data: np.ndarray, thresh: int) -> bool:
    """
    True if ANY channel has both a sample > +thresh AND a sample < -thresh.
    This mimics the Hi-Lo trigger's core requirement.
    """
    for ch in range(4):
        col = data[:, ch]
        if col.max() > thresh and col.min() < -thresh:
            return True
    return False


def peak_amplitude(data: np.ndarray) -> int:
    """Max absolute value across all channels and samples."""
    return int(np.abs(data).max())


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Prepare sanity-check chunks for tb_sanity.sv")
    parser.add_argument("--project-root", default=".",
                        help="Project root directory (default: current directory)")
    parser.add_argument("--thresh", type=int, default=300,
                        help="Hi-Lo threshold in 12-bit ADC counts (default: 300 ≈ 4.7σ). "
                             "Signal events must exceed ±THRESH on at least one channel.")
    parser.add_argument("--n-signals", type=int, default=3,
                        help="Number of signal chunks to select (default: 3)")
    parser.add_argument("--scan-limit", type=int, default=200,
                        help="Max number of signal/noise candidates to scan (default: 200)")
    args = parser.parse_args()

    root = pathlib.Path(args.project_root).resolve()
    out_dir = root / "hw" / "sim" / "sanity_data"
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Project root : {root}")
    print(f"Output dir   : {out_dir}")
    print(f"THRESH       : {args.thresh}  (12-bit ADC counts, ~{args.thresh/64:.1f}σ if σ=64 counts)")

    # ------------------------------------------------------------------
    # Locate checkouts
    # ------------------------------------------------------------------
    try:
        wrapper_dir = find_checkout(root, "cnn-core-wrapper")
    except FileNotFoundError as e:
        print(f"ERROR: {e}")
        raise SystemExit(1)

    testhex_dir = (wrapper_dir / "cnn_core_wrapper" /
                   "cnn_core_wrapper.sim" / "sim_1" / "behav" / "xsim" / "testhex_stream")
    if not testhex_dir.exists():
        print(f"ERROR: testhex_stream not found at {testhex_dir}")
        print("       Run the cnn-core-wrapper Vivado simulation once to generate test data.")
        raise SystemExit(1)

    # ------------------------------------------------------------------
    # Load labels
    # ------------------------------------------------------------------
    labels_path = testhex_dir / "labels.hex"
    labels = load_labels_hex(labels_path)
    print(f"Loaded {len(labels)} labels — {labels.sum()} signal, {(labels==0).sum()} noise")

    signal_indices = np.where(labels == 1)[0]
    noise_indices  = np.where(labels == 0)[0]

    # ------------------------------------------------------------------
    # Scan signal events: find bipolar events with highest peak amplitude
    # ------------------------------------------------------------------
    print(f"\nScanning signal events (limit={args.scan_limit}) for bipolar waveforms ...")
    signal_candidates = []  # list of (peak_amp, sample_id, data)

    for idx in signal_indices[:args.scan_limit]:
        hex_path = testhex_dir / f"test_input_sample{idx}.hex"
        if not hex_path.exists():
            continue
        data = load_hex_chunk(hex_path)
        if is_bipolar(data, args.thresh):
            peak = peak_amplitude(data)
            signal_candidates.append((peak, int(idx), data))

    signal_candidates.sort(key=lambda x: -x[0])  # sort by amplitude descending

    if len(signal_candidates) < args.n_signals:
        print(f"  WARNING: only {len(signal_candidates)} bipolar signals found. "
              f"Try lowering --thresh (currently {args.thresh}).")
    else:
        print(f"  Found {len(signal_candidates)} bipolar signal candidates. "
              f"Selecting top {args.n_signals}.")

    selected_signals = signal_candidates[:args.n_signals]

    # ------------------------------------------------------------------
    # Select noise event (label=0).
    #
    # ARIANNA context: thermal noise can have large transient amplitudes,
    # so ALL events (signal and noise) may have large peak values.  We do
    # NOT require a "quiet" noise event.  The Hi-Lo trigger may fire on
    # noise too — the CNN is the actual discriminator.  We pick the noise
    # event with the LOWEST peak so it is less likely to be confused with
    # a signal by the CNN.
    # ------------------------------------------------------------------
    print(f"\nSelecting noise event (label=0) with lowest peak amplitude ...")
    noise_candidates = []

    for idx in noise_indices[:args.scan_limit]:
        hex_path = testhex_dir / f"test_input_sample{idx}.hex"
        if not hex_path.exists():
            continue
        data = load_hex_chunk(hex_path)
        noise_candidates.append((peak_amplitude(data), int(idx), data))

    noise_candidates.sort(key=lambda x: x[0])  # quietest first

    if not noise_candidates:
        print("  ERROR: no label=0 events found in dataset.")
        raise SystemExit(1)

    selected_noise = noise_candidates[0]
    peak_n, idx_n, _ = selected_noise
    bipolar_n = is_bipolar(_, args.thresh)
    print(f"  Selected sample {idx_n}  peak={peak_n} ({peak_n/64:.1f}σ)  "
          f"bipolar_at_thresh={'YES — Hi-Lo will fire' if bipolar_n else 'no'}")

    # ------------------------------------------------------------------
    # Copy hex files and write golden reference
    # ------------------------------------------------------------------
    print("\n--- Selected chunks ---")
    golden_lines = [
        "# chunk_id  type    sample_id  peak_amp_counts  peak_amp_sigma  bipolar_thresh\n"
    ]
    chunks_info = []

    for i, (peak, idx, data) in enumerate(selected_signals):
        src = testhex_dir / f"test_input_sample{idx}.hex"
        dst = out_dir / f"chunk_sig{i}.hex"
        shutil.copy(src, dst)
        sigma = peak / 64.0
        print(f"  chunk_sig{i}.hex  <- sample {idx:4d}  peak={peak:5d} ({sigma:.1f}σ)  [SIGNAL]")
        golden_lines.append(f"{i}  signal  {idx:4d}  {peak:5d}  {sigma:.2f}  label=1\n")
        chunks_info.append({"id": i, "type": "signal", "sample_id": idx,
                             "peak": peak, "path": dst})

    peak, idx, data = selected_noise
    src = testhex_dir / f"test_input_sample{idx}.hex"
    dst = out_dir / "chunk_noise0.hex"
    shutil.copy(src, dst)
    sigma = peak / 64.0
    noise_chunk_id = args.n_signals
    print(f"  chunk_noise0.hex <- sample {idx:4d}  peak={peak:5d} ({sigma:.1f}σ)  [NOISE]")
    golden_lines.append(
        f"{noise_chunk_id}  noise   {idx:4d}  {peak:5d}  {sigma:.2f}  label=0\n")
    chunks_info.append({"id": noise_chunk_id, "type": "noise", "sample_id": idx,
                        "peak": peak, "path": dst})

    golden_path = out_dir / "sanity_golden.txt"
    with open(golden_path, "w") as f:
        f.writelines(golden_lines)
    print(f"\nGolden reference -> {golden_path}")

    # ------------------------------------------------------------------
    # Print recommended tb_sanity.sv parameter overrides
    # ------------------------------------------------------------------
    print("\n--- Recommended tb_sanity.sv parameters ---")
    print(f"  P_THRESH       = {args.thresh}")
    print(f"  P_HILO_WINDOW  = 10")
    print(f"  P_COINC_WINDOW = 20")
    print(f"  P_BIN_THR      = 1   (single-channel, easiest to trigger)")
    print(f"\n  If L0 does NOT fire for signal events, lower P_THRESH or P_BIN_THR.")
    print(f"  If L0 fires for the noise event, raise P_THRESH.\n")

    # ------------------------------------------------------------------
    # Quick waveform preview (matplotlib)
    # ------------------------------------------------------------------
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(args.n_signals + 1, 4, figsize=(16, 3 * (args.n_signals + 1)),
                                  sharey=False, sharex=True)
        fig.suptitle(f"Sanity chunk preview  (THRESH={args.thresh} counts = "
                     f"{args.thresh/64:.1f}σ)", fontsize=12)

        for row, info in enumerate(chunks_info):
            data = load_hex_chunk(info["path"])
            t = np.arange(256)
            for ch in range(4):
                ax = axes[row, ch]
                ax.plot(t, data[:, ch], lw=0.8, color=f"C{ch}")
                ax.axhline( args.thresh, color="r", lw=0.7, ls="--", label="±THRESH")
                ax.axhline(-args.thresh, color="r", lw=0.7, ls="--")
                ax.set_ylabel("ADC counts")
                ax.set_title(f"{'sig' if info['type']=='signal' else 'noise'} "
                             f"s{info['sample_id']} / ch{ch}")
                ax.legend(fontsize=6)

        for ax in axes[-1, :]:
            ax.set_xlabel("Sample index")

        preview_path = out_dir / "chunk_preview.png"
        fig.tight_layout()
        fig.savefig(preview_path, dpi=120)
        plt.close(fig)
        print(f"Preview plot -> {preview_path}")
    except ImportError:
        print("matplotlib not available — skipping preview plot.")

    print("\nDone. Files written to:", out_dir)
    print("Next: run tb_sanity.sv in Vivado XSim, then plot_sanity_results.py.\n")


if __name__ == "__main__":
    main()
