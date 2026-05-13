#!/usr/bin/env python3
"""
prepare_thermal_sim.py

Converts hilo-trigger thermal noise .npy files into a flat stimulus.txt
suitable for tb_thermal.sv.

Mirrors the logic of hilo-trigger/analysis/scripts/submodule/stimulus_generation.py
but updated for N_SAMPLES=16 (hilo-trigger v2.2.3).

Input .npy shape: (events, 4, total_samples)  — float values in σ units
  total_samples is typically 512 (32 batches × 16 samples each).

Output stimulus.txt:
  One line per 16-sample batch.
  Each line: 64 space-separated integers (ch0×16, ch1×16, ch2×16, ch3×16).
  Events are concatenated without inter-event gaps or resets — the RTL
  sees a continuous noise stream and triggers naturally.

Usage:
  python3 scripts/prepare_thermal_sim.py \\
      --data-dir  <path/to/thermal_chunk_*.npy files> \\
      --out-dir   hw/sim/thermal_data \\
      [--n-chunks 5]       # number of .npy chunk files to use (default: all)
      [--scale    64.0]    # ADC counts per σ  (default: 64)
      [--skip-if-exists]   # skip if stimulus.txt already present

Outputs:
  hw/sim/thermal_data/stimulus.txt   — stimulus for tb_thermal.sv
  hw/sim/thermal_data/stim_meta.txt  — metadata (events, batches, threshold hint)
"""

import argparse
import pathlib
import glob
import numpy as np

N_SAMPLES = 16  # must match PRE_TRIGGER_PKG.vhd N_SAMPLES constant


def write_stimulus(npy_paths: list[pathlib.Path],
                   out_path: pathlib.Path,
                   scale: float) -> tuple[int, int, float]:
    """
    Convert .npy files to stimulus.txt.

    Returns (total_events, total_batches, noise_rms).
    noise_rms is computed from the first chunk's first 1000 events.
    """
    total_events  = 0
    total_batches = 0
    noise_rms     = None

    with open(out_path, 'w') as f:
        for chunk_path in npy_paths:
            data = np.load(chunk_path)
            data = np.squeeze(data)               # ensure (events, 4, samples)
            if data.ndim != 3:
                print(f"  [SKIP] {chunk_path.name}: unexpected shape {data.shape}")
                continue

            events, channels, total_samps = data.shape

            if noise_rms is None:
                sample_slice = data[:min(1000, events)]
                noise_rms = float(np.std(sample_slice))
                print(f"  Noise RMS from first chunk: {noise_rms:.6f} σ-units"
                      f"  ({noise_rms * scale:.1f} ADC counts)")

            batches_per_event = total_samps // N_SAMPLES
            adc = np.round(data * scale).astype(np.int32)
            adc = np.clip(adc, -2048, 2047)

            for ev in range(events):
                for b in range(batches_per_event):
                    start = b * N_SAMPLES
                    end   = start + N_SAMPLES
                    chunk = adc[ev, :, start:end]   # (4, 16)
                    flat  = chunk.flatten()          # ch0×16, ch1×16, ch2×16, ch3×16
                    f.write(" ".join(map(str, flat)) + "\n")
                    total_batches += 1

            total_events += events
            print(f"  {chunk_path.name}: {events} events × {batches_per_event} batches")

    return total_events, total_batches, noise_rms or 1.0


def main():
    ap = argparse.ArgumentParser(
        description="Prepare thermal-noise stimulus for tb_thermal.sv")
    ap.add_argument("--data-dir",
                    default="/home/work1/Works/Hi-Lo-Trigger/analysis/data/thermal",
                    help="Directory containing thermal_chunk_*.npy files")
    ap.add_argument("--out-dir",   default="hw/sim/thermal_data",
                    help="Output directory (default: hw/sim/thermal_data)")
    ap.add_argument("--n-chunks",  type=int, default=0,
                    help="Number of .npy files to use (0 = all, default: all)")
    ap.add_argument("--scale",     type=float, default=64.0,
                    help="ADC counts per σ (default: 64.0)")
    ap.add_argument("--skip-if-exists", action="store_true",
                    help="Skip generation if stimulus.txt already exists")
    args = ap.parse_args()

    data_dir = pathlib.Path(args.data_dir)
    out_dir  = pathlib.Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    stim_path = out_dir / "stimulus.txt"
    meta_path = out_dir / "stim_meta.txt"

    if args.skip_if_exists and stim_path.exists():
        print(f"[SKIP] {stim_path} already exists.")
        return

    npy_files = sorted(data_dir.glob("thermal_chunk_*.npy"))
    if not npy_files:
        npy_files = sorted(data_dir.glob("*.npy"))
    if not npy_files:
        print(f"[ERROR] No .npy files found in {data_dir}")
        raise SystemExit(1)

    if args.n_chunks > 0:
        npy_files = npy_files[:args.n_chunks]

    print(f"Data dir   : {data_dir}")
    print(f"Output dir : {out_dir}")
    print(f"N_SAMPLES  : {N_SAMPLES}  (must match PRE_TRIGGER_PKG.vhd)")
    print(f"Scale      : {args.scale} ADC counts/σ")
    print(f"Files      : {len(npy_files)}")
    print()

    total_events, total_batches, noise_rms = write_stimulus(
        npy_files, stim_path, args.scale)

    # Threshold hint: 3σ for false-trigger analysis
    thr_3sigma = int(3.0 * noise_rms * args.scale)
    thr_4sigma = int(4.0 * noise_rms * args.scale)

    with open(meta_path, 'w') as f:
        f.write(f"n_samples_per_batch  {N_SAMPLES}\n")
        f.write(f"scale_factor         {args.scale}\n")
        f.write(f"total_events         {total_events}\n")
        f.write(f"total_batches        {total_batches}\n")
        f.write(f"noise_rms_sigma      {noise_rms:.6f}\n")
        f.write(f"thresh_3sigma_adc    {thr_3sigma}\n")
        f.write(f"thresh_4sigma_adc    {thr_4sigma}\n")

    print()
    print(f"Written : {stim_path}  ({total_batches} lines)")
    print(f"Written : {meta_path}")
    print()
    print("Threshold hints (for tb_thermal.sv P_THRESH parameter):")
    print(f"  3σ → {thr_3sigma} ADC counts")
    print(f"  4σ → {thr_4sigma} ADC counts")
    print()
    print("Next: run tb_thermal.sv via scripts/run_thermal_sim.sh")


if __name__ == "__main__":
    main()
