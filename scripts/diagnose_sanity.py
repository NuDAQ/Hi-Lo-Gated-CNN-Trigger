#!/usr/bin/env python3
"""
diagnose_sanity.py

Analyses hw/sim/sanity_data/ files to catch data-format or threshold issues
BEFORE or AFTER running the simulation.  Useful when the simulation fails
and you need to know whether the problem is in the data or in the RTL.

Checks performed
----------------
1. Hex file existence and format (256 lines, valid 64-bit words).
2. ADC value range — are values reasonable 12-bit signed integers?
3. Per-channel amplitude — does any channel cross ±THRESH both ways (bipolar)?
4. Python Hi-Lo emulation — does the trigger fire for signal events?
5. Consistency between sanity_wave.csv (simulation output) and the hex files
   (what we expected the simulation to drive).
6. Simulation results summary — what did the RTL actually report?

Usage
-----
  python3 scripts/diagnose_sanity.py [--data-dir hw/sim/sanity_data] [--thresh 300]
"""

import argparse
import pathlib
import sys
import numpy as np


# ---------------------------------------------------------------------------
# Data loading helpers
# ---------------------------------------------------------------------------

def load_hex_chunk(path: pathlib.Path):
    """Return (256, 4) int32 array from a 256-line, 64-bit-per-line hex file."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            word = int(line, 16)
            ch = []
            for c in range(4):
                raw16 = (word >> (c * 16)) & 0xFFFF
                raw12 = raw16 & 0x0FFF
                if raw12 & 0x800:
                    raw12 -= 0x1000          # sign-extend 12-bit
                ch.append(raw12)
            rows.append(ch)
    return np.array(rows, dtype=np.int32)   # (256, 4)


def emulate_hilo(data, thresh, hilo_window=10, coinc_window=20, bin_thr=1):
    """Boolean Hi-Lo emulation; returns (pre_trig bool array, first_fire int or -1)."""
    N = len(data)
    hilo_window  = min(hilo_window,  16)
    coinc_window = min(coinc_window, 32)
    gate4 = np.zeros((N, 4), dtype=bool)
    for ch in range(4):
        col    = data[:, ch].astype(np.int32)
        ot_hi  = col >  thresh
        ot_lo  = col < -thresh
        g_hi   = np.zeros(N, dtype=bool)
        g_lo   = np.zeros(N, dtype=bool)
        for i in range(N):
            s = max(0, i - hilo_window + 1)
            g_hi[i] = ot_hi[s:i+1].any()
            g_lo[i] = ot_lo[s:i+1].any()
        gate4[:, ch] = g_hi & g_lo
    coinc4 = np.zeros((N, 4), dtype=bool)
    for ch in range(4):
        for i in range(N):
            s = max(0, i - coinc_window + 1)
            coinc4[i, ch] = gate4[s:i+1, ch].any()
    mult     = coinc4.sum(axis=1)
    pre_trig = mult >= bin_thr
    first    = int(np.argmax(pre_trig)) if pre_trig.any() else -1
    return pre_trig, first


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------

def check_hex_file(path: pathlib.Path, label: str) -> bool:
    ok = True
    if not path.exists():
        print(f"  [MISSING] {label}: {path}")
        return False
    try:
        data = load_hex_chunk(path)
    except Exception as e:
        print(f"  [ERROR]   {label}: parse failed — {e}")
        return False
    if len(data) != 256:
        print(f"  [WARN]    {label}: {len(data)} lines (expected 256)")
        ok = False
    peak = int(np.abs(data).max())
    mn, mx = int(data.min()), int(data.max())
    print(f"  [OK]      {label}: 256 samples, range [{mn}, {mx}], peak_abs={peak}")
    if peak > 2047:
        print(f"            WARNING: peak {peak} > 2047 — values exceed 12-bit signed range!")
        ok = False
    return ok


def check_threshold(data: np.ndarray, label: str, thresh: int) -> bool:
    bipolar_chs = []
    for ch in range(4):
        col = data[:, ch]
        if col.max() > thresh and col.min() < -thresh:
            bipolar_chs.append(ch)
    if bipolar_chs:
        print(f"  [OK]      {label}: bipolar above ±{thresh} on ch{bipolar_chs} → Hi-Lo CAN fire")
        return True
    else:
        pmax = int(np.abs(data).max())
        print(f"  [WARN]    {label}: NO channel has bipolar crossings above ±{thresh}. "
              f"peak_abs={pmax}. Raise thresh or check data.")
        return False


def check_emulation(data: np.ndarray, label: str, thresh: int, expect_fire: bool) -> bool:
    _, first = emulate_hilo(data, thresh)
    fired = first >= 0
    match = fired == expect_fire
    status = "[OK]  " if match else "[FAIL]"
    fire_str = f"fires at sample {first}" if fired else "does not fire"
    print(f"  {status}    {label}: Python Hi-Lo emulation {fire_str} "
          f"(expected={'fire' if expect_fire else 'no fire'})")
    return match


def check_wave_csv(csv_path: pathlib.Path, hex_files: list[pathlib.Path]):
    """Compare simulation-driven waveform to what hex files contain."""
    if not csv_path.exists():
        print(f"  [MISSING] sanity_wave.csv — simulation not yet run")
        return
    import csv
    rows = []
    with open(csv_path) as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split(',')
            if len(parts) == 6:
                rows.append([int(x) for x in parts])
    rows = np.array(rows)    # (N, 6): ev_id, sample_idx, ch0..ch3
    if len(rows) == 0:
        print("  [WARN]    sanity_wave.csv is empty")
        return
    max_diff = 0
    for ev_id, hf in enumerate(hex_files):
        if not hf.exists():
            continue
        expected = load_hex_chunk(hf)       # (256, 4)
        sim_rows  = rows[rows[:, 0] == ev_id]
        if len(sim_rows) != 256:
            print(f"  [WARN]    ev{ev_id}: simulation drove {len(sim_rows)} samples "
                  f"(expected 256)")
            continue
        sim_rows = sim_rows[sim_rows[:, 1].argsort()]
        for s in range(256):
            for ch in range(4):
                diff = abs(int(sim_rows[s, ch + 2]) - int(expected[s, ch]))
                if diff > max_diff:
                    max_diff = diff
    if max_diff == 0:
        print(f"  [OK]      sanity_wave.csv matches hex files exactly (max diff = 0)")
    elif max_diff <= 1:
        print(f"  [OK]      sanity_wave.csv matches hex files (max diff = {max_diff}, rounding)")
    else:
        print(f"  [WARN]    sanity_wave.csv diverges from hex files (max diff = {max_diff}) "
              f"— possible data packing mismatch!")


def check_results(results_path: pathlib.Path):
    if not results_path.exists():
        print(f"  [MISSING] sanity_results.txt — simulation not yet run")
        return
    with open(results_path) as f:
        lines = [l.strip() for l in f if not l.startswith('#') and l.strip()]
    print(f"  Simulation results ({len(lines)} events):")
    for line in lines:
        parts = line.split(',')
        if len(parts) >= 9:
            ev_id, typ, l0, l0t, evst, cnn, raw, score, passed = parts[:9]
            status = "PASS" if passed.strip() == '1' else "FAIL"
            print(f"    ev{ev_id} [{typ}]: L0={'yes' if l0=='1' else 'no'}, "
                  f"CNN={'yes' if cnn=='1' else 'no'}, "
                  f"score={float(score):.3f}, {status}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Diagnose sanity simulation data")
    ap.add_argument("--data-dir", default="hw/sim/sanity_data")
    ap.add_argument("--thresh",       type=int, default=300)
    ap.add_argument("--hilo-window",  type=int, default=10)
    ap.add_argument("--coinc-window", type=int, default=20)
    ap.add_argument("--bin-thr",      type=int, default=1)
    args = ap.parse_args()

    data_dir = pathlib.Path(args.data_dir)
    T = args.thresh

    print("=" * 60)
    print(f"  Sanity data diagnosis")
    print(f"  Data dir : {data_dir}")
    print(f"  THRESH   : {T}  (~{T/64:.1f}σ if σ=64 counts)")
    print("=" * 60)

    chunks = {
        "chunk_sig0":   (data_dir / "chunk_sig0.hex",   True),
        "chunk_sig1":   (data_dir / "chunk_sig1.hex",   True),
        "chunk_sig2":   (data_dir / "chunk_sig2.hex",   True),
        "chunk_noise0": (data_dir / "chunk_noise0.hex", False),
    }

    all_ok = True
    loaded = {}

    # ── 1. File existence and format ─────────────────────────────────────────
    print("\n[1] Hex file existence and format:")
    for label, (path, _) in chunks.items():
        ok = check_hex_file(path, label)
        all_ok = all_ok and ok
        if ok:
            loaded[label] = load_hex_chunk(path)

    # ── 2. Amplitude range ───────────────────────────────────────────────────
    print(f"\n[2] Bipolar threshold check (THRESH={T}):")
    for label, (path, expect_fire) in chunks.items():
        if label in loaded:
            ok = check_threshold(loaded[label], label, T)
            all_ok = all_ok and (ok == expect_fire or not expect_fire)

    # ── 3. Python Hi-Lo emulation ────────────────────────────────────────────
    print(f"\n[3] Python Hi-Lo emulation "
          f"(win={args.hilo_window}, coinc={args.coinc_window}, thr={args.bin_thr}):")
    for label, (path, expect_fire) in chunks.items():
        if label in loaded:
            ok = check_emulation(loaded[label], label, T, expect_fire)
            all_ok = all_ok and ok

    # ── 4. Simulation waveform consistency ───────────────────────────────────
    print("\n[4] Simulation waveform vs hex files:")
    hex_list = [data_dir / f"chunk_{n}.hex"
                for n in ["sig0", "sig1", "sig2", "noise0"]]
    check_wave_csv(data_dir / "sanity_wave.csv", hex_list)

    # ── 5. Simulation results ────────────────────────────────────────────────
    print("\n[5] Simulation results:")
    check_results(data_dir / "sanity_results.txt")

    # ── Summary ──────────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    if all_ok:
        print("  Data looks correct. If simulation still fails, the issue")
        print("  is in the RTL connection or initialization, not the data.")
    else:
        print("  Issues found — fix data or threshold before re-running sim.")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    main()
