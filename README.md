# Hi-Lo Gated CNN Trigger
[![SHL-2.1 license](https://img.shields.io/badge/license-SHL--2.1-green)](LICENSE)

## Introduction

A Hi-Lo Gated CNN Trigger for ARIANNA, a neutrino experiment. This is a submodule for the DAQ System. This module is a 4-channel trigger, composed of a Hi-Lo pre-trigger followed by an AI trigger. [Here](https://github.com/NuDAQ/Hi-Lo-Trigger) is the repository for the Hi-Lo trigger.

### The Structure

#### Module Hierarchy

```
HILO_CNN_TRIGGER              hw/rtl/HILO_CNN_TRIGGER.vhd   (top-level, structural)
├── PRE_TRIGGER               [dep: hilo-trigger v2.1.2]    — L0 bipolar pre-trigger
│   ├── PRE_TRIGGER_1CH × 4
│   └── MULT2BIN × 32
├── CNN_CHUNK_CAPTURE         hw/rtl/CNN_CHUNK_CAPTURE.vhd  — ring buffer + ping-pong + CDC
└── WRAPPER_TOP               [dep: cnn-core-wrapper v1.0.1] — CNN AXI-Stream wrapper
    └── cnn_core              [dep: cnn-core v1.0.4]         — HLS-generated RTL
```

#### Data Flow

```
ADC_DATA4  (4 ch × 32 samples × 12-bit, CLK_ADC domain)
    │
    ├──► PRE_TRIGGER ─────────────────────────────────► L0_PRE_TRIG (out)
    │         bipolar threshold + coincidence window
    │                   │
    │              L0_PRE_TRIG (internal)
    │                   │
    └──► CNN_CHUNK_CAPTURE
              │
              │  CLK_ADC: Pre-trigger ring buffer (4 batches = 128 samples)
              │           Post-trigger capture    (4 batches = 128 samples)
              │           Total chunk: 256 samples → BRAM ping-pong
              │
              │  [True Dual-Port BRAM, 512 × 64-bit]
              │  [4-phase CDC handshake]
              │
              │  CLK_CNN: Stream 256 × 64-bit words to WRAPPER_TOP
              │
              └──► WRAPPER_TOP ──► cnn_core ──► CNN_OUT_DATA / CNN_OUT_VALID
```

#### Clock Domains

| Clock    | Typical frequency | Responsibilities                                      |
|----------|-------------------|-------------------------------------------------------|
| `CLK_ADC`| 31.25 MHz         | ADC ingestion, Hi-Lo trigger, ring buffer, BRAM write |
| `CLK_CNN`| 200 MHz           | CNN streaming (AXI-S), BRAM read, WRAPPER_TOP control |

`RST` is shared (active-high, synchronous to `CLK_ADC`).
`CNN_CHUNK_CAPTURE` re-times it into the `CLK_CNN` domain via a 2-FF synchronizer
and drives the active-low `rst_n` that `WRAPPER_TOP` requires.

#### Chunk Window

| Region       | Batches | Samples | BRAM addresses |
|--------------|---------|---------|----------------|
| Pre-trigger  | 4       | 128     | 0 – 127        |
| Post-trigger | 4       | 128     | 128 – 255      |
| **Total**    | **8**   | **256** | **0 – 255**    |

Alignment is batch-granular (±31 samples): the triggering batch is the first
post-trigger batch (addr 128). Signal events typically span tens of samples,
so batch-level alignment is sufficient.

64-bit word format (one sample across 4 channels):
```
[63:48] ch3 (4-bit pad | 12-bit ADC)
[47:32] ch2
[31:16] ch1
[15: 0] ch0
```

#### Top-Level Port Interface (`HILO_CNN_TRIGGER`)

| Port            | Dir | Width | Clock    | Description                              |
|-----------------|-----|-------|----------|------------------------------------------|
| `CLK_ADC`       | in  | 1     | —        | ADC batch clock                          |
| `CLK_CNN`       | in  | 1     | —        | CNN inference clock                      |
| `RST`           | in  | 1     | CLK_ADC  | Shared active-high synchronous reset     |
| `DATA_STR`      | in  | 1     | CLK_ADC  | Data strobe — one pulse per 32-sample batch |
| `ADC_DATA4`     | in  | 4×32×12 | CLK_ADC | ADC samples, `adc_data4_type`           |
| `THRESH`        | in  | 12    | static   | Absolute threshold (ADC counts)          |
| `HILO_WINDOW`   | in  | 5     | static   | Hi-Lo coincidence window (≤ 16 samples)  |
| `COINC_WINDOW`  | in  | 6     | static   | Channel coincidence smear (≤ 32 samples) |
| `BIN_THR`       | in  | 4     | static   | Min active channels for `L0_PRE_TRIG`   |
| `L0_PRE_TRIG`   | out | 1     | CLK_ADC  | Real-time L0 pre-trigger output          |
| `CNN_OUT_DATA`  | out | 32    | CLK_CNN  | CNN inference score (AXI-S data)         |
| `CNN_OUT_VALID` | out | 1     | CLK_CNN  | AXI-S valid                             |
| `CNN_OUT_READY` | in  | 1     | CLK_CNN  | AXI-S ready                             |
| `CHUNK_OVERFLOW`| out | 1     | CLK_ADC  | Sticky: trigger dropped (buffer full)   |

#### Instantiating for Multiple Antenna Pairs

The module accepts exactly 4 ADC channels. For an 8-channel system (two
antenna pairs), instantiate twice at a higher level:

```vhdl
u_TRIG_A : entity work.HILO_CNN_TRIGGER
    port map (ADC_DATA4 => adc(0 to 3), CLK_ADC => clk_adc, ...);

u_TRIG_B : entity work.HILO_CNN_TRIGGER
    port map (ADC_DATA4 => adc(4 to 7), CLK_ADC => clk_adc, ...);
```

#### Ping-Pong Buffer and Overflow Behaviour

Two BRAM buffers allow one event to be fed to the CNN while the ADC side
captures the next. A third concurrent trigger is dropped and `CHUNK_OVERFLOW`
is set (sticky until `RST`). Wire `CHUNK_OVERFLOW` to an ILA probe or a
slow-control status register for run-time monitoring.

## Build Flow

```bash
# Step 1 — generate CNN RTL from the Keras model (once per model update)
cd <cnn-core checkout>/cnn_core_project
vitis_hls -f build_prj.tcl

# Step 2 — fetch / update all Bender dependencies
bender update

# Step 3 — generate the Vivado source-file script
bender script vivado > add_sources.tcl
```

Open Vivado from the **project root** (not a sub-directory), then:
```tcl
source add_sources.tcl
```
Add `.xdc` constraint files manually (not managed by Bender).

## Developer Notes

- **Updating the CNN model**: replace `models/hgq_config_*.keras` in the
  `cnn-core` repo, re-run `vitis_hls -f build_prj.tcl`, then bump the
  `cnn-core` version in `Bender.yml` and run `bender update`.
- **Changing batch size**: `adc_data_type` (32 samples) is defined in the
  `hilo-trigger` package.  Changing the batch size requires a coordinated
  update of that dependency and all files in this repo.
- **CHUNK_OVERFLOW**: non-zero overflow during a run indicates the CNN
  inference time is longer than the mean trigger inter-arrival time.
  Reduce the trigger rate, increase `BIN_THR`, or widen `THRESH`.
- **Mixed-language simulation**: `HILO_CNN_TRIGGER_TB_WRAP.vhd` bridges the
  flat SV vector to `adc_data4_type`. Packing convention must match the
  generate block in `tb_hilo_cnn_trigger.sv` — both use MSB-first, ch0 at
  the low end.

## License
This project is licensed under the SHL-2.1 License. See the [LICENSE](LICENSE).

---
> The remaining part is for developers. End-users should focus on the above sections only.

## Bender How-To

More information about Bender can be found [here](https://github.com/pulp-platform/bender).

1. Add source files to your working directory or declare new external IPs, in `Bender.yml`.
2. `Bender Update`.
3. `bender script vivado` for the vivado script.

How to write `Bender.yml`?

```yml
package:
  name: my_project
  description: "Description for this project."
  authors:
    - "Albert <albert@example.com>" # current maintainer
    - "Albert <albert@example.com>" # current maintainer

dependencies:
  # METHODOLOGY FIX: Never track a moving branch like 'main'. 
  # Pin to exact semantic versions or commit hashes to guarantee reproducible builds.
  common_cells: { git: "https://github.com/pulp-platform/common_cells.git", version: 1.37.0 }
  mydep: { git: "git@github.com:pulp-platform/common_verification.git", rev: "<commit-ish>" }
  mydep: { git: "git@github.com:pulp-platform/common_verification.git", version: "1.1" }

sources:
  # Source files grouped in levels. Files in level 0 have no dependencies on files in this
  # package. Files in level 1 only depend on files in level 0, files in level 2 on files in
  # levels 1 and 0, etc. Files within a level are ordered alphabetically.
  # Level 0
  - src/axi_pkg.sv
  # Level 1
  - src/axi_intf.sv
  # Level 2
  - src/axi_atop_filter.sv
  - src/axi_burst_splitter_gran.sv
  - src/axi_burst_unwrap.sv

  - target: synth_test
    files:
      - test/axi_synth_bench.sv

  - target: simulation
    files:
      - src/axi_chan_compare.sv
      - src/axi_dumper.sv
      - src/axi_sim_mem.sv
      - src/axi_test.sv
```
