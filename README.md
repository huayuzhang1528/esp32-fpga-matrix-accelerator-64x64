# ESP32 + FPGA 4x4-Datapath / 64x64 Matrix Accelerator

This is a separate implementation derived from the original project. It does
not overwrite the original Verilog or Arduino sketch.

## Locked design

- FPGA: Gowin `GW1NSR-LV4CQN48PC6/I5` (Tang Nano 4K class device)
- Input: signed INT8
- Accumulation and output: signed INT32
- Matrix sizes: multiples of four from 4x4 through 64x64
- Compute tile: 2x4, using the full 4x4-equivalent multiplier datapath
- Physical datapath: eight dual-multiply-accumulate DSP blocks, using all
  sixteen 18x18 multipliers in the device
- B/weight matrix remains resident in FPGA memory
- A is streamed two rows at a time
- C is returned one 2x4 tile at a time

The streaming layout avoids storing a full INT32 result matrix on the FPGA.
The parallel compute path needs eight independently readable B banks; that
banking consumes eight of the ten physical BSRAM blocks. A two-row block maps
directly to the eight DSP datapaths and avoids exceeding the FPGA register
capacity while B grows to 64x64.

## Files

- `DEVELOPMENT_REPORT_CN.md`: complete Chinese engineering retrospective,
  including successful changes, failed experiments, measurements, and the
  debugging method used throughout the project
- `fpga/src/ESP32_FPGA_4x4_64x64.v`: validated 64x64 FPGA implementation
- `fpga/src/ESP32_FPGA_4x4_64x64.cst`: high-speed logical pin mapping
- `fpga/build/ESP32_FPGA_4x4_64x64.fs`: validated 64x64 SRAM bitstream
- `fpga/build/ESP32_FPGA_4x4_highspeed.fs`: validated 32x32/20 MHz fallback
- `fpga/build/ESP32_FPGA_4x4_100k_reliable.fs`: conservative fallback bitstream
- `fpga/ESP32_FPGA_4x4.gprj`: Gowin project
- `esp32/ESP32_FPGA_Matrix_Accelerator_64x64/ESP32_FPGA_Matrix_Accelerator_64x64.ino`:
  standalone ESP32 bring-up, correctness, and optional benchmark sketch
- `sim/tb_ESP32_FPGA_4x4.v`: signed 4x4 protocol testbench with known results

## Wiring

The physical wires are unchanged, but their logical roles are remapped so SCLK
uses a clock-capable FPGA pin and the former SCLK wire carries MISO.

| Signal | ESP32 | FPGA package pin |
|---|---:|---:|
| SCLK | GPIO 4 | 42 |
| MISO | GPIO 18 | 40 |
| MOSI | GPIO 23 | 31 |
| CS | GPIO 5 | 39 |
| DONE | GPIO 19 | 32 |
| GND | GND | GND |

Both boards use 3.3 V logic. Grounds must be connected.

## Bring-up

1. Open `fpga/ESP32_FPGA_4x4.gprj` in Gowin IDE and generate the bitstream.
2. Program the FPGA.
3. Open and flash the new Arduino sketch.
4. Open Serial Monitor at 115200 baud.
5. Run these commands in order:

```text
test 4
test 8
test 16
test 32
test 64
bench 64
```

The 64x64 sketch starts at the hardware-validated 19 MHz setting. `speed 0` selects
the conservative 100 kHz fallback. `speed M` accepts integer requests from 1
through 40 MHz for diagnostics. The 64x64 build passed through 19 MHz; 20 MHz
produced deterministic input-data errors after the deeper BSRAM routing, so it
is deliberately not used as the default.

`run N` is the final-style path: it performs only FPGA computation and prints a
completion checksum. It does not calculate a CPU reference or print latency.
`test N` and `bench N` are explicit development commands.

## Synthesized resource check

The standalone RTL completed synthesis, placement, routing, and bitstream
generation with Gowin V1.9.11.03 for the exact project part:

| Resource | Usage |
|---|---:|
| DSP blocks (`MULTADDALU18X18`) | 8 / 8 |
| BSRAM | 8 / 10 |
| Logic | 1012 / 4608 (22%) |
| Registers | 1220 / 3573 (35%) |

This confirms that the full DSP datapath and 64x64 streaming storage fit with
substantial logic/register margin. An 8x8 physical multiplier array is still
not realistic because all eight DSP blocks are already used.

## Measured hardware results

The final 19 MHz build passed full signed-INT8 result comparison at 4x4, 8x8,
16x16, 32x32, and 64x64, including repeated 64x64 runs. Benchmarks keep B
resident after a one-time load and include A transfer, FPGA execution, DONE
polling, and C readback in the FPGA path. The one-time B load is reported
separately (about 2.18 ms for 64x64).

| Size | ESP32 reference | FPGA resident-B path | End-to-end speedup |
|---:|---:|---:|---:|
| 4x4 | 0.007 ms | 0.071 ms | 0.097x |
| 8x8 | 0.050 ms | 0.268 ms | 0.188x |
| 16x16 | 0.389 ms | 1.032 ms | 0.377x |
| 32x32 | 3.060 ms | 4.001 ms | 0.765x |
| 64x64 | 24.297 ms | 15.810 ms | 1.537x |

The measured crossover occurs between 32x32 and 64x64: smaller matrices remain
slower because fixed SPI and command overhead dominates, while 64x64 obtains a
repeatable 1.537x end-to-end resident-B speedup. This is the defensible project
claim; it should not be presented as a speedup for every matrix size.

## Security note

The original Arduino sketch contains a Wi-Fi password and Telegram Bot token
in source code. This new sketch intentionally contains neither. Revoke the
exposed Bot token, change the exposed Wi-Fi password, and keep replacements in
an ignored `secrets.h` file rather than committing them.
