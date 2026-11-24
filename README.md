# c5div: dual ESP32-C5 diversity video receiver for Raspberry Pi

Two [C5VRX](https://github.com/Twotoz/C5VRX) receivers (Seeed XIAO ESP32-C5
running the C5VRX firmware) feed a **Raspberry Pi Zero 2W or Pi 3B**. The Pi
combines both composite streams in software (diversity), decodes the result,
and outputs the picture on **HDMI** and/or as a **web stream
over a Wi-Fi access point** that phones and laptops can view.

```
 VTX 5.8 GHz
   │                 6-bit DAC bus (digital, tapped before the resistors)
   ├─► XIAO C5 #A ──────────────┐
   └─► XIAO C5 #B ──────────┐   │   SMI 16-bit @ 25 MS/s (async)
                            ▼   ▼
                  Pi Zero 2W / Pi 3B
   capture ring ─► A: code clock + sync ─┐
                └► B: code clock + sync ─┴─► fusion ─► decoder ─► bob deinterlace + OSD ─┬─► HDMI (KMS, YUV plane)
                                          (composite)                                    └─► MJPEG web viewer (Wi-Fi AP)
```

## Credit: C5VRX

The receivers run **C5VRX** by Twotoz and the C5VRX contributors
([source](https://github.com/Twotoz/C5VRX), [official website](https://twotoz.github.io/C5VRX/),
where the C5VRX Discord invite is available). The Pi side, c5div, is new code,
but it relies on C5VRX's documented findings and interface: the 6-bit DAC pin
order, the bit-2 move to D3 from C5VRX's *Clock Pin D10* option and LAP87's
capture tests, the DAC code levels (sync 0, blanking 20, about 18 codes of sync
depth) and the 20 MS/s code cadence on a 40 MHz TX clock. The synthetic source
in `src/source_synth.c` models those levels. `firmware/` is
**c5div-rx**, a reduced fork of the C5VRX-5 firmware for this setup (one
button, raceband only, bit 2 fixed on D3). It is GPL-3.0-only with its own
attribution and list of modifications in `firmware/README.md`. Neither c5div
nor c5div-rx is an official C5VRX release.

## Why the digital bus?

C5VRX demodulates the FM video *on the C5* (in its BitScrambler) and drives a
6-bit resistor DAC. The C5 cannot export raw IQ fast enough: its USB is
full-speed, and raw IQ would need 40–80 MB/s. The demodulated 6-bit bus,
though, is only 20 MS/s of codes, and the Pi's SMI peripheral can sample it
directly. No analog capture hardware is needed, and both receivers share one
sample clock and timeline. That shared timeline is what makes exact line-by-line diversity
possible.

## What the decoder does

Per receiver (`src/decoder.c`), all adaptive because C5VRX output levels
follow the VTX deviation and the C5's OUTPUT setting:

- **Sync slicer:** box filter plus a Schmitt trigger with sub-sample edge interpolation.
- **Line PLL:** tracks hsync, coasts through dropouts, re-phases after capture gaps.
- **Vertical:** detects broad pulses, determines field parity, and runs a vertical flywheel.
  It auto-detects **PAL / NTSC** from lines per field.
- **Levels:** measures sync tip and blanking per line (AGC and clamp).
- **Code clock recovery** (`src/reclock.c`): the C5 holds each code for 50 ns
  and the Pi samples every 40 ns from its own clock, so one sample in five is a
  repeat. A narrow PLL tracks which one (it slips every ~1.7 ms) and each code is taken
  once, from the middle of its hold. Without it the hold images fold into the
  picture as a diagonal pattern, and samples taken on changing bits show as
  coloured streaks at every slip.
- **TBC:** resamples every line onto the BT.601 13.5 MHz grid (720 px).
- **Colour:** burst-locked quadrature demodulation with an absolute-time
  subcarrier oscillator, PAL V-switch from consecutive bursts, ACC, a colour
  killer, and luma by chroma remodulation/subtraction.
- **Quality:** per-line SNR from porch noise, which drives the diversity choice.

With two receivers the default is **fuse** (`src/fusion.c`): the two composite
streams are aligned to a few ns, weighted by each receiver's noise line by line,
FM clicks on one are replaced from the other, and one decoder decodes the
result. The alternative combiner (`src/diversity.c`, `--div auto|select|blend`)
runs two full decoders, pairs their fields, and per line **selects** the
better receiver (with hysteresis) or **blends** both. Lines that are missing (sync loss, capture gaps) keep the
previous frame's content instead of smearing.

## Quick start without hardware

Everything runs on a normal Linux PC against a built-in **C5 emulator**:
composite plus 6-bit quantisation, a 20 MS/s hold, async 25 MS/s sampling, noise,
FM clicks, fades, and SMI gaps.

```
sudo dnf install cmake gcc libdrm-devel libjpeg-turbo-devel      # or apt equivalents
cmake -S . -B build && cmake --build build -j
./build/test_pipeline                         # end-to-end tests
./build/c5div run -s synth:pal --synth-fade 3 -o web --web-port 8080
#   open http://localhost:8080/   (A and B fade alternately; watch the bars)
./build/c5div run -s synth:ntsc --fast --seconds 2 -o y4m:out.y4m   # then: ffplay out.y4m
```

## On the Pi

See **[docs/WIRING.md](docs/WIRING.md)** for the pin table, the required C5VRX
setting (EXPERIMENTAL → Clock Pin D10 = ON), and probe output.

```
sudo ./pi/install.sh                    # deps, build, SMI overlay, config.txt, service
sudo reboot
sudo c5div probe                        # wiring and signal check
sudo c5div run                          # HDMI + web viewer on :8080
sudo ./pi/setup-ap.sh MYSSID MYPASSWORD # optional: Wi-Fi AP at 192.168.50.1
sudo systemctl enable --now c5div       # optional: start at boot (/etc/default/c5div)
sudo c5div record --seconds 5 -O cap.raw   # raw capture, replay with -s file:cap.raw
```

### Outputs

- **HDMI** (`-o hdmi`): DRM/KMS atomic. It asks for 720×576@50 (PAL) or
  720×480@60 (NTSC) when the display supports it, so there is no rate
  conversion; otherwise the hardware scaler fits the picture at 4:3
  (`--stretch` to fill). It uses a YUV420 plane, so no CPU colour conversion.
  Run on Raspberry Pi OS **Lite** (no desktop holding the display).
- **Web** (`-o web`): `/` is a fullscreen viewer with receiver status,
  `/stream.mjpg` is plain MJPEG (also works in VLC/ffplay),
  `/snapshot.jpg`, and `/status.json`. Encoding only runs while someone
  watches, and slow clients skip frames instead of building latency.
- **Y4M** (`-o y4m:PATH`) for testing.

### Useful options

```
--rx A                  one receiver only (8-bit SMI)
--div fuse|auto|select|blend|a|b    diversity mode (default fuse)
--no-reclock            skip code clock recovery
--std pal|ntsc          skip auto-detection
--deint weave           full vertical resolution, combing on motion (default bob = 50/60 fps)
--hq                    20-tap sinc resampler (more CPU)
--sat --contrast --brightness --hshift --voffset   picture adjustments
--smi-timing 1,3,1,0    SMI setup,strobe,hold,pace cycles (125 MHz → 25 MS/s)
```

## Status

The full pipeline has been tested **on a PC** against the synthetic C5 source:
PAL and NTSC lock, standard detection, colour accuracy (75% bars within 4
luma / 3 chroma codes), diversity selection and blending, fades, SMI-gap
recovery, and the web output.
Not yet verified **on hardware**:

- **Capture (verified on a Zero 2W, 2026-11-23):** `-s smi` (default) is a
  gapless capture. SMI read cycles run into a circular DMA chain on DMA
  channel 7, writing to a firmware-owned 8 MiB ring, and the rate measured
  24.997 MS/s with zero gaps over 30 s. The kernel `/dev/smi` read path is
  still available as `-s smidev`, but it loses 20–30 % of the signal between
  reads: the driver copies each buffer out of uncached memory at about
  110 MB/s while SMI is stopped.
- **HDMI output on vc4:** it compiles, but no display test has been run.
- **Live on a Zero 2W (2026-11-23, one C5VRX, NTSC):** 59.9 fields/s with
  `--workers 3` (active lines are decoded on worker threads), about 1.5
  cores. Decoding a recording runs at 1.4× realtime with 3 workers and 1.06×
  with 2. Two full decoders do not fit. The fuse mode
  needs about 1.6× one receiver on a PC and has not been measured on the Pi;
  nor has code clock recovery (2026-11-24, tested on recordings).
- **Real C5VRX signal levels, chroma bandwidth and field-order conventions**
  (`--voffset` / `--hshift` exist for alignment).

## Roadmap

1. Bring-up on hardware: `probe`, then record real captures for offline tuning.
3. NEON-tuned slicer and filters if the Zero 2W is short on CPU.
4. A 2-D comb filter to reduce the remaining cross-colour/cross-luminance.
5. Hardware H.264 (bcm2835 V4L2 M2M) for lower Wi-Fi bandwidth, plus an RTP/UDP output.

Licence: `firmware/` (c5div-rx) is GPL-3.0-only, as C5VRX. The Pi software
in `src/` is new code that reads the receivers' DAC pins; see the C5VRX credit
above.
