# c5div: dual ESP32-C5 diversity video receiver for Raspberry Pi

Two [C5VRX](https://github.com/Twotoz/C5VRX) receivers (Seeed XIAO ESP32-C5
running the C5VRX firmware) feed a **Raspberry Pi Zero 2W**. The Pi
combines both composite streams in software (diversity), decodes the result,
and outputs the picture on **HDMI** and/or as a **web stream
over a Wi-Fi access point** that phones and laptops can view.

> **Highly experimental.** It runs on real hardware, but it can be unstable and
> things change often. See [Status](#status).

```
 VTX 5.8 GHz
   │                 6-bit DAC bus (digital, tapped before the resistors)
   ├─► XIAO C5 #A ──────────────┐
   └─► XIAO C5 #B ──────────┐   │   SMI 16-bit @ 25 MS/s (async)
                            ▼   ▼
                  Pi Zero 2W
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
sudo systemctl stop c5div; sudo c5div probe   # wiring and signal check (then: start c5div)
sudo systemctl stop c5div; sudo c5div run   # by hand: HDMI + web viewer
sudo ./pi/setup-ap.sh MYSSID MYPASSWORD # optional: Wi-Fi AP at 192.168.50.1
# c5div starts at boot; options in /etc/default/c5div
sudo c5div record --seconds 5 -O cap.raw   # raw capture, replay with -s file:cap.raw
```

### Outputs

- **HDMI** (`-o hdmi`): DRM/KMS atomic. It asks for 720×576@50 (PAL) or
  720×480@60 (NTSC) when the display supports it, so there is no rate
  conversion; otherwise the hardware scaler fits the picture at 4:3
  (`--stretch` to fill). It uses a YUV420 plane, so no CPU colour conversion.
  Run on Raspberry Pi OS **Lite** (no desktop holding the display).
- **Composite AV** (`-o av`): the Pi's analog TV output, 720 pixels per line
  passed through 1:1, PAL or NTSC like the camera (`--av-std`,
  `--av-progressive`). It replaces HDMI while it runs.
- **Jumpers** (`-o jumpers`, the service default): a jumper on GPIO4 picks AV
  instead of HDMI and one on GPIO5 turns the web viewer on, also while
  running. See [docs/WIRING.md](docs/WIRING.md#output-jumpers).
- **Web** (`-o web`, ports 80 and 8080): `/` is a fullscreen viewer with receiver status,
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

**It works on real hardware, but it is highly experimental and can be unstable.**
Expect rough edges, picture glitches and breaking changes. Do not rely on it as
your only video link.

Working on a Pi Zero 2W with two XIAO ESP32-C5 receivers (c5div-rx firmware) and
an NTSC camera:

- **Capture:** both receivers at 25 MS/s, gapless, recovering by itself if the DMA
  channel is stopped. (The kernel `/dev/smi` path, `-s smidev`, loses 20–30 % of the
  signal and is only a fallback.)
- **Picture:** live colour video. C5 code clock recovery removed the coloured streaks
  and the diagonal pattern in replays of a real recording.
- **Diversity:** live fusion of both receivers.
- **Web viewer** over the Pi's Wi-Fi access point.
- **Operation:** starts at boot as a service, about 22 s after power-on; the Wi-Fi
  jumper works.
- **Speed:** two-receiver recordings play at 59.9 fields/s in real time with the web
  viewer connected and no skipped fields. When the CPU is short, whole fields are
  skipped and the log says so. The Pi gets warm (72 °C seen): use a heatsink.

Not tested yet, or only in simulation:

- **HDMI and AV:** no display test on record; the code is tested against a fake libdrm
  (`test_outputs`). For AV on a Zero 2W it is open whether `enable_tvout` is needed
  and whether switching works without a reboot.
- **PAL:** synthetic source only.
- **Range and flight:** no test on record.

## Roadmap

1. HDMI and AV on real displays; PAL camera.
2. Comparison of the firmware's demodulators (D32, P64, U50P64, U85Q).
3. A 2-D comb filter to reduce the remaining cross-colour/cross-luminance.
4. Hardware H.264 (bcm2835 V4L2 M2M) for lower Wi-Fi bandwidth, plus an RTP/UDP output.

The firmware is a git submodule ([fork of C5VRX](https://github.com/r-u-t-r-A/C5VRX),
branch `c5div-lite`): `git submodule update --init` fetches it. The Pi does not need it.

Licence: `firmware/` (c5div-rx) is GPL-3.0-only, as C5VRX. The Pi software
in `src/` is new code that reads the receivers' DAC pins; see the C5VRX credit
above.
