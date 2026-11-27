# Wiring: two XIAO ESP32-C5 (C5VRX) → Raspberry Pi Zero 2W

The Pi reads each C5's 6-bit video DAC bus **digitally**, tapped *before* the
DAC resistors, with the Pi's Secondary Memory Interface (SMI) at 25 MS/s.
Both receivers share one 16-bit SMI sample: A on the low byte, B on the
high byte.

## Receiver firmware (both receivers)

**Recommended: c5div-rx** (`firmware/`, a reduced C5VRX-5 fork). Bit 2 is
already on D3, there is no menu, and the BOOT button cycles raceband R1–R8:

```
cd firmware && tools/build-container.sh
# hold BOOT while plugging in USB, then:
esptool.py --chip esp32c5 -p /dev/ttyACM0 write_flash 0x0 build/c5div_rx_merged.bin
```

Do the same for the second XIAO and wire the [channel link](#channel-link). The
BOOT button of the master changes the channel of both: a press moves both to the next
channel, a 2 s hold puts both on R1. See `firmware/README.md` for the one-time
calibration (8 s hold of BOOT, on each XIAO).

**Alternative: stock C5VRX** (web flasher on the C5VRX website):

1. Menu → **EXPERIMENTAL → Clock Pin D10 → ON**, then power off.
   This moves DAC **bit 2 from D6/GPIO11 to D3/GPIO7**: GPIO11 has a ~25 ns
   rising-edge delay, which would corrupt bit 2 at every code change. c5div
   does not use the D10 clock itself.
2. Put both receivers on the **same channel** as the VTX.
3. Leave AUTO OUTPUT off (the default).

## Pin table

A **220–470 Ω series resistor** in each data line is recommended, though not
strictly required. SMI may drive its data pins between transfers (reported
for iosoft's register-level code, unverified for `/dev/smi`); the resistor
limits any contention to a harmless few mA. On a dedicated receiver with SPI
and I2S off, direct wiring works but has no margin.

| DAC bit | XIAO pin (c5div-rx / Clock Pin D10) | Receiver A → Pi | Receiver B → Pi |
|---|---|---|---|
| bit 0 (LSB) | D4 (GPIO23) | GPIO8 (pin 24) | GPIO16 (pin 36) |
| bit 1 | D5 (GPIO24) | GPIO9 (pin 21) | GPIO17 (pin 11) |
| bit 2 | **D3 (GPIO7)** | GPIO10 (pin 19) | GPIO18 (pin 12) |
| bit 3 | D7 (GPIO12) | GPIO11 (pin 23) | GPIO19 (pin 35) |
| bit 4 | D8 (GPIO8) | GPIO12 (pin 32) | GPIO20 (pin 38) |
| bit 5 (MSB) | D9 (GPIO9) | GPIO13 (pin 33) | GPIO21 (pin 40) |
| GND | GND | GND (pin 6, 9, 14, 20, 25, 30, 34, 39) | same |
| power | 5V pin | Pi 5V (pin 2 or 4) | same |

Notes:

- Pin numbers are the Pi's 40-pin header.
- Keep the wires short (under ~10 cm) and give each C5 its own ground wire.
- Powering both C5s from the Pi's 5 V pin is fine with a good 5 V / 2.5 A
  supply into the Pi.
- The resistor DAC is optional. Keep it on one receiver if you also want an
  analog output, e.g. for goggles, at the same time.
- **One receiver only:** wire receiver A and run with `--rx A` (8-bit SMI).

## Channel link

Both receivers must be on the same channel. You change the channel with the BOOT
button of the **master** (either one); it tells the slave and the Pi which channel it is on.

| From | To |
|---|---|
| master XIAO **D6** | slave XIAO **D10** |
| master XIAO **D6** (same wire) | Pi **GPIO15 / RXD, pin 10** |
| GND | common (already there for the data bus) |

Nothing else goes on D10: it is not a button. Both
XIAOs run the same firmware (c5div-rx with channel link); the one that hears the other
becomes the slave. Details: `firmware/README.md`.

The Pi only listens. The web viewer shows `Channel: R3`, or `Channel: --` when nothing
was heard for 12 s; it is also in `status.json` and the log. `pi/config-txt.sh` (run by
`install.sh`) sets the UART up: Bluetooth off, so that GPIO15 gets the full UART, and no
login console on it. Another port: `--channel-uart /dev/ttyUSB0`; off: `--channel-uart none`.

## Output jumpers

Two jumpers choose the output. They work while c5div runs with `-o jumpers`
(the service default), with half a second of debounce.

| Jumper | Pins | Open | Fitted |
|---|---|---|---|
| AV | GPIO4 (pin 7) to GND (pin 9) | HDMI | composite AV |
| Wi-Fi | GPIO5 (pin 29) to GND (pin 30) | off | web viewer on |

HDMI and AV share the Pi's pixel pipeline, so only one runs at a time; the
web viewer runs next to either. The Wi-Fi jumper starts and stops the viewer,
not the access point (`pi/setup-ap.sh`). If you use pin 9 or 30 as a ground for
a receiver, move that ground to another pin.

The AV jumper is also read at power-on (`[gpio4=0]` in `config.txt` sets
`enable_tvout=1`, which the Raspberry Pi docs ask for on Zero boards). If
switching while running gives no AV picture, reboot with the jumper fitted.

## Composite AV connection

- **Zero 2W:** test pads on the underside
  ([drawing](https://files.waveshare.com/upload/1/11/Raspberry-pi-zero-2-w-test-pads.pdf)).
  Underside facing you, 40-pin header at the top: both pads are along the
  bottom edge, 3 mm up from it. **TV** is 45.6 mm from the left board edge and its
  **GND** is 3.8 mm further right. Solder a short shielded wire (TV to the
  core, GND to the shield) and fix it with glue: the pads tear off easily.
- The output is 1 Vpp into 75 Ω, no extra parts needed.

Tested on a real display: it works, with 300–400 ms latency from camera to screen.

The standard follows the camera (576i PAL / 480i NTSC). `--av-std pal|ntsc` forces
one, `--av-progressive` outputs 288p/240p for displays that flicker on interlaced video.

## Antennas for diversity

Diversity helps most when the two receivers fade at different times:

- different antenna types/orientations, e.g. an omni plus a patch, or two
  patches pointing different ways, or
- spatial separation of at least several centimetres (λ at 5.8 GHz ≈ 5 cm;
  more is better).

## Pi setup

```
git clone https://github.com/r-u-t-r-A/pi-vrx && cd pi-vrx
sudo ./pi/install.sh           # build, overlay, config.txt, service
sudo reboot
ls -l /dev/smi                 # must exist
sudo systemctl stop c5div; sudo c5div probe   # with both C5s running
```

`install.sh` adds this to `/boot/firmware/config.txt` (and makes a backup):

```
dtoverlay=vc4-kms-v3d,composite   # (changes the existing line) AV connector next to HDMI
dtoverlay=c5div-smi    # SMI on GPIO 8-13 and 16-21 only
dtoverlay=smi-dev      # /dev/smi character device
dtparam=spi=off        # GPIO8-11 are SPI0 otherwise
dtparam=i2s=off        # GPIO18-21 are I2S otherwise
gpio=4,5=ip,pu         # output jumpers
[gpio4=0]              # AV jumper fitted at power-on
enable_tvout=1
[all]
```

If `/dev/smi` is missing after the reboot, check `dmesg | grep -i smi` and
`modinfo bcm2835_smi_dev`. The module ships with Raspberry Pi OS kernels,
but if your kernel lacks `CONFIG_BCM2835_SMI_DEV` you will have to build it.

## Reading `c5div probe`

Example from the synthetic source (`c5div probe -s synth:pal`). Real
hardware output has the same layout:

```
receiver A (Pi GPIO 8-13)
  codes: min 0  p1 1  median 32  p99 61  max 63
  bit toggles / s: b0 7.4M b1 7.1M b2 5.6M b3 3.6M b4 3.6M b5 2M
  sample-order check: 1.00 (ok)
  decoder: LOCKED, PAL, line 64.000 us (capture rate est. 25.0001 MS/s)
           sync depth 18.0 codes, blank 20.0, burst 0.58 x nominal, line SNR 26.9 dB
           fields 47, lines 15634, coasted 0, resyncs 0, lock losses 0
```

- **A bit that never toggles** is a broken or missing wire. The probe names the C5
  pin and Pi header pin.
- **The measured rate is not about 25 MS/s:** the SMI clock is not the 125 MHz
  assumed. Pass the measured value with `--fs`, or adjust `--smi-timing`.
  The decoder's "capture rate est." comes from the video line period and is
  the most accurate number.
- **Sample-order check above 1.4:** the two 16-bit samples in each 32-bit
  word arrive swapped. Add `--smi-swap`.
- **The bits toggle but there is no lock:** check bit order (D4 = LSB), ground,
  and the VTX channel. With no VTX, the C5 outputs idle/noise.
- **The burst is about 0.55× nominal**, which is expected: the C5's 20 MS/s hold and
  the anti-alias prefilter attenuate the subcarrier, and ACC corrects it.
