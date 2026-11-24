# Two-receiver diversity: status and next steps

Updated 2026-11-24. Everything here was tested **on the PC only** (synthetic source and recordings of the
real receiver A). Nothing from 2026-11-24 has run on the Pi, and receiver B is not wired.

## How it works

```
ring -> reclock A -> sync-only decoder A -> fusion_push \
ring -> reclock B -> sync-only decoder B -> fusion_push  > fusion thread -> one full decoder -> diversity (passthrough) -> frames
```

- `src/reclock.c`: C5 code clock recovery (25 MS/s samples -> one sample per 20 MS/s code). On by default at
  25 MS/s; `--no-reclock` disables it. Pull-in verified to +-40 ppm (the real difference is ~6 ppm); it fails at 80.
- `src/fusion.c`: alignment, per-line weights, click rejection. See the comment in `fusion.h`.
- `src/pipeline.c`: threads and wiring, used by the app and the tests.
- Blocks carry codes, timing and the per-line noise log together, so results do not depend on thread timing.

## Findings

- **The streaks were ours.** The quasi-periodic coloured streaks in the live picture came from sampling on changing bits each time
  the Pi and C5 clocks slip (every ~1.7 ms). The earlier theory (C5 gain/servo writes) was wrong. `--no-reclock` brings them back.
- cap_fix.raw (real, one receiver), frame-to-frame noise: luma 12.2 -> 9.0, Cb 12.0 -> 9.1 with clock recovery (part of it is
  the better resampler).
- Earlier figures in this file ("+2.8 dB at equal noise") were wrong: the synth gives B a shallower sync (16 codes, A has 18).
- The porch SNR does not predict picture error. Use `test_pipeline bench`.
- The delay must be measured by comparing the same line on both receivers. Adding up N line lengths is off by N x the clock
  difference, which rotates the colour.

## Picture error (`test_pipeline bench`: luma noise / chroma RMS in the bars; one clock for both C5s)

| case | A only | auto | fuse |
|---|---|---|---|
| clean (before clock recovery: 9.7) | 2.0 / 1.5 | 1.6 / 1.4 | 1.6 / 1.4 |
| noise 4/4 | 11.6 / 5.0 | 8.7 / 3.9 | 8.7 / 3.8 |
| noise 8/8 | 23.8 / 11.5 | 17.9 / 9.0 | 18.1 / 7.6 |
| noise 4/8 | 11.6 / 5.0 | 11.6 / 5.0 | 10.7 / 4.6 |
| clicks 8/line | 7.3 / 3.2 | 6.4 / 2.9 | 3.8 / 2.0 |
| 2 ms dips | 21.3 / 17.7 | 5.4 / 6.5 | 5.5 / 3.8 |

CPU on the PC for 1.04 s of real video: one receiver 0.26 s, fuse 0.39-0.45 s, auto 0.60 s. Not measured on the Pi.

Tests: `test_pipeline` (all), `test_pipeline fuse`, `test_pipeline bench [case]`. ASan/UBSan/TSan clean (run in the ESP-IDF
container, the host has no sanitizer runtime).

## To do

1. **Pi, one receiver** (needs the lab): build in `~/c5div-src`, install, run live with clock recovery. Check the picture, fps and
   CPU. Check `pgrep c5div` first, restart the viewer after:
   `nohup /usr/local/bin/c5div run --rx A --workers 3 -o web,hdmi > /tmp/c5div.log &`
2. **Pi, fuse CPU**: `c5div run -s synth:pal ...` is too slow there to be useful as a source; copy a two-lane recording over
   (`-s file:...`) and time it. If too slow: NEON for `reclock_feed` / `interp_b`, fewer BTAPS, SR_TAPS 8 -> 6.
   Also deploy the AV output and jumpers (docs/WIRING.md): run `pi/config-txt.sh`, reboot, then check: AV picture with the jumper
   fitted at boot; HDMI<->AV while running; `[gpio4=0]` / `enable_tvout` behaviour on the Zero 2W (docs and forums disagree).
   The installed /etc/default/c5div is kept by install.sh: set `--output jumpers` there by hand.
3. **Second XIAO**: header pins 36, 11, 12, 35, 38, 40 (GPIO16-21). Check 16-bit gapless capture (50 MB/s), then live: A-B delay,
   outlier rate, one C5 restarting, one antenna shadowed. Both C5s must be on the same channel.
4. **Known gaps**: noise clipped at code 0 biases the sync depth (picture gain) at very low SNR, for every mode; both receivers in a
   dip at once is not repairable; the D32/P64/U50P64/U85Q decoder comparison is still open (worth redoing with clock recovery).

## Rules to keep

- Do not touch the C5 serial port (`/dev/ttyACM0`) without asking: opening it can reset and hang the C5.
- `c5div record` and `probe` on the Pi kill a running c5div's capture (shared DMA channel).
