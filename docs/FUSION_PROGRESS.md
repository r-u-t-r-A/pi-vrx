# Two-receiver diversity: status and next steps

Fusion runs live on a Zero 2W with both receivers wired (2026-11-24). The picture-error figures below come from the
synthetic source, the CPU figures from recordings played on the Pi.

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

## Flywheel (sync reconstruction)

`ST_FLY` in `src/decoder.c`. Before it, a sync loss of 50 lines dropped all timing; re-acquisition needed 8 clean lines in a row
plus a full vsync, which scraps rarely give, so the picture froze (partial fields keep old content for missing lines).

- Entered after 4 lines without sync, once the standard, levels and vertical timing are known. The phase and line period come
  from the last clean stretch (8 matched lines), the period as a long-term average: the PLL's own period moves with every
  correction (beta) and was off by ~230 ppm after short bursts, which put the sync 1-3 us off after an 8 ms dip.
- Every line is decoded; levels are held; the vertical flywheel keeps making fields. Snow forever with no signal (user's choice).
- Back to lock: 7 of 8 lines with a sync within 1 us of the prediction, or 7 of 8 with a sync within 0.4 us of one place
  elsewhere (moved timing: phase jump). Noise pulses do not pass that; a looser histogram vote gave ~1 false jump per 10 ms of
  noise. Steady syncs at another line period (other camera or standard) go back to full acquisition.
- On for all decoders, also the sync-only ones in fuse mode: their line logs keep coming, so the fusion weights line by line
  through dropouts (fuse, 20 % scraps per receiver: 28 % -> 36 % of rows in place). `locked` stays false while flying, so
  alignment only uses locked timing. The diversity combiner counts a flying decoder as present (it waits for its fields).
- `test_pipeline fly`: scraps 50/20/10 % (fields keep coming, rows in place ~ the signal share, no false re-syncs), 2 s outage,
  VTX restart with a new line phase and with a new field phase, NTSC lost and back as PAL (synth `synth_std_at`: the
  flywheel sees the new line period and relocks). `test_pipeline bench [case] [nofly]` has scraps cases.
- Not tested: live on the Pi.

### Flywheel: status (2026-11-28)

Done on the PC and tested, not committed yet. Still to do:
1. Commit (when the user asks).
2. On the Pi (only when the user says the VRX is free): deploy, record a weak-signal capture
   (`sudo systemctl stop c5div; sudo c5div record --seconds 20 -O weak.raw`), replay with and without `--no-flywheel`, watch
   live on AV next to the goggles, check CPU with `-v`.

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

CPU on the Zero 2W for 1.04 s of two-receiver video (2026-11-24, fuse, without capture and JPEG): 4.5 s before, 2.1 s after the
fixed-point/NEON work; real-time run with web viewer: 59.9 fps, no skipped fields. `-v` prints CPU per thread at the end, `top -H`
shows the thread names. When behind by more than 22 ms, whole fields are left out (status line: "CPU overload") instead of running
late until the capture ring overruns (that was the slow-motion/jump effect in the web viewer).

Tests: `test_pipeline` (all), `test_pipeline fuse`, `test_pipeline bench [case]`. ASan/UBSan/TSan clean (run in the ESP-IDF
container, the host has no sanitizer runtime).

## To do

1. **Pi, one receiver** (needs the lab): build in `~/c5div-src`, install, run live with clock recovery. Check the picture, fps and
   CPU. Check `pgrep c5div` first, restart the viewer after:
   `nohup /usr/local/bin/c5div run --rx A --workers 3 -o web,hdmi > /tmp/c5div.log &`
2. **Pi, fuse CPU**: `c5div run -s synth:pal ...` is too slow there to be useful as a source; copy a two-lane recording over
   (`-s file:...`) and time it. If too slow: NEON for `reclock_feed` / `interp_b`, fewer BTAPS, SR_TAPS 8 -> 6.
   AV output: tested on a real display, 300-400 ms latency from camera to screen. Still open: HDMI on a display, HDMI<->AV
   while running.
   The installed /etc/default/c5div is kept by install.sh: set `--output jumpers` there by hand.
3. **Second XIAO**: header pins 36, 11, 12, 35, 38, 40 (GPIO16-21). Check 16-bit gapless capture (50 MB/s), then live: A-B delay,
   outlier rate, one C5 restarting, one antenna shadowed. Both C5s must be on the same channel.
4. **Known gaps**: noise clipped at code 0 biases the sync depth (picture gain) at very low SNR, for every mode; both receivers in a
   dip at once is not repairable; the D32/P64/U50P64/U85Q decoder comparison is still open (worth redoing with clock recovery).

## Rules to keep

- Do not touch the C5 serial port (`/dev/ttyACM0`) without asking: opening it can reset and hang the C5.
- Only one c5div can capture (lock file). Stop the service before `record`/`probe`/a manual run, start it afterwards.
