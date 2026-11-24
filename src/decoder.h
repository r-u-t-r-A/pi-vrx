/* Per-receiver composite video decoder.
 *
 * Input: the 6-bit C5VRX DAC code stream of one receiver, sampled
 * asynchronously by the Pi (default 25 MS/s, C5 updates at 20 MS/s).
 * The C5 outputs FM-demodulated video with no regenerated sync, and its
 * levels follow VTX deviation and the C5's OUTPUT setting, so everything
 * here is adaptive:
 *
 *   slicer      box-filtered Schmitt trigger, sub-sample edge interpolation
 *   line PLL    tracks hsync, coasts through dropouts, snaps after jumps
 *   vertical    broad-pulse detection, field parity, vertical flywheel
 *   levels      sync tip / blanking measured per line (AGC + clamp)
 *   TBC         each line resampled to the BT.601 13.5 MHz grid (720 px)
 *   colour      burst-locked quadrature demodulation, PAL V-switch, ACC,
 *               colour killer, luma by chroma remodulation/subtraction
 *   quality     per-line SNR from porch noise (used by diversity)
 */
#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#include "field.h"
#include "video_std.h"

typedef struct {
    int lane;              /* 0 = SD0..5 (receiver A), 1 = SD8..13 (receiver B) */
    video_std_t std_force; /* STD_UNKNOWN = auto-detect */
    double fs_nom;         /* nominal capture rate, Hz */
    bool mono;
    bool chroma_vavg;      /* average chroma of adjacent lines (less colour noise) */
    bool median3;          /* 3-tap median on input (kills single-sample glitches) */
    bool hq_resample;      /* 20-tap windowed-sinc resampler instead of prefilter + linear */
    float saturation;
    float contrast;
    float brightness;      /* added to Y (8-bit units) */
    float hshift_us;       /* horizontal picture shift */
    int voffset;           /* vertical picture shift, lines */
    float pll_alpha;       /* line phase correction gain (0..1) */
    float pll_beta;        /* line period correction gain */
    int workers;           /* threads decoding active lines (0 = inline) */
    bool reclock;          /* recover the C5's code clock first (25 MS/s captures) */
    int code_max;          /* input code range: 63 raw C5 codes (default), 255 fused stream */
} decoder_cfg_t;

void decoder_cfg_default(decoder_cfg_t *c);

typedef struct {
    atomic_bool locked;
    atomic_int std;
    atomic_uint_fast64_t progress;  /* virtual sample index fully processed */
    _Atomic float snr_db;           /* smoothed line quality */
    _Atomic float line_us;          /* measured line period */
    _Atomic float fs_est;           /* capture rate estimated from line period */
    _Atomic float sync_amp;         /* sync depth, DAC codes */
    _Atomic float blank;            /* blanking level, DAC codes */
    _Atomic float burst;            /* burst amplitude relative to nominal */
    atomic_uint_fast64_t fields, lines, coasted, resyncs, lock_losses, vsync_flywheel, gaps;
    atomic_uint_fast64_t pulses_hsync, pulses_eq, pulses_broad, pulses_other, vsyncs, vsync_rejected;
} decoder_stats_t;

typedef struct decoder decoder_t;

/* Line/field timing and levels, for another thread (fusion). Positions
 * are absolute sample indices of the capture stream. */
typedef struct {
    bool locked, vlock;
    double H;               /* samples per line */
    double line_pos;        /* start of a recent line */
    double last_vs;         /* start of the last vertical sync */
    double lines_per_field;
    float blank, amp;       /* blanking level and sync depth, input codes */
    float snr_db;           /* smoothed line quality */
} decoder_timing_t;

/* Per-line quality log. */
typedef struct {
    double pos;             /* line start, sample index */
    float nv;               /* noise power / sync depth^2, measured on the porches */
} decoder_line_t;
#define DECODER_LINES 128

decoder_t *decoder_new(const decoder_cfg_t *cfg, int rx_id, field_sink_t sink);
void decoder_free(decoder_t *d);

/* Feed one block of raw 16-bit samples (lane extracted internally).
 * start = virtual index of s[0]; discont = samples before were lost. */
void decoder_feed(decoder_t *d, const uint16_t *s, uint32_t n, uint64_t start, bool discont);
/* Same for a stream of 8-bit samples (no lane extraction; the fused stream). */
void decoder_feed_u8(decoder_t *d, const uint8_t *s, uint32_t n, uint64_t start, bool discont);
/* The next three are for the thread that feeds the decoder. */
void decoder_get_timing(const decoder_t *d, decoder_timing_t *t);
/* Lines processed since the last call (oldest first); returns the count. */
int decoder_take_lines(decoder_t *d, decoder_line_t out[DECODER_LINES]);
/* Published positions (field start/end, progress) = scale * index + offset:
 * puts decoders on different sample clocks on one timeline. */
void decoder_set_timebase(decoder_t *d, double scale, double offset);
/* End of stream: deliver the field in progress. */
void decoder_flush(decoder_t *d);

decoder_stats_t *decoder_stats(decoder_t *d);
