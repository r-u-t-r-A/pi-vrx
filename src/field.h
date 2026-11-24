/* One decoded video field (one receiver or the diversity output). */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common.h"
#include "video_std.h"

#define Q_INVALID (-99.0f)

typedef struct field {
    uint64_t start_pos;   /* virtual sample index of the vsync that opened the field */
    uint64_t end_pos;     /* virtual sample index of the next vsync */
    double t_wall;        /* wall time the field completed */
    int parity;           /* 0: vsync at line start (top field), 1: half-line offset */
    video_std_t std;
    int lines;            /* active lines (288 PAL / 240 NTSC) */
    int rx;               /* producing receiver, or -1 for combined */
    bool synthetic_vsync; /* vertical flywheel generated the boundary */
    float snr_db;         /* mean line quality of valid lines */
    int valid_lines;
    uint32_t seq;
    uint8_t y[MAX_FIELD_LINES][OUT_W];
    uint8_t cb[MAX_FIELD_LINES][OUT_CW];
    uint8_t cr[MAX_FIELD_LINES][OUT_CW];
    float q[MAX_FIELD_LINES];       /* per-line quality (dB), Q_INVALID if missing */
    uint8_t src[MAX_FIELD_LINES];   /* combined fields: 0=A, 1=B, 2=blend */
} field_t;

/* Where a decoder gets empty fields and delivers finished ones. */
typedef struct {
    field_t *(*acquire)(void *ctx, int rx);
    void (*submit)(void *ctx, field_t *f);
    void *ctx;
} field_sink_t;
