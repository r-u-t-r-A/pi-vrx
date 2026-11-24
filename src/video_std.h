/* Analog video standard parameters (PAL-B/G/I 625/50, NTSC-M 525/59.94). */
#pragma once

typedef enum { STD_UNKNOWN = 0, STD_PAL = 1, STD_NTSC = 2 } video_std_t;

typedef struct {
    video_std_t std;
    const char *name;
    double line_us;          /* line period */
    double fsc_hz;           /* colour subcarrier */
    double lines_per_field;  /* 312.5 / 262.5 */
    double sync_ratio;       /* white level / sync depth (above blank / below blank) */
    double burst_nom;        /* burst peak amplitude in luma units (white = 1) */
    double h_active_us;      /* first output pixel after 0H (BT.601 grid) */
    double burst_start_us, burst_end_us;   /* window used to measure burst */
    double fp_start_us, fp_end_us;         /* front porch window (negative = before 0H) */
    double bp_start_us, bp_end_us;         /* back porch window after burst */
    double analog_start_us, analog_end_us; /* analog active video */
    int active_lines;        /* output lines per field */
    int first_active_k;      /* field line index (from first line start at/after vsync) */
    int frame_h;             /* output frame height */
    int field_rate_num, field_rate_den;
    int pal;                 /* 1 = PAL V-switch */
} std_params_t;

const std_params_t *std_params(video_std_t s);
const char *std_name(video_std_t s);
