/* Diversity combiner.
 *
 * Both receivers are captured by the same SMI stream, so their fields share
 * one sample timeline and can be paired exactly by vsync position. Every
 * line was time-base corrected against its own receiver's sync, so lines
 * from A and B line up and can be switched or mixed line by line.
 *
 * Modes:
 *   select  per line, take the receiver with the better (vertically
 *           smoothed) SNR, with hysteresis
 *   blend   SNR-weighted average of both receivers
 *   auto    blend when both are within blend_db of each other, else select
 *   a / b   force one receiver
 */
#pragma once

#include <stdbool.h>

#include "decoder.h"
#include "field.h"
#include "frame.h"

/* DIV_FUSE is composite-level fusion (fusion.c) before a single decoder;
 * the combiner then only publishes that decoder's fields. */
typedef enum { DIV_AUTO = 0, DIV_SELECT, DIV_BLEND, DIV_FORCE_A, DIV_FORCE_B, DIV_FUSE } div_mode_t;

typedef struct {
    div_mode_t mode;
    float hyst_db;      /* select: switch when the other side is this much better */
    float blend_db;     /* auto: blend when |snrA - snrB| < blend_db */
    int smooth;         /* lines each side used to smooth quality */
    bool lossless;      /* wait instead of dropping (file/synth sources) */
    bool realtime;      /* publish "no signal" frames when idle */
} div_cfg_t;

typedef struct {
    atomic_uint_fast64_t fields_out, fields_paired, fields_solo, dropped;
    atomic_uint_fast64_t lines_from[3];   /* A, B, blend */
    _Atomic float snr_db[2];
    _Atomic float sel_frac[2];
} div_stats_t;

typedef struct diversity diversity_t;

void div_cfg_default(div_cfg_t *c);
diversity_t *diversity_new(const div_cfg_t *cfg, int nrx, frame_pub_t *pub);
void diversity_free(diversity_t *dv);
field_sink_t diversity_sink(diversity_t *dv);
void diversity_set_decoder(diversity_t *dv, int rx, decoder_t *d);
void diversity_poke(diversity_t *dv);          /* decoder made progress */
void diversity_rx_eof(diversity_t *dv, int rx);
int diversity_start(diversity_t *dv);
void diversity_join(diversity_t *dv);
div_stats_t *diversity_stats(diversity_t *dv);
