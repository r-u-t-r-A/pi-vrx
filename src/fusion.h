/* Composite-level diversity fusion of two receivers.
 *
 * One thread per receiver recovers that C5's code clock (reclock.c) and runs
 * a sync-only decoder on the codes; it pushes codes, timing and per-line
 * noise, block by block, into the fusion. The fusion thread combines them
 * into ONE 8-bit composite stream on receiver A's code timeline, which a
 * single full decoder decodes:
 *
 *   alignment  B's delay: whole lines by correlating the vertical interval,
 *              the rest from the two line PLLs; B is interpolated at A's
 *              code instants (the two C5 clocks differ by a few ppm)
 *   levels     each receiver normalised by its own blank and sync depth
 *   combining  per-line inverse-noise weights (maximum-ratio combining),
 *              interpolated along the line
 *   clicks     where A and B disagree by much more than their noise, the
 *              better receiver, or the sample that fits its neighbours, wins
 *   fallback   an unlocked receiver counts as very noisy; with both
 *              unlocked the last delay is kept and they are averaged
 */
#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#include "decoder.h"
#include "reclock.h"

typedef struct {
    _Atomic float share[2];       /* weight share of A and B, 0..1 */
    _Atomic float delay_us;       /* B relative to A */
    atomic_bool aligned;
    atomic_uint_fast64_t outliers[2];   /* samples where A / B was rejected */
    atomic_uint_fast64_t realigns;      /* alignment lost or changed */
} fusion_stats_t;

typedef struct fusion fusion_t;

fusion_t *fusion_new(decoder_t *main_dec);
void fusion_free(fusion_t *f);

/* Receiver thread, after feeding `dec` with these codes. Blocks while the
 * queue is full. */
void fusion_push(fusion_t *f, int lane, decoder_t *dec, const uint8_t *codes, uint32_t n, uint64_t k0,
                 const reclock_state_t *st, uint64_t cap_start, bool discont);
void fusion_eof(fusion_t *f, int lane);

/* Fusion thread: wait for and process one block; false at the end. */
bool fusion_step(fusion_t *f);

fusion_stats_t *fusion_stats(fusion_t *f);

/* Fused stream scale: headroom below the sync tip for noise (clipping there
 * would bias the levels), peak white at 210, 100% bars at 254. */
#define FUSED_BLANK 84.0f
#define FUSED_SYNC 54.0f
