/* Decoding pipeline between the capture ring and the frame publisher:
 * per-receiver decoders, optional composite fusion, the diversity combiner,
 * and their threads. Shared by the app and the tests.
 *
 *   fuse mode (2 receivers):  ring -> sync-only decoder A  \
 *                             ring -> sync-only decoder B   > timing
 *                             ring -> fusion -> main decoder -> combiner -> frames
 *   other modes:              ring -> decoder A/B -> combiner (per-line choice) -> frames
 */
#pragma once

#include <stdbool.h>

#include "decoder.h"
#include "diversity.h"
#include "frame.h"
#include "fusion.h"
#include "ring.h"

typedef struct {
    decoder_cfg_t dec;
    div_cfg_t div;      /* div.mode DIV_FUSE needs nrx == 2 (falls back to auto) */
    int nrx;
    bool realtime;      /* threads also stop when g_running clears */
} pipeline_cfg_t;

typedef struct pipeline pipeline_t;

pipeline_t *pipeline_new(const pipeline_cfg_t *cfg, ring_t *ring, frame_pub_t *pub);
int pipeline_start(pipeline_t *p);
void pipeline_join(pipeline_t *p);     /* after ring_set_eof */
void pipeline_free(pipeline_t *p);

div_mode_t pipeline_mode(const pipeline_t *p);
decoder_t *pipeline_rx(const pipeline_t *p, int rx);   /* per-receiver decoder (sync-only when fusing) */
decoder_t *pipeline_fused(const pipeline_t *p);        /* decoder of the fused stream, or NULL */
fusion_t *pipeline_fusion(const pipeline_t *p);        /* or NULL */
diversity_t *pipeline_diversity(const pipeline_t *p);
/* Weight / selection share of a receiver, 0..1. */
float pipeline_share(const pipeline_t *p, int rx);
