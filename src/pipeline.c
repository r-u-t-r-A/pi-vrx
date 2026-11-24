#include "pipeline.h"

#include <pthread.h>
#include <stdlib.h>

#include "reclock.h"

#include "common.h"

/* One receiver: clock recovery (optional) and a decoder, fed from the ring.
 * In fuse mode the decoder is sync-only and everything goes on to the fusion;
 * otherwise its fields go to the combiner. */
typedef struct {
    decoder_t *dec;
    fusion_t *fu;
    reclock_t *rc;
    ring_t *ring;
    int cid;
    diversity_t *dv;
    int rx;
    bool realtime;
    double fs;
    pthread_t th;
} consumer_t;

#define LATE_S 0.022

struct pipeline {
    pipeline_cfg_t cfg;
    div_mode_t mode;
    diversity_t *dv;
    decoder_t *rx[2];
    decoder_t *fused;
    fusion_t *fu;
    consumer_t c[2];
    int nc;
    pthread_t fth, dth;
    int started;            /* consumer threads running */
    bool fstarted;
};

static void *consumer_thread(void *arg)
{
    consumer_t *t = arg;
    const char *name = t->rx ? "rxB" : "rxA";
    thread_begin(name);
    for (;;) {
        const ring_block_t *b;
        bool ovr;
        ring_status_t st = ring_next(t->ring, t->cid, &b, &ovr, 200);
        if (st == RING_EOF)
            break;
        if (st == RING_TIMEOUT) {
            if (!atomic_load(&g_running) && t->realtime)
                break;
            continue;
        }
        const bool disc = (b->flags & RING_F_DISCONT) || ovr;
        const uint64_t start = b->start;
        /* Behind real time by more than 1.3 fields: leave out whole fields (lower
         * frame rate) instead of running late until the ring overruns. */
        const bool late = t->realtime && ring_backlog(t->ring, t->cid) * (double)b->n > LATE_S * t->fs;
        if (!t->fu)
            decoder_set_skip(t->dec, late);
        if (t->rc) {
            uint32_t m;
            uint64_t k0;
            const uint8_t *codes = reclock_feed(t->rc, b->data, b->n, start, disc, &m, &k0);
            ring_release(t->ring, t->cid);
            reclock_state_t rs;
            reclock_state(t->rc, &rs);
            /* Publish positions on the capture timeline both receivers share. */
            const double b0 = reclock_to_capture(&rs, 0.0);
            decoder_set_timebase(t->dec, (reclock_to_capture(&rs, 1e6) - b0) * 1e-6, b0);
            if (m)
                decoder_feed_u8(t->dec, codes, m, k0, disc);
            if (t->fu)
                fusion_push(t->fu, t->rx, t->dec, codes, m, k0, &rs, start, disc, late);
        } else {
            decoder_feed(t->dec, b->data, b->n, start, disc);
            ring_release(t->ring, t->cid);
        }
        if (!t->fu)
            diversity_poke(t->dv);
    }
    ring_remove_consumer(t->ring, t->cid);
    if (t->fu) {
        fusion_eof(t->fu, t->rx);
    } else {
        decoder_flush(t->dec);
        diversity_rx_eof(t->dv, t->rx);
    }
    thread_end(name);
    return NULL;
}

static void *fusion_thread(void *arg)
{
    pipeline_t *p = arg;
    thread_begin("fuse");
    while (fusion_step(p->fu))
        ;
    thread_end("fuse");
    return NULL;
}

static void *decode_thread(void *arg)
{
    pipeline_t *p = arg;
    thread_begin("decode");
    while (fusion_decode_step(p->fu))
        diversity_poke(p->dv);
    decoder_flush(p->fused);
    diversity_rx_eof(p->dv, 0);
    thread_end("decode");
    return NULL;
}

pipeline_t *pipeline_new(const pipeline_cfg_t *cfg, ring_t *ring, frame_pub_t *pub)
{
    pipeline_t *p = calloc(1, sizeof(*p));
    if (!p)
        return NULL;
    p->cfg = *cfg;
    div_cfg_t vc = cfg->div;
    const bool rcl = cfg->dec.reclock && reclock_supported(cfg->dec.fs_nom);
    if (cfg->nrx == 1 && vc.mode == DIV_FORCE_B)
        vc.mode = DIV_FORCE_A;
    if (vc.mode == DIV_FUSE && (cfg->nrx == 1 || !rcl)) {
        if (cfg->nrx == 2)
            LOGW("fuse mode needs clock recovery (25 MS/s capture): using auto");
        vc.mode = DIV_AUTO;
    }
    p->mode = vc.mode;
    const bool fuse = vc.mode == DIV_FUSE;
    p->dv = diversity_new(&vc, fuse ? 1 : cfg->nrx, pub);
    if (!p->dv)
        goto fail;

    decoder_cfg_t c = cfg->dec;
    if (rcl)
        c.fs_nom = 20e6;
    if (fuse) {
        decoder_cfg_t m = c;
        m.code_max = 255;
        m.median3 = false;   /* the fusion's click rejection does that job */
        p->fused = decoder_new(&m, 0, diversity_sink(p->dv));
        if (!p->fused || !(p->fu = fusion_new(p->fused)))
            goto fail;
        diversity_set_decoder(p->dv, 0, p->fused);
        c.workers = 0;
    }
    for (int r = 0; r < cfg->nrx; r++) {
        c.lane = r;
        const field_sink_t none = {0};
        if (!(p->rx[r] = decoder_new(&c, r, fuse ? none : diversity_sink(p->dv))))
            goto fail;
        consumer_t *t = &p->c[p->nc++];
        *t = (consumer_t){.dec = p->rx[r], .fu = p->fu, .ring = ring, .cid = ring_add_consumer(ring), .dv = p->dv,
                          .rx = r, .realtime = cfg->realtime, .fs = cfg->dec.fs_nom};
        if (rcl && !(t->rc = reclock_new(r)))
            goto fail;
        if (!fuse)
            diversity_set_decoder(p->dv, r, p->rx[r]);
    }
    return p;
fail:
    LOGE("pipeline: out of memory");
    pipeline_free(p);
    return NULL;
}

int pipeline_start(pipeline_t *p)
{
    if (diversity_start(p->dv))
        return -1;
    if (p->fu) {
        if (pthread_create(&p->fth, NULL, fusion_thread, p) || pthread_create(&p->dth, NULL, decode_thread, p))
            return -1;
        p->fstarted = true;
    }
    for (int k = 0; k < p->nc; k++) {
        if (pthread_create(&p->c[k].th, NULL, consumer_thread, &p->c[k]))
            return -1;
        p->started++;
    }
    return 0;
}

void pipeline_join(pipeline_t *p)
{
    for (int k = 0; k < p->started; k++)
        pthread_join(p->c[k].th, NULL);
    if (p->fstarted) {
        pthread_join(p->fth, NULL);
        pthread_join(p->dth, NULL);
    }
    if (p->started)
        diversity_join(p->dv);
    p->started = 0;
    p->fstarted = false;
}

void pipeline_free(pipeline_t *p)
{
    if (!p)
        return;
    for (int k = 0; k < p->nc; k++)
        reclock_free(p->c[k].rc);
    fusion_free(p->fu);
    decoder_free(p->fused);
    for (int r = 0; r < 2; r++)
        decoder_free(p->rx[r]);
    diversity_free(p->dv);
    free(p);
}

div_mode_t pipeline_mode(const pipeline_t *p) { return p->mode; }
decoder_t *pipeline_rx(const pipeline_t *p, int rx) { return p->rx[rx]; }
decoder_t *pipeline_fused(const pipeline_t *p) { return p->fused; }
fusion_t *pipeline_fusion(const pipeline_t *p) { return p->fu; }
diversity_t *pipeline_diversity(const pipeline_t *p) { return p->dv; }

float pipeline_share(const pipeline_t *p, int rx)
{
    if (p->fu)
        return atomic_load(&fusion_stats(p->fu)->share[rx]);
    return atomic_load(&diversity_stats(p->dv)->sel_frac[rx]);
}
