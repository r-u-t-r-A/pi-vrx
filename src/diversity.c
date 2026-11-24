#include "diversity.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define NPOOL 6

struct diversity {
    div_cfg_t cfg;
    int nrx;
    frame_pub_t *pub;
    decoder_t *dec[2];
    pthread_mutex_t mu;
    pthread_cond_t cv;      /* new field / progress / eof */
    pthread_cond_t cv_free; /* pool buffer released */
    field_t *pool[2][NPOOL];
    bool in_use[2][NPOOL];
    field_t *q[2][NPOOL];
    int qn[2];
    bool eof[2];
    pthread_t th;
    bool started;
    field_t *out;
    int sel;                /* current selection for select mode */
    double last_out_wall;
    video_std_t last_std;
    div_stats_t st;
};

void div_cfg_default(div_cfg_t *c)
{
    c->mode = DIV_AUTO;
    c->hyst_db = 1.5f;
    c->blend_db = 3.0f;
    c->smooth = 3;
    c->lossless = false;
    c->realtime = true;
}

static field_t *sink_acquire(void *ctx, int rx)
{
    diversity_t *dv = ctx;
    field_t *f = NULL;
    pthread_mutex_lock(&dv->mu);
    for (;;) {
        for (int i = 0; i < NPOOL; i++)
            if (!dv->in_use[rx][i]) {
                dv->in_use[rx][i] = true;
                f = dv->pool[rx][i];
                break;
            }
        if (f)
            break;
        if (dv->cfg.lossless) {
            pthread_cond_wait(&dv->cv_free, &dv->mu);
            continue;
        }
        /* Realtime: drop this receiver's oldest queued field. */
        if (dv->qn[rx] > 0) {
            f = dv->q[rx][0];
            memmove(&dv->q[rx][0], &dv->q[rx][1], sizeof(field_t *) * (size_t)(dv->qn[rx] - 1));
            dv->qn[rx]--;
            atomic_fetch_add(&dv->st.dropped, 1);
        }
        break;
    }
    pthread_mutex_unlock(&dv->mu);
    return f;
}

static void sink_submit(void *ctx, field_t *f)
{
    diversity_t *dv = ctx;
    pthread_mutex_lock(&dv->mu);
    int rx = f->rx;
    if (dv->qn[rx] < NPOOL)
        dv->q[rx][dv->qn[rx]++] = f;
    pthread_cond_broadcast(&dv->cv);
    pthread_mutex_unlock(&dv->mu);
}

static void release_field(diversity_t *dv, field_t *f)
{
    if (!f)
        return;
    for (int r = 0; r < 2; r++)
        for (int i = 0; i < NPOOL; i++)
            if (dv->pool[r][i] == f)
                dv->in_use[r][i] = false;
}

static float smoothed_q(const field_t *f, int row, int n)
{
    if (!f)
        return Q_INVALID;
    float s = 0;
    int c = 0;
    for (int r = row - n; r <= row + n; r++) {
        if (r < 0 || r >= f->lines)
            continue;
        if (f->q[r] > Q_INVALID) {
            s += f->q[r];
            c++;
        }
    }
    return c ? s / c : Q_INVALID;
}

static void copy_line(field_t *o, int r, const field_t *s)
{
    memcpy(o->y[r], s->y[r], OUT_W);
    memcpy(o->cb[r], s->cb[r], OUT_CW);
    memcpy(o->cr[r], s->cr[r], OUT_CW);
    o->q[r] = s->q[r];
}

static void blend_line(field_t *o, int r, const field_t *a, const field_t *b, float qa, float qb)
{
    /* Power-SNR weighting (maximum-ratio style). */
    float wa = powf(10.0f, qa / 10.0f), wb = powf(10.0f, qb / 10.0f);
    int ia = (int)lrintf(256.0f * wa / (wa + wb));
    int ib = 256 - ia;
    for (int i = 0; i < OUT_W; i++)
        o->y[r][i] = (uint8_t)((a->y[r][i] * ia + b->y[r][i] * ib + 128) >> 8);
    for (int i = 0; i < OUT_CW; i++) {
        o->cb[r][i] = (uint8_t)((a->cb[r][i] * ia + b->cb[r][i] * ib + 128) >> 8);
        o->cr[r][i] = (uint8_t)((a->cr[r][i] * ia + b->cr[r][i] * ib + 128) >> 8);
    }
    o->q[r] = 10.0f * log10f(wa + wb);
}

/* a = receiver A field, b = receiver B field; either may be NULL. */
static void combine(diversity_t *dv, const field_t *a, const field_t *b)
{
    field_t *o = dv->out;
    const field_t *ref = a ? a : b;
    if (a && b && (a->std != b->std || a->lines != b->lines)) {
        if (a->snr_db >= b->snr_db) b = NULL; else a = NULL;
        ref = a ? a : b;
    }
    o->start_pos = ref->start_pos;
    o->end_pos = ref->end_pos;
    o->t_wall = ref->t_wall;
    o->parity = ref->parity;
    o->std = ref->std;
    o->lines = ref->lines;
    o->rx = -1;
    uint64_t cnt[3] = {0, 0, 0};
    double qs = 0;
    int qn = 0;
    for (int r = 0; r < o->lines; r++) {
        float qa = a ? a->q[r] : Q_INVALID;
        float qb = b ? b->q[r] : Q_INVALID;
        float sa = qa > Q_INVALID ? smoothed_q(a, r, dv->cfg.smooth) : Q_INVALID;
        float sb = qb > Q_INVALID ? smoothed_q(b, r, dv->cfg.smooth) : Q_INVALID;
        int pick;   /* 0 A, 1 B, 2 blend, -1 none */
        if (sa <= Q_INVALID && sb <= Q_INVALID) pick = -1;
        else if (sb <= Q_INVALID) pick = 0;
        else if (sa <= Q_INVALID) pick = 1;
        else {
            switch (dv->cfg.mode) {
            case DIV_FORCE_A: pick = 0; break;
            case DIV_FORCE_B: pick = 1; break;
            case DIV_BLEND: pick = 2; break;
            case DIV_AUTO:
                if (fabsf(sa - sb) < dv->cfg.blend_db) { pick = 2; break; }
                /* fall through */
            case DIV_SELECT:
            default:
                if (dv->sel == 0 && sb > sa + dv->cfg.hyst_db) dv->sel = 1;
                else if (dv->sel == 1 && sa > sb + dv->cfg.hyst_db) dv->sel = 0;
                pick = dv->sel;
                break;
            }
        }
        if (dv->cfg.mode == DIV_FORCE_A && a) pick = qa > Q_INVALID ? 0 : -1;
        if (dv->cfg.mode == DIV_FORCE_B && b) pick = qb > Q_INVALID ? 1 : -1;
        if (pick == 0) copy_line(o, r, a);
        else if (pick == 1) copy_line(o, r, b);
        else if (pick == 2) blend_line(o, r, a, b, qa, qb);
        else o->q[r] = Q_INVALID;
        if (pick >= 0) {
            o->src[r] = (uint8_t)pick;
            cnt[pick]++;
            qs += o->q[r];
            qn++;
        }
    }
    o->valid_lines = qn;
    o->snr_db = qn ? (float)(qs / qn) : Q_INVALID;
    for (int i = 0; i < 3; i++)
        atomic_fetch_add(&dv->st.lines_from[i], cnt[i]);
    float tot = (float)(cnt[0] + cnt[1] + cnt[2]);
    if (tot > 0) {
        atomic_store(&dv->st.sel_frac[0], (cnt[0] + 0.5f * cnt[2]) / tot);
        atomic_store(&dv->st.sel_frac[1], (cnt[1] + 0.5f * cnt[2]) / tot);
    }
    if (a) atomic_store(&dv->st.snr_db[0], a->snr_db);
    if (b) atomic_store(&dv->st.snr_db[1], b->snr_db);
}

static void fill_osd(diversity_t *dv, osd_info_t *o)
{
    o->nrx = dv->nrx;
    for (int r = 0; r < 2; r++) {
        o->locked[r] = dv->dec[r] ? atomic_load(&decoder_stats(dv->dec[r])->locked) : false;
        o->snr_db[r] = dv->dec[r] ? atomic_load(&decoder_stats(dv->dec[r])->snr_db) : 0;
        o->sel_frac[r] = atomic_load(&dv->st.sel_frac[r]);
    }
}

enum { ACT_NONE, ACT_SOLO, ACT_PAIR, ACT_DONE };

static int decide(diversity_t *dv, field_t **fa, field_t **fb)
{
    *fa = *fb = NULL;
    field_t *h0 = dv->qn[0] ? dv->q[0][0] : NULL;
    field_t *h1 = dv->qn[1] ? dv->q[1][0] : NULL;
    if (!h0 && !h1) {
        bool done = true;
        for (int r = 0; r < dv->nrx; r++)
            done = done && dv->eof[r];
        return done ? ACT_DONE : ACT_NONE;
    }
    field_t *F = !h1 ? h0 : !h0 ? h1 : (h0->start_pos <= h1->start_pos ? h0 : h1);
    int r = F->rx, o = 1 - r;
    if (dv->nrx == 1) {
        if (r == 0) *fa = F; else *fb = F;
        return ACT_SOLO;
    }
    double flen = F->end_pos > F->start_pos ? (double)(F->end_pos - F->start_pos) : 1e6;
    for (int i = 0; i < dv->qn[o]; i++) {
        field_t *G = dv->q[o][i];
        if (fabs((double)G->start_pos - (double)F->start_pos) < 0.3 * flen) {
            *fa = r == 0 ? F : G;
            *fb = r == 0 ? G : F;
            return ACT_PAIR;
        }
    }
    bool other_gone = dv->eof[o] || !dv->dec[o] || !atomic_load(&decoder_stats(dv->dec[o])->locked);
    uint64_t prog = dv->dec[o] ? atomic_load(&decoder_stats(dv->dec[o])->progress) : 0;
    double line = flen / std_params(F->std)->lines_per_field;
    bool other_past = (double)prog > (double)F->end_pos + 8.0 * line;
    bool stale = !dv->cfg.lossless && now_s() - F->t_wall > 0.03;
    if (other_gone || other_past || stale) {
        if (r == 0) *fa = F; else *fb = F;
        return ACT_SOLO;
    }
    return ACT_NONE;
}

static void dequeue(diversity_t *dv, field_t *f)
{
    if (!f)
        return;
    int r = f->rx;
    for (int i = 0; i < dv->qn[r]; i++)
        if (dv->q[r][i] == f) {
            memmove(&dv->q[r][i], &dv->q[r][i + 1], sizeof(field_t *) * (size_t)(dv->qn[r] - i - 1));
            dv->qn[r]--;
            return;
        }
}

static void *combiner_thread(void *arg)
{
    diversity_t *dv = arg;
    thread_begin("combine");
    for (;;) {
        field_t *fa, *fb;
        int act;
        pthread_mutex_lock(&dv->mu);
        for (;;) {
            act = decide(dv, &fa, &fb);
            if (act != ACT_NONE)
                break;
            if (!atomic_load(&g_running) && !dv->cfg.lossless) {
                act = ACT_DONE;
                break;
            }
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_nsec += 10 * 1000000L;
            if (ts.tv_nsec >= 1000000000L) {
                ts.tv_sec++;
                ts.tv_nsec -= 1000000000L;
            }
            int rc = pthread_cond_timedwait(&dv->cv, &dv->mu, &ts);
            if (rc == ETIMEDOUT && dv->cfg.realtime && now_s() - dv->last_out_wall > 0.3) {
                pthread_mutex_unlock(&dv->mu);
                osd_info_t osd;
                fill_osd(dv, &osd);
                frame_pub_put_blank(dv->pub, dv->last_std ? dv->last_std : STD_PAL, &osd);
                dv->last_out_wall = now_s() - 0.1;   /* repeat at ~5 fps */
                pthread_mutex_lock(&dv->mu);
            }
        }
        if (act == ACT_DONE) {
            pthread_mutex_unlock(&dv->mu);
            break;
        }
        dequeue(dv, fa);
        dequeue(dv, fb);
        pthread_mutex_unlock(&dv->mu);

        combine(dv, fa, fb);
        atomic_fetch_add(&dv->st.fields_out, 1);
        atomic_fetch_add(act == ACT_PAIR ? &dv->st.fields_paired : &dv->st.fields_solo, 1);
        osd_info_t osd;
        fill_osd(dv, &osd);
        frame_pub_put_field(dv->pub, dv->out, &osd);
        dv->last_out_wall = now_s();
        dv->last_std = dv->out->std;

        pthread_mutex_lock(&dv->mu);
        release_field(dv, fa);
        release_field(dv, fb);
        pthread_cond_broadcast(&dv->cv_free);
        pthread_mutex_unlock(&dv->mu);
    }
    frame_pub_set_eof(dv->pub);
    thread_end("combine");
    return NULL;
}

diversity_t *diversity_new(const div_cfg_t *cfg, int nrx, frame_pub_t *pub)
{
    diversity_t *dv = calloc(1, sizeof(*dv));
    if (!dv)
        return NULL;
    dv->cfg = *cfg;
    dv->nrx = nrx;
    dv->pub = pub;
    pthread_mutex_init(&dv->mu, NULL);
    pthread_cond_init(&dv->cv, NULL);
    pthread_cond_init(&dv->cv_free, NULL);
    for (int r = 0; r < nrx; r++)
        for (int i = 0; i < NPOOL; i++)
            if (!(dv->pool[r][i] = calloc(1, sizeof(field_t))))
                return NULL;
    dv->out = calloc(1, sizeof(field_t));
    dv->last_out_wall = now_s();
    return dv->out ? dv : NULL;
}

void diversity_free(diversity_t *dv)
{
    if (!dv)
        return;
    for (int r = 0; r < 2; r++)
        for (int i = 0; i < NPOOL; i++)
            free(dv->pool[r][i]);
    free(dv->out);
    pthread_mutex_destroy(&dv->mu);
    pthread_cond_destroy(&dv->cv);
    pthread_cond_destroy(&dv->cv_free);
    free(dv);
}

field_sink_t diversity_sink(diversity_t *dv)
{
    field_sink_t s = {.acquire = sink_acquire, .submit = sink_submit, .ctx = dv};
    return s;
}

void diversity_set_decoder(diversity_t *dv, int rx, decoder_t *d) { dv->dec[rx] = d; }

void diversity_poke(diversity_t *dv)
{
    pthread_mutex_lock(&dv->mu);
    pthread_cond_broadcast(&dv->cv);
    pthread_mutex_unlock(&dv->mu);
}

void diversity_rx_eof(diversity_t *dv, int rx)
{
    pthread_mutex_lock(&dv->mu);
    dv->eof[rx] = true;
    pthread_cond_broadcast(&dv->cv);
    pthread_mutex_unlock(&dv->mu);
}

int diversity_start(diversity_t *dv)
{
    if (pthread_create(&dv->th, NULL, combiner_thread, dv))
        return -1;
    dv->started = true;
    return 0;
}

void diversity_join(diversity_t *dv)
{
    if (dv->started)
        pthread_join(dv->th, NULL);
    dv->started = false;
}

div_stats_t *diversity_stats(diversity_t *dv) { return &dv->st; }
