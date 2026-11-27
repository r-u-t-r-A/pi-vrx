#include "frame.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

int frame_alloc(frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->w = OUT_W;
    f->h = 2 * MAX_FIELD_LINES;
    f->y = malloc((size_t)OUT_W * 2 * MAX_FIELD_LINES);
    f->u = malloc((size_t)OUT_CW * MAX_FIELD_LINES);
    f->v = malloc((size_t)OUT_CW * MAX_FIELD_LINES);
    if (!f->y || !f->u || !f->v)
        return -1;
    memset(f->y, 16, (size_t)OUT_W * 2 * MAX_FIELD_LINES);
    memset(f->u, 128, (size_t)OUT_CW * MAX_FIELD_LINES);
    memset(f->v, 128, (size_t)OUT_CW * MAX_FIELD_LINES);
    return 0;
}

void frame_release(frame_t *f)
{
    free(f->y);
    free(f->u);
    free(f->v);
    f->y = f->u = f->v = NULL;
}

void frame_copy(frame_t *dst, const frame_t *src)
{
    dst->w = src->w;
    dst->h = src->h;
    dst->std = src->std;
    dst->seq = src->seq;
    dst->t_wall = src->t_wall;
    dst->no_signal = src->no_signal;
    memcpy(dst->y, src->y, (size_t)src->w * src->h);
    memcpy(dst->u, src->u, (size_t)(src->w / 2) * (src->h / 2));
    memcpy(dst->v, src->v, (size_t)(src->w / 2) * (src->h / 2));
}

int frame_pub_init(frame_pub_t *p, deint_t deint, bool osd)
{
    memset(p, 0, sizeof(*p));
    pthread_mutex_init(&p->mu, NULL);
    pthread_cond_init(&p->cv, NULL);
    p->deint = deint;
    p->osd = osd;
    p->prev = calloc(1, sizeof(field_t));
    if (!p->prev)
        return -1;
    if (frame_alloc(&p->fr))
        return -1;
    p->fr.h = 576;
    return 0;
}

void frame_pub_destroy(frame_pub_t *p)
{
    frame_release(&p->fr);
    free(p->prev);
    pthread_mutex_destroy(&p->mu);
    pthread_cond_destroy(&p->cv);
}

void frame_pub_add_listener(frame_pub_t *p, frame_listener_fn fn, void *ctx)
{
    pthread_mutex_lock(&p->mu);
    if (p->nlisteners < FRAME_MAX_LISTENERS) {
        p->listeners[p->nlisteners] = fn;
        p->listener_ctx[p->nlisteners] = ctx;
        p->nlisteners++;
    }
    pthread_mutex_unlock(&p->mu);
}

static inline void avg_row(uint8_t *dst, const uint8_t *a, const uint8_t *b, int n)
{
    for (int i = 0; i < n; i++)
        dst[i] = (uint8_t)((a[i] + b[i] + 1) >> 1);
}

static inline bool line_ok(const field_t *f, int l) { return l >= 0 && l < f->lines && f->q[l] > Q_INVALID; }

/* Missing lines (sync dropouts, capture gaps, partial fields) keep what the
 * frame showed before: temporal fill instead of smearing a neighbour. */
static void build_bob(frame_t *fr, const field_t *f)
{
    const int L = f->lines;
    if (fr->h != 2 * L || fr->std != f->std) {
        memset(fr->y, 16, (size_t)OUT_W * 2 * L);
        memset(fr->u, 128, (size_t)OUT_CW * L);
        memset(fr->v, 128, (size_t)OUT_CW * L);
    }
    fr->w = OUT_W;
    fr->h = 2 * L;
    fr->std = f->std;
    const bool top = f->parity == 0;
    for (int l = 0; l < L; l++) {
        if (!line_ok(f, l))
            continue;
        const uint8_t *cur = f->y[l];
        uint8_t *r0 = fr->y + (size_t)(2 * l) * OUT_W;
        uint8_t *r1 = r0 + OUT_W;
        if (top) {
            memcpy(r0, cur, OUT_W);
            if (line_ok(f, l + 1))
                avg_row(r1, cur, f->y[l + 1], OUT_W);
            else
                memcpy(r1, cur, OUT_W);
        } else {
            if (line_ok(f, l - 1))
                avg_row(r0, f->y[l - 1], cur, OUT_W);
            else
                memcpy(r0, cur, OUT_W);
            memcpy(r1, cur, OUT_W);
        }
        memcpy(fr->u + (size_t)l * OUT_CW, f->cb[l], OUT_CW);
        memcpy(fr->v + (size_t)l * OUT_CW, f->cr[l], OUT_CW);
    }
}

static void build_weave(frame_t *fr, const field_t *cur, const field_t *prev)
{
    const int L = cur->lines;
    if (fr->h != 2 * L || fr->std != cur->std) {
        build_bob(fr, cur);
        return;
    }
    const field_t *topf = cur->parity == 0 ? cur : prev;
    const field_t *botf = cur->parity == 0 ? prev : cur;
    for (int l = 0; l < L; l++) {
        if (line_ok(topf, l))
            memcpy(fr->y + (size_t)(2 * l) * OUT_W, topf->y[l], OUT_W);
        if (line_ok(botf, l))
            memcpy(fr->y + (size_t)(2 * l + 1) * OUT_W, botf->y[l], OUT_W);
        if (line_ok(cur, l)) {
            memcpy(fr->u + (size_t)l * OUT_CW, cur->cb[l], OUT_CW);
            memcpy(fr->v + (size_t)l * OUT_CW, cur->cr[l], OUT_CW);
        }
    }
}

/* Two signal bars, top-left: length = SNR, bright/green = receiver in use,
 * orange = flywheel (no sync, timing held), red = no lock. */
static void draw_osd(frame_t *fr, const osd_info_t *o)
{
    if (!o)
        return;
    const int bw = 160, bh = 10, x0 = 24, y0 = 24;
    for (int r = 0; r < o->nrx && r < 2; r++) {
        int yb = y0 + r * (bh + 6);
        float frac = o->locked[r] || o->fly[r] ? clampf(o->snr_db[r] / 36.0f, 0.0f, 1.0f) : 0.0f;
        int fill = (int)(frac * bw);
        bool active = o->sel_frac[r] >= 0.5f;
        for (int yy = yb; yy < yb + bh && yy < fr->h; yy++) {
            uint8_t *row = fr->y + (size_t)yy * fr->w;
            for (int xx = x0; xx < x0 + bw; xx++)
                row[xx] = xx - x0 < fill ? (active ? 220 : 140) : 40;
        }
        for (int yy = yb / 2; yy < (yb + bh) / 2; yy++) {
            uint8_t *u = fr->u + (size_t)yy * (fr->w / 2);
            uint8_t *v = fr->v + (size_t)yy * (fr->w / 2);
            for (int xx = x0 / 2; xx < (x0 + bw) / 2; xx++) {
                bool in = xx * 2 - x0 < fill;
                if (o->fly[r] && !o->locked[r]) {
                    u[xx] = 70; v[xx] = 190;             /* orange: flywheel */
                } else if (!o->locked[r]) {
                    u[xx] = 110; v[xx] = 200;            /* red: no lock */
                } else if (in && active) {
                    u[xx] = 70; v[xx] = 70;              /* green: in use */
                } else {
                    u[xx] = 128; v[xx] = 128;
                }
            }
        }
    }
}

static void notify(frame_pub_t *p)
{
    p->fr.seq = ++p->seq;
    p->fr.t_wall = now_s();
    for (int i = 0; i < p->nlisteners; i++)
        p->listeners[i](p->listener_ctx[i], &p->fr);
    pthread_cond_broadcast(&p->cv);
}

void frame_pub_put_field(frame_pub_t *p, const field_t *f, const osd_info_t *osd)
{
    pthread_mutex_lock(&p->mu);
    if (p->deint == DEINT_WEAVE && p->have_prev && p->prev->parity != f->parity &&
        p->prev->std == f->std && p->prev->lines == f->lines)
        build_weave(&p->fr, f, p->prev);
    else
        build_bob(&p->fr, f);
    if (p->deint == DEINT_WEAVE) {
        memcpy(p->prev, f, sizeof(field_t));
        p->have_prev = true;
    }
    p->fr.no_signal = false;
    if (p->osd)
        draw_osd(&p->fr, osd);
    notify(p);
    pthread_mutex_unlock(&p->mu);
}

void frame_pub_put_blank(frame_pub_t *p, video_std_t std, const osd_info_t *osd)
{
    pthread_mutex_lock(&p->mu);
    const std_params_t *sp = std_params(std);
    p->fr.w = OUT_W;
    p->fr.h = sp->frame_h;
    p->fr.std = std;
    memset(p->fr.y, 32, (size_t)OUT_W * p->fr.h);
    memset(p->fr.u, 170, (size_t)OUT_CW * (p->fr.h / 2));
    memset(p->fr.v, 118, (size_t)OUT_CW * (p->fr.h / 2));
    p->fr.no_signal = true;
    p->have_prev = false;
    if (p->osd)
        draw_osd(&p->fr, osd);
    notify(p);
    pthread_mutex_unlock(&p->mu);
}

void frame_pub_set_eof(frame_pub_t *p)
{
    pthread_mutex_lock(&p->mu);
    p->eof = true;
    pthread_cond_broadcast(&p->cv);
    pthread_mutex_unlock(&p->mu);
}

bool frame_pub_is_eof(frame_pub_t *p)
{
    pthread_mutex_lock(&p->mu);
    bool e = p->eof;
    pthread_mutex_unlock(&p->mu);
    return e;
}

bool frame_pub_get(frame_pub_t *p, frame_t *dst, uint64_t *seq, int timeout_ms)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout_ms / 1000;
    ts.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000L;
    }
    pthread_mutex_lock(&p->mu);
    while (p->seq == *seq && !p->eof) {
        if (pthread_cond_timedwait(&p->cv, &p->mu, &ts) == ETIMEDOUT)
            break;
    }
    bool got = p->seq != *seq;
    if (got) {
        frame_copy(dst, &p->fr);
        *seq = p->seq;
    }
    pthread_mutex_unlock(&p->mu);
    return got;
}
