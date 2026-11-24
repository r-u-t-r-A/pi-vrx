#include "fusion.h"

#include <math.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"
#include "vec16.h"

#define FS_CODE 20e6
#define QDEPTH 6            /* blocks queued per receiver */
#define OQDEPTH 4           /* fused blocks queued for the decoder */
#define HIST_LINES 400      /* history kept: a field plus the line-offset search span */
#define H_MAX 1290          /* codes per line, upper bound */
#define HOLD_LINES 4        /* output lags this much so line weights see both sides */
#define LINES_KEPT 110      /* per-line noise history: alignment range plus two blocks */
#define SMOOTH 3            /* line noise averaged over +-SMOOTH lines */
#define NV_UNLOCKED 4.0f    /* noise power of a receiver without lock (sync depth^2) */
#define CHUNK 256           /* samples per constant-weight, constant-phase step */
#define BTAPS 8
#define BPH 64
#define OUTLIER 4.5f        /* sigmas */

#define CORR_K 40           /* whole-line offsets searched: +-40 lines (+-2.5 ms) */
#define CORR_PRE 4          /* window: from 4 lines before A's vertical sync ... */
#define CORR_LINES 28       /* ... over 28 lines: the vertical interval, unique line by line */
#define CORR_STEP 3         /* every 3rd code */

typedef struct {
    uint8_t *codes;
    uint32_t n, cap;
    uint64_t k0, cap_start;
    bool discont, late;
    reclock_state_t st;
    decoder_timing_t tm;
    decoder_line_t ll[DECODER_LINES];
    int nll;
} blk_t;

typedef struct {
    blk_t q[QDEPTH];
    unsigned head, tail;        /* pop at head, push at tail */
    bool eof;
    /* codes: x[i] = code base + i */
    uint8_t *x;
    size_t cap, len;
    uint64_t base;
    reclock_state_t st;
    decoder_timing_t tm;
    decoder_line_t ln[LINES_KEPT + DECODER_LINES];
    int nln;
} lane_t;

struct fusion {
    decoder_t *main;
    pthread_mutex_t mu;
    pthread_cond_t cv;
    lane_t l[2];
    uint64_t out_next;
    bool started, discont_out;
    /* alignment: B's content is `delta` codes later than A's */
    bool have_delta;
    double delta;
    int jump;                   /* consecutive blocks with the line phase off */
    int mismatch;
    double cand;
    uint64_t next_check;
    int solo;                   /* receiver used while not aligned */
    float noise_cal;            /* measured (A-B)^2 per porch-predicted noise */
    float *va;
    size_t va_cap;
    float bt[BPH + 1][BTAPS];
    int16_t a16[CHUNK], b16[CHUNK];
    /* fused blocks for the decoder thread */
    struct {
        uint8_t *buf;
        size_t n, cap;
        uint64_t start;
        bool discont, skip;
    } oq[OQDEPTH];
    unsigned ohead, otail;
    bool oeof, skip;
    fusion_stats_t st;
};

static double bessel(double x)
{
    double s = 1, t = 1;
    for (int k = 1; k < 25; k++) {
        t *= (x / (2 * k)) * (x / (2 * k));
        s += t;
    }
    return s;
}

fusion_t *fusion_new(decoder_t *main_dec)
{
    fusion_t *f = calloc(1, sizeof(*f));
    if (!f)
        return NULL;
    f->main = main_dec;
    f->noise_cal = 2.0f;
    pthread_mutex_init(&f->mu, NULL);
    pthread_cond_init(&f->cv, NULL);
    for (int p = 0; p <= BPH; p++) {    /* fractional delay, flat to ~6 MHz */
        const double frac = (double)p / BPH;
        double acc = 0;
        for (int k = 0; k < BTAPS; k++) {
            const double x = (double)(k - (BTAPS / 2 - 1)) - frac, r = x / (BTAPS / 2.0);
            double h = fabs(x) < 1e-9 ? 1.0 : sin(M_PI * x) / (M_PI * x);
            h *= fabs(r) < 1.0 ? bessel(4.0 * sqrt(1.0 - r * r)) / bessel(4.0) : 0.0;
            f->bt[p][k] = (float)h;
            acc += h;
        }
        for (int k = 0; k < BTAPS; k++)
            f->bt[p][k] = (float)(f->bt[p][k] / acc);
    }
    return f;
}

void fusion_free(fusion_t *f)
{
    if (!f)
        return;
    for (int r = 0; r < 2; r++) {
        for (int k = 0; k < QDEPTH; k++)
            free(f->l[r].q[k].codes);
        free(f->l[r].x);
    }
    pthread_mutex_destroy(&f->mu);
    pthread_cond_destroy(&f->cv);
    free(f->va);
    for (int k = 0; k < OQDEPTH; k++)
        free(f->oq[k].buf);
    free(f);
}

fusion_stats_t *fusion_stats(fusion_t *f) { return &f->st; }

void fusion_push(fusion_t *f, int lane, decoder_t *dec, const uint8_t *codes, uint32_t n, uint64_t k0,
                 const reclock_state_t *st, uint64_t cap_start, bool discont, bool late)
{
    lane_t *l = &f->l[lane];
    pthread_mutex_lock(&f->mu);
    while (l->tail - l->head >= QDEPTH && !f->l[1 - lane].eof)
        pthread_cond_wait(&f->cv, &f->mu);
    if (l->tail - l->head >= QDEPTH) {      /* the other receiver ended: nothing consumes this */
        pthread_mutex_unlock(&f->mu);
        return;
    }
    blk_t *b = &l->q[l->tail % QDEPTH];
    pthread_mutex_unlock(&f->mu);
    if (b->cap < n) {
        b->cap = n + 4096;
        b->codes = realloc(b->codes, b->cap);
    }
    memcpy(b->codes, codes, n);
    b->n = n;
    b->k0 = k0;
    b->cap_start = cap_start;
    b->discont = discont;
    b->late = late;
    b->st = *st;
    decoder_get_timing(dec, &b->tm);
    b->nll = decoder_take_lines(dec, b->ll);
    pthread_mutex_lock(&f->mu);
    l->tail++;
    pthread_cond_broadcast(&f->cv);
    pthread_mutex_unlock(&f->mu);
}

void fusion_eof(fusion_t *f, int lane)
{
    pthread_mutex_lock(&f->mu);
    f->l[lane].eof = true;
    pthread_cond_broadcast(&f->cv);
    pthread_mutex_unlock(&f->mu);
}

/* ------------------------------------------------------------ timelines */

/* B code index at the capture instant of A code k, and back. */
static double a2b(const fusion_t *f, double k)
{
    return reclock_from_capture(&f->l[1].st, reclock_to_capture(&f->l[0].st, k));
}

static double b2a(const fusion_t *f, double k)
{
    return reclock_from_capture(&f->l[0].st, reclock_to_capture(&f->l[1].st, k));
}

/* ------------------------------------------------------------- alignment */

/* Whole-line part of B's delay: correlate A's vertical interval (equalising,
 * broad and normal sync lines differ line by line, whatever the picture
 * shows) with B shifted by dl + k*H for every k. Returns 1 when found, 0 if the
 * peak is not clear, -1 if the window is not (yet) inside the history. */
static int resolve_lines(fusion_t *f, double dl, int *k_out)
{
    const lane_t *A = &f->l[0], *B = &f->l[1];
    const double H = A->tm.H, W = CORR_LINES * H;
    const double a0 = A->tm.last_vs - CORR_PRE * H, b00 = a2b(f, a0) + dl;
    if (a0 < (double)A->base || a0 + W + 2 >= (double)(A->base + A->len) || b00 - CORR_K * H < (double)B->base ||
        b00 + CORR_K * H + W + 2 >= (double)(B->base + B->len))
        return -1;
    const int n = (int)(W / CORR_STEP);
    if (f->va_cap < (size_t)n) {
        f->va_cap = (size_t)n;
        f->va = realloc(f->va, f->va_cap * sizeof(float));
    }
    float *va = f->va;
    const uint8_t *pa = A->x + (size_t)(a0 - (double)A->base);
    float ma = 0, sa = 0;
    for (int j = 0; j < n; j++)
        ma += pa[j * CORR_STEP];
    ma /= (float)n;
    for (int j = 0; j < n; j++) {
        va[j] = (float)pa[j * CORR_STEP] - ma;
        sa += va[j] * va[j];
    }
    if (sa < 1.0f)
        return 0;
    float best = -2, second = -2;
    int kbest = 0;
    for (int k = -CORR_K; k <= CORR_K; k++) {
        const uint8_t *pb = B->x + (size_t)(b00 + k * H - (double)B->base + 0.5);
        float mb = 0, sb = 0, sab = 0;
        for (int j = 0; j < n; j++) {
            const float v = pb[j * CORR_STEP];
            mb += v;
            sb += v * v;
            sab += va[j] * v;
        }
        sb -= mb * mb / (float)n;
        const float r = sb > 0 ? sab / sqrtf(sa * sb) : 0;
        if (r > best) {
            second = best;
            best = r;
            kbest = k;
        } else if (r > second) {
            second = r;
        }
    }
    LOGT("fusion: vertical interval correlation %.3f at %d lines, next %.3f", best, kbest, second);
    if (best < 0.3f || best - second < 0.05f)
        return 0;
    *k_out = kbest;
    return 1;
}

static void unalign(fusion_t *f, const char *why)
{
    f->have_delta = false;
    f->jump = f->mismatch = 0;
    f->next_check = 0;
    f->solo = f->l[1].tm.locked && (!f->l[0].tm.locked || f->l[1].tm.snr_db > f->l[0].tm.snr_db);
    atomic_fetch_add(&f->st.realigns, 1);
    LOGD("fusion: %s, realigning", why);
}

/* How far the current delay is off: B's recent lines against the same lines
 * on A. Comparing the same line keeps the line length (slightly different on
 * the two C5 clocks) out of it. */
static bool line_error(const fusion_t *f, double *err)
{
    const lane_t *A = &f->l[0], *B = &f->l[1];
    const double H = A->tm.H;
    double sum = 0;
    int n = 0, ia = A->nln - 1;
    for (int ib = B->nln - 1; ib >= 0 && n < 24; ib--) {
        const double want = b2a(f, B->ln[ib].pos - f->delta);
        while (ia > 0 && fabs(A->ln[ia - 1].pos - want) <= fabs(A->ln[ia].pos - want))
            ia--;
        if (ia < 0 || fabs(A->ln[ia].pos - want) > 0.5 * H)
            continue;               /* not there (yet) */
        sum += want - A->ln[ia].pos;
        n++;
    }
    if (n < 8)
        return false;
    *err = sum / n;
    return true;
}

static void update_alignment(fusion_t *f)
{
    const decoder_timing_t *ta = &f->l[0].tm, *tb = &f->l[1].tm;
    if (!f->have_delta && !(f->solo ? tb : ta)->locked && (f->solo ? ta : tb)->locked)
        f->solo = !f->solo;
    if (!(ta->locked && tb->locked) || fabs(ta->H - tb->H) > 2.0)
        return;
    const double H = tb->H;
    double dl = tb->line_pos - a2b(f, ta->line_pos), e;
    dl -= H * round(dl / H);
    if (f->have_delta && line_error(f, &e)) {
        if (fabs(e) > 6.0) {            /* 0.3 us: a C5 restarted its output */
            if (++f->jump > 12)
                unalign(f, "line phase jumped");
        } else {
            f->jump = 0;
            f->delta += 0.05 * e;
        }
    }
    const uint64_t now = f->l[0].base + f->l[0].len;
    int k;
    const int found = ta->vlock && now >= f->next_check ? resolve_lines(f, dl, &k) : -1;
    if (found == 0)
        f->next_check = now + (uint64_t)(0.01 * FS_CODE);   /* the search is expensive: not on every block */
    if (found > 0) {
        const double cand = dl + k * H;
        f->next_check = now + (uint64_t)(0.5 * FS_CODE);
        if (!f->have_delta) {
            f->delta = cand;
            f->have_delta = true;
            LOGD("fusion: receiver B is %.2f us %s receiver A", fabs(cand) / FS_CODE * 1e6,
                 cand >= 0 ? "behind" : "ahead of");
        } else if (fabs(cand - f->delta) > 0.5 * H) {
            /* Whole lines off: confirm on the next field, then realign. */
            f->next_check = now + (uint64_t)(0.018 * FS_CODE);
            f->mismatch = f->mismatch && fabs(cand - f->cand) < 0.5 * H ? f->mismatch + 1 : 1;
            f->cand = cand;
            if (f->mismatch >= 2)
                unalign(f, "line offset changed");
        } else {
            f->mismatch = 0;
        }
    }
    atomic_store(&f->st.delay_us, (float)(f->delta / FS_CODE * 1e6));
}

/* --------------------------------------------------------------- weights */

typedef struct {
    double pos[LINES_KEPT + DECODER_LINES];  /* A timeline */
    float nv[LINES_KEPT + DECODER_LINES];
    int n, i;
    double H;
    bool locked;
} noise_t;

static void noise_prepare(const fusion_t *f, int lane, noise_t *z)
{
    const lane_t *l = &f->l[lane];
    z->n = l->nln;
    z->i = 0;
    z->H = l->tm.H;
    z->locked = l->tm.locked;
    const double d = f->have_delta ? f->delta : 0.0;
    for (int j = 0; j < l->nln; j++) {
        z->pos[j] = lane ? b2a(f, l->ln[j].pos - d) : l->ln[j].pos;
        /* Mean without the largest two: a click in a porch is not noise. */
        float s = 0, m1 = 0, m2 = 0;
        int c = 0;
        for (int m = j - SMOOTH; m <= j + SMOOTH; m++)
            if (m >= 0 && m < l->nln) {
                const float v = l->ln[m].nv;
                s += v;
                c++;
                if (v > m1) { m2 = m1; m1 = v; }
                else if (v > m2) m2 = v;
            }
        if (c > 4) {
            s -= m1 + m2;
            c -= 2;
        }
        z->nv[j] = fminf(s / (float)c, NV_UNLOCKED) + 2e-4f;
    }
}

/* Noise power around A position k (positions asked in rising order). */
static float noise_at(noise_t *z, double k)
{
    while (z->i + 1 < z->n && z->pos[z->i + 1] <= k)
        z->i++;
    if (!z->n || z->pos[z->i] > k)
        return z->n && z->pos[z->i] - k < 3 * z->H ? z->nv[z->i] : NV_UNLOCKED;
    const int i = z->i;
    if (i + 1 >= z->n)
        return z->locked || k - z->pos[i] < 3 * z->H ? z->nv[i] : NV_UNLOCKED;
    const double span = z->pos[i + 1] - z->pos[i];
    if (span > 3 * z->H)
        return NV_UNLOCKED;             /* no lock in between */
    const float t = span > 0 ? (float)((k - z->pos[i]) / span) : 0;
    return z->nv[i] + t * (z->nv[i + 1] - z->nv[i]);
}

/* ------------------------------------------------------------- combining */

/* A and B in 1/32 fused codes, all int16: these loops vectorise. */
static void scale_a(int16_t *restrict o, const uint8_t *restrict x, int slope, int off)
{
    const v8s k = v8_dup(slope), f = v8_dup(off);
    for (int i = 0; i < CHUNK; i += 8)
        v8_store(o + i, v8_load_u8(x + i) * k + f);
}

/* Taps carry the lane's gain. Products of 6-bit codes fit int16, and wrapped
 * partial sums are harmless: the final value fits. */
static void interp_b(int16_t *restrict o, const uint8_t *restrict x, const int16_t h[BTAPS], int off)
{
    const v8s f = v8_dup(off);
#ifdef __ARM_NEON
    const int16x8_t t = vld1q_s16(h);
    int16x8_t a = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(x)));
    for (int i = 0; i < CHUNK; i += 8) {
        const int16x8_t b = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(x + i + 8)));
        int16x8_t s = vmlaq_laneq_s16((int16x8_t)f, a, t, 0);
        s = vmlaq_laneq_s16(s, vextq_s16(a, b, 1), t, 1);
        s = vmlaq_laneq_s16(s, vextq_s16(a, b, 2), t, 2);
        s = vmlaq_laneq_s16(s, vextq_s16(a, b, 3), t, 3);
        s = vmlaq_laneq_s16(s, vextq_s16(a, b, 4), t, 4);
        s = vmlaq_laneq_s16(s, vextq_s16(a, b, 5), t, 5);
        s = vmlaq_laneq_s16(s, vextq_s16(a, b, 6), t, 6);
        s = vmlaq_laneq_s16(s, vextq_s16(a, b, 7), t, 7);
        vst1q_s16(o + i, s);
        a = b;
    }
#else
    for (int i = 0; i < CHUNK; i += 8) {
        v8s s = f;
        for (int k = 0; k < BTAPS; k++)
            s += v8_load_u8(x + i + k) * v8_dup(h[k]);
        v8_store(o + i, s);
    }
#endif
}

/* u = b + w (a - b), w in Q15; returns how many |a-b| exceed thr and the sum of
 * the others. */
static int mix(uint8_t *restrict out, const int16_t *restrict a, const int16_t *restrict b, int w, int thr, int32_t *sum)
{
    const v8s wq = v8_dup(w), th = v8_dup(thr), r = v8_dup(16);
    v8s cnt = v8_dup(0);
    int32_t s = 0;
    for (int i = 0; i < CHUNK; i += 8) {
        const v8s vb = v8_load(b + i), d = v8_load(a + i) - vb, ad = v8_abs(d);
        cnt -= (v8s)(ad > th);
        s += v8_sum(v8_min(ad, th) >> 3);
        v8_store_u8(out + i, (vb + v8_mulr(d, wq) + r) >> 5);
    }
    const int n = v8_sum(cnt);
    *sum = 8 * s - n * thr;
    return n;
}

static inline uint8_t clip_code(int v)
{
    v = (v + 16) >> 5;
    return (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
}

static float lane_slope(const lane_t *l) { return FUSED_SYNC * 32.0f / fmaxf(l->tm.amp, 5.0f); }

static void emit(fusion_t *f)
{
    lane_t *A = &f->l[0], *B = &f->l[1];
    const bool both = f->have_delta;
    const double d = both ? f->delta : 0.0;
    const bool useA = both || f->solo == 0, useB = both || f->solo == 1;

    /* As far as both receivers have data, less the weight look-ahead. */
    double lim = (double)(A->base + A->len);
    const double limB = b2a(f, (double)(B->base + B->len) - BTAPS - d);
    if (limB < lim)
        lim = limB;
    lim -= HOLD_LINES * A->tm.H;
    if (f->out_next < A->base)
        f->out_next = A->base;
    if (lim < (double)f->out_next + CHUNK)
        return;
    const size_t cnt = (size_t)(lim - (double)f->out_next) / CHUNK * CHUNK;
    pthread_mutex_lock(&f->mu);
    while (f->otail - f->ohead >= OQDEPTH)
        pthread_cond_wait(&f->cv, &f->mu);
    pthread_mutex_unlock(&f->mu);
    const unsigned os = f->otail % OQDEPTH;
    if (f->oq[os].cap < cnt) {
        f->oq[os].cap = cnt + 8192;
        f->oq[os].buf = realloc(f->oq[os].buf, f->oq[os].cap);
    }
    uint8_t *const out = f->oq[os].buf;

    noise_t zn[2];
    noise_prepare(f, 0, &zn[0]);
    noise_prepare(f, 1, &zn[1]);
    const float sa = lane_slope(A), sb = lane_slope(B);
    const int slopeA = (int)lrintf(sa), offA = (int)lrintf(FUSED_BLANK * 32.0f - A->tm.blank * (float)slopeA);
    const int offB = (int)lrintf(FUSED_BLANK * 32.0f - B->tm.blank * lrintf(sb));
    const double scale = a2b(f, (double)f->out_next + 1e6) - a2b(f, (double)f->out_next);
    const double b0 = a2b(f, (double)f->out_next) + d;
    float share = 0;
    double sum = 0, pred = 0;
    uint64_t rej[2] = {0, 0};
    /* two subcarrier cycles in codes: PAL 9.02, NTSC 11.17 */
    const int back = fabs(A->tm.H / FS_CODE * 1e6 - 63.556) < 0.12 ? 11 : 9;

    for (size_t c = 0; c < cnt; c += CHUNK) {
        const uint64_t ka = f->out_next + c;
        uint8_t *o = out + c;
        const uint8_t *pa = A->x + (ka - A->base);
        /* B position, constant fraction over the chunk (ppm differences) */
        const double kb = b0 + ((double)c + 0.5 * CHUNK) * scale * 1e-6 - 0.5 * CHUNK;
        const double fl = floor(kb);
        const long ib = (long)fl - (BTAPS / 2 - 1) - (long)B->base;
        const bool haveB = ib >= 0 && (size_t)ib + CHUNK + BTAPS <= B->len;

        if (useA)
            scale_a(f->a16, pa, slopeA, offA);
        if (useB && haveB) {
            const float *h = f->bt[(int)((kb - fl) * BPH + 0.5)];
            int16_t hq[BTAPS];
            for (int k = 0; k < BTAPS; k++)
                hq[k] = (int16_t)lrintf(h[k] * sb);
            /* rounding must not change the gain: put the rest on the main tap */
            int rest = (int)lrintf(sb), big = 0;
            for (int k = 0; k < BTAPS; k++) {
                rest -= hq[k];
                if (hq[k] > hq[big])
                    big = k;
            }
            hq[big] = (int16_t)(hq[big] + rest);
            interp_b(f->b16, B->x + ib, hq, offB);
        }
        if (!both || !haveB) {
            const int16_t *src = useA ? f->a16 : haveB ? f->b16 : NULL;
            for (int i = 0; i < CHUNK; i++)
                o[i] = src ? clip_code(src[i]) : (uint8_t)FUSED_BLANK;
            share += useA ? 1.0f : 0.0f;
            continue;
        }

        const float na = noise_at(&zn[0], (double)ka + 0.5 * CHUNK), nb = noise_at(&zn[1], (double)ka + 0.5 * CHUNK);
        const float w = nb / (na + nb);
        share += w;
        const float sig = sqrtf(f->noise_cal * (na + nb)) * FUSED_SYNC * 32.0f;
        const int thr = (int)fminf(fmaxf(OUTLIER * sig, 0.35f * FUSED_SYNC * 32.0f), 30000.0f);
        int32_t s;
        int big = mix(o, f->a16, f->b16, (int)lrintf(w * 32767.0f), thr, &s);
        if (sig < 8.0f * FUSED_SYNC) {          /* calibrate on healthy stretches only */
            sum += s;
            pred += sqrtf(na + nb) * CHUNK;
        }
        if (!big)
            continue;
        const float ra = sqrtf(na), rb = sqrtf(nb);
        for (int i = 0; i < CHUNK; i++) {
            const int a = f->a16[i], b = f->b16[i];
            if (abs(a - b) <= thr)
                continue;
            /* One of them clicked. Which fits better: the neighbours (luma), or
             * the sample two subcarrier cycles back (saturated colour)? The
             * noisier receiver needs the better fit. */
            const bool first = c == 0 && i == 0;
            const int prv = first ? (f->a16[1] + f->b16[1]) / 2 : (int)o[i - 1] << 5;
            const int i1 = i + 1 < CHUNK ? i + 1 : i - 1;
            const int nxt = abs(f->a16[i1] - f->b16[i1]) <= thr ? (f->a16[i1] + f->b16[i1]) / 2 : prv;
            const int r1 = (prv + nxt) / 2, r2 = c + (size_t)i >= (size_t)back ? (int)o[i - back] << 5 : r1;
            const int ea1 = abs(a - r1), ea2 = abs(a - r2), eb1 = abs(b - r1), eb2 = abs(b - r2);
            const bool takeA = (float)(ea1 < ea2 ? ea1 : ea2) * ra <= (float)(eb1 < eb2 ? eb1 : eb2) * rb;
            o[i] = clip_code(takeA ? a : b);
            rej[takeA ? 1 : 0]++;
        }
    }
    if (pred > 0) {
        /* sigma = 1.25 mean|d| */
        const float m = 1.25f * (float)(sum / (pred * FUSED_SYNC * 32.0));
        f->noise_cal += 0.05f * (clampf(m * m, 0.3f, 12.0f) - f->noise_cal);
    }
    share /= (float)(cnt / CHUNK);
    atomic_store(&f->st.share[0], share);
    atomic_store(&f->st.share[1], 1.0f - share);
    atomic_store(&f->st.aligned, both);
    atomic_fetch_add(&f->st.outliers[0], rej[0]);
    atomic_fetch_add(&f->st.outliers[1], rej[1]);

    f->oq[os].n = cnt;
    f->oq[os].start = f->out_next;
    f->oq[os].discont = f->discont_out;
    f->oq[os].skip = f->skip;
    f->discont_out = false;
    f->out_next += cnt;
    pthread_mutex_lock(&f->mu);
    f->otail++;
    pthread_cond_broadcast(&f->cv);
    pthread_mutex_unlock(&f->mu);
}

bool fusion_decode_step(fusion_t *f)
{
    pthread_mutex_lock(&f->mu);
    while (f->ohead == f->otail && !f->oeof)
        pthread_cond_wait(&f->cv, &f->mu);
    if (f->ohead == f->otail) {
        pthread_mutex_unlock(&f->mu);
        return false;
    }
    pthread_mutex_unlock(&f->mu);
    const unsigned os = f->ohead % OQDEPTH;
    decoder_set_skip(f->main, f->oq[os].skip);
    decoder_feed_u8(f->main, f->oq[os].buf, (uint32_t)f->oq[os].n, f->oq[os].start, f->oq[os].discont);
    pthread_mutex_lock(&f->mu);
    f->ohead++;
    pthread_cond_broadcast(&f->cv);
    pthread_mutex_unlock(&f->mu);
    return true;
}

/* --------------------------------------------------------------- stepping */

static void lane_append(lane_t *l, const blk_t *b, bool reset)
{
    if (reset || b->k0 != l->base + l->len) {
        l->base = b->k0;
        l->len = 0;
        l->nln = 0;
    }
    if (l->cap < l->len + b->n) {
        l->cap = l->len + b->n + (HIST_LINES + 1000) * H_MAX;
        l->x = realloc(l->x, l->cap);
    }
    memcpy(l->x + l->len, b->codes, b->n);
    l->len += b->n;
    l->st = b->st;
    l->tm = b->tm;
    const int max = LINES_KEPT;
    if (l->nln + b->nll > max) {
        int drop = l->nln + b->nll - max;
        if (drop > l->nln)
            drop = l->nln;
        memmove(l->ln, l->ln + drop, sizeof(l->ln[0]) * (size_t)(l->nln - drop));
        l->nln -= drop;
    }
    memcpy(l->ln + l->nln, b->ll, sizeof(l->ln[0]) * (size_t)b->nll);
    l->nln += b->nll;
}

/* Drop history nobody needs (alignment search, output not yet produced). */
static void lane_trim(lane_t *l, double need)
{
    double lo = (double)(l->base + l->len) - HIST_LINES * H_MAX;
    if (need - 64 < lo)
        lo = need - 64;
    if (lo <= (double)l->base)
        return;
    size_t drop = (size_t)(lo - (double)l->base);
    if (drop < 900 * H_MAX)     /* move rarely: the buffers are cheap, the copies are not */
        return;
    memmove(l->x, l->x + drop, l->len - drop);
    l->len -= drop;
    l->base += drop;
}

bool fusion_step(fusion_t *f)
{
    lane_t *A = &f->l[0], *B = &f->l[1];
    pthread_mutex_lock(&f->mu);
    blk_t *a, *b;
    for (;;) {
        while (A->head == A->tail || B->head == B->tail) {
            if ((A->head == A->tail && A->eof) || (B->head == B->tail && B->eof)) {
                f->oeof = true;
                pthread_cond_broadcast(&f->cv);
                pthread_mutex_unlock(&f->mu);
                return false;
            }
            pthread_cond_wait(&f->cv, &f->mu);
        }
        a = &A->q[A->head % QDEPTH];
        b = &B->q[B->head % QDEPTH];
        if (a->cap_start == b->cap_start)
            break;
        /* One receiver's thread lost blocks: drop the other's copies. */
        lane_t *old = a->cap_start < b->cap_start ? A : B;
        old->head++;
        f->discont_out = true;
        pthread_cond_broadcast(&f->cv);
    }
    pthread_mutex_unlock(&f->mu);

    const bool disc = a->discont || b->discont;
    f->skip = a->late || b->late;
    lane_append(A, a, disc);
    lane_append(B, b, disc);
    pthread_mutex_lock(&f->mu);
    A->head++;
    B->head++;
    pthread_cond_broadcast(&f->cv);
    pthread_mutex_unlock(&f->mu);

    if (!f->started || f->out_next < A->base || f->out_next > A->base + A->len) {
        f->discont_out = f->started;
        f->out_next = A->base;
        f->started = true;
    }
    update_alignment(f);
    emit(f);
    const double d = f->have_delta ? f->delta : 0.0;
    lane_trim(A, (double)f->out_next);
    lane_trim(B, a2b(f, (double)f->out_next) + d);
    return true;
}
