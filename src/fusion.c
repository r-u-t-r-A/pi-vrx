#include "fusion.h"

#include <math.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"

#define FS_CODE 20e6
#define QDEPTH 6            /* blocks queued per receiver */
#define HIST_LINES 400      /* history kept: a field plus the line-offset search span */
#define H_MAX 1290          /* codes per line, upper bound */
#define HOLD_LINES 4        /* output lags this much so line weights see both sides */
#define SMOOTH 3            /* line noise averaged over +-SMOOTH lines */
#define NV_UNLOCKED 4.0f    /* noise power of a receiver without lock (sync depth^2) */
#define CHUNK 128           /* samples per constant-weight, constant-phase step */
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
    bool discont;
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
    decoder_line_t ln[2 * DECODER_LINES];
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
    uint8_t *out;
    size_t out_cap;
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
    free(f->out);
    free(f);
}

fusion_stats_t *fusion_stats(fusion_t *f) { return &f->st; }

void fusion_push(fusion_t *f, int lane, decoder_t *dec, const uint8_t *codes, uint32_t n, uint64_t k0,
                 const reclock_state_t *st, uint64_t cap_start, bool discont)
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
 * shows) with B shifted by dl + k*H for every k. Returns false if the window
 * is not (yet) inside the history or the peak is not clear. */
static bool resolve_lines(fusion_t *f, double dl, int *k_out)
{
    const lane_t *A = &f->l[0], *B = &f->l[1];
    const double H = A->tm.H, W = CORR_LINES * H;
    const double a0 = A->tm.last_vs - CORR_PRE * H, b00 = a2b(f, a0) + dl;
    if (a0 < (double)A->base || a0 + W + 2 >= (double)(A->base + A->len) || b00 - CORR_K * H < (double)B->base ||
        b00 + CORR_K * H + W + 2 >= (double)(B->base + B->len))
        return false;
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
        return false;
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
        return false;
    *k_out = kbest;
    return true;
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
    if (ta->vlock && (!f->have_delta || now >= f->next_check) && resolve_lines(f, dl, &k)) {
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
    double pos[2 * DECODER_LINES];  /* A timeline */
    float nv[2 * DECODER_LINES];
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

/* A and B in 1/32 fused codes. Plain loops on int16: they vectorise. */
static void scale_a(int16_t *restrict o, const uint8_t *restrict x, int n, int slope, int off)
{
    for (int i = 0; i < n; i++)
        o[i] = (int16_t)((int16_t)x[i] * (int16_t)slope + (int16_t)off);
}

static void interp_b(int16_t *restrict o, const uint8_t *restrict x, int n, const int16_t h[BTAPS], int off)
{
    for (int i = 0; i < n; i++) {
        int32_t acc = 0;
        for (int k = 0; k < BTAPS; k++)
            acc += (int32_t)h[k] * (int16_t)x[i + k];
        o[i] = (int16_t)(((acc + 32) >> 6) + off);
    }
}

/* u = b + w (a - b), w in 1/4096; returns how many |a-b| exceed thr and adds
 * up the rest. */
static int mix(uint8_t *restrict out, const int16_t *restrict a, const int16_t *restrict b, int n, int w, int thr,
               int32_t *sum)
{
    int32_t cnt = 0, s = 0;
    for (int i = 0; i < n; i++) {
        const int32_t d = (int32_t)a[i] - b[i];
        const int32_t ad = d < 0 ? -d : d;
        const int32_t big = ad > thr;
        cnt += big;
        s += big ? 0 : ad;
        const int32_t u = (b[i] + ((d * w + 2048) >> 12) + 16) >> 5;
        out[i] = (uint8_t)(u < 0 ? 0 : u > 255 ? 255 : u);
    }
    *sum = s;
    return cnt;
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
    if (f->out_cap < cnt) {
        f->out_cap = cnt;
        f->out = realloc(f->out, cnt);
    }

    noise_t zn[2];
    noise_prepare(f, 0, &zn[0]);
    noise_prepare(f, 1, &zn[1]);
    const float sa = lane_slope(A), sb = lane_slope(B);
    const int slopeA = (int)lrintf(sa), offA = (int)lrintf(FUSED_BLANK * 32.0f - A->tm.blank * sa);
    const int offB = (int)lrintf(FUSED_BLANK * 32.0f - B->tm.blank * sb);
    const double scale = a2b(f, (double)f->out_next + 1e6) - a2b(f, (double)f->out_next);
    const double b0 = a2b(f, (double)f->out_next) + d;
    float share = 0;
    double sum = 0, pred = 0;
    uint64_t rej[2] = {0, 0};
    /* two subcarrier cycles in codes: PAL 9.02, NTSC 11.17 */
    const int back = fabs(A->tm.H / FS_CODE * 1e6 - 63.556) < 0.12 ? 11 : 9;

    for (size_t c = 0; c < cnt; c += CHUNK) {
        const uint64_t ka = f->out_next + c;
        uint8_t *o = f->out + c;
        const uint8_t *pa = A->x + (ka - A->base);
        /* B position, constant fraction over the chunk (ppm differences) */
        const double kb = b0 + ((double)c + 0.5 * CHUNK) * scale * 1e-6 - 0.5 * CHUNK;
        const double fl = floor(kb);
        const long ib = (long)fl - (BTAPS / 2 - 1) - (long)B->base;
        const bool haveB = ib >= 0 && (size_t)ib + CHUNK + BTAPS <= B->len;

        if (useA)
            scale_a(f->a16, pa, CHUNK, slopeA, offA);
        if (useB && haveB) {
            const float *h = f->bt[(int)((kb - fl) * BPH + 0.5)];
            int16_t hq[BTAPS];
            for (int k = 0; k < BTAPS; k++)
                hq[k] = (int16_t)lrintf(h[k] * sb * 64.0f);
            interp_b(f->b16, B->x + ib, CHUNK, hq, offB);
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
        int big = mix(o, f->a16, f->b16, CHUNK, (int)lrintf(w * 4096.0f), thr, &s);
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

    decoder_feed_u8(f->main, f->out, (uint32_t)cnt, f->out_next, f->discont_out);
    f->discont_out = false;
    f->out_next += cnt;
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
        l->cap = 2 * (l->len + b->n) + (HIST_LINES + 100) * H_MAX;
        l->x = realloc(l->x, l->cap);
    }
    memcpy(l->x + l->len, b->codes, b->n);
    l->len += b->n;
    l->st = b->st;
    l->tm = b->tm;
    const int max = 2 * DECODER_LINES;
    if (l->nln + b->nll > max) {
        const int drop = l->nln + b->nll - max;
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
    if (drop < 200 * H_MAX)
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
