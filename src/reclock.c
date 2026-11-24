#include "reclock.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

#define WIN 2560            /* samples per phase measurement (~0.1 ms, multiple of 5) */
#define WIN_EVERY 4         /* measure one window in four: the clocks drift slowly */
#define ALPHA_MIN 0.012     /* loop gain: averages over many slips of the stepwise phase */
#define TAIL 4

struct reclock {
    int lane;
    bool started;
    uint64_t base;              /* capture index of buf[0] */
    uint8_t *buf;
    uint32_t len, cap;
    uint8_t *out;
    uint32_t out_cap;
    uint64_t k_next;
    /* PLL on the repeat phase D (samples, unwrapped) at capture index t0 */
    double D, rate, t0;         /* rate: D per sample */
    int nmeas;
    float contrast;
};

bool reclock_supported(double fs) { return fabs(fs - 25e6) < 25e6 * 400e-6; }

reclock_t *reclock_new(int lane)
{
    reclock_t *rc = calloc(1, sizeof(*rc));
    if (rc)
        rc->lane = lane;
    return rc;
}

void reclock_free(reclock_t *rc)
{
    if (!rc)
        return;
    free(rc->buf);
    free(rc->out);
    free(rc);
}

void reclock_state(const reclock_t *rc, reclock_state_t *st)
{
    st->locked = rc->nmeas > 8;
    st->d0 = rc->D;
    st->d1 = rc->rate;
    st->t0 = rc->t0;
    st->ppm = (float)(rc->rate * 0.25 * 1e6);
    st->contrast = rc->contrast;
}

double reclock_to_capture(const reclock_state_t *st, double k)
{
    /* pos = 1.25k - .5 - .25 (d0 + d1 (pos - t0)) */
    return (RECLOCK_RATIO * k - 0.5 - 0.25 * (st->d0 - st->d1 * st->t0)) / (1.0 + 0.25 * st->d1);
}

double reclock_from_capture(const reclock_state_t *st, double pos)
{
    return (pos + 0.5 + 0.25 * (st->d0 + st->d1 * (pos - st->t0))) / RECLOCK_RATIO;
}

/* Which of the five phases repeats, over n samples at p (p[-1] valid,
 * capture index of p[0] is idx). */
static void measure(reclock_t *rc, const uint8_t *p, uint32_t n, uint64_t idx)
{
    uint32_t cnt[5] = {0};
    const uint8_t *prev = p - 1;
#ifdef __ARM_NEON
    /* 80 = 5 x 16: byte lane l of vector v has residue (16 v + l) % 5 */
    uint32_t i = 0;
    while (i + 80 <= n) {
        uint8x16_t acc[5] = {vdupq_n_u8(0), vdupq_n_u8(0), vdupq_n_u8(0), vdupq_n_u8(0), vdupq_n_u8(0)};
        for (int r = 0; r < 255 && i + 80 <= n; r++, i += 80)
            for (int v = 0; v < 5; v++)
                acc[v] = vsubq_u8(acc[v], vceqq_u8(vld1q_u8(p + i + 16 * v), vld1q_u8(prev + i + 16 * v)));
        for (int v = 0; v < 5; v++) {
            uint8_t t[16];
            vst1q_u8(t, acc[v]);
            for (int l = 0; l < 16; l++)
                cnt[(16 * v + l) % 5] += t[l];
        }
    }
    for (; i < n; i++)
        cnt[i % 5] += p[i] == prev[i];
#else
    for (uint32_t j = 0; j < 5; j++) {
        uint32_t c = 0;
        for (uint32_t i = j; i < n; i += 5)
            c += p[i] == prev[i];
        cnt[j] = c;
    }
#endif
    float re = 0, im = 0, tot = 0;
    static const float cs[5] = {1.0f, 0.309017f, -0.809017f, -0.809017f, 0.309017f};
    static const float sn[5] = {0.0f, 0.951057f, 0.587785f, -0.587785f, -0.951057f};
    const uint32_t i5 = (uint32_t)(idx % 5);
    for (uint32_t j = 0; j < 5; j++) {
        const uint32_t a = (j + i5) % 5;
        re += cs[a] * (float)cnt[j];
        im += sn[a] * (float)cnt[j];
        tot += (float)cnt[j];
    }
    const float per = (float)(n / 5);
    const float con = hypotf(re, im) / per;
    const double t = (double)idx + 0.5 * n;
    const double pred = rc->D + rc->rate * (t - rc->t0);
    rc->contrast += 0.02f * (con - rc->contrast);
    rc->t0 = t;
    rc->D = pred;
    if (con < 0.25f || tot > 4.6f * per)
        return;                 /* flat or garbage: coast */
    double e = atan2f(im, re) / (2.0 * M_PI) * 5.0 - pred;
    e -= 5.0 * round(e / 5.0);
    if (rc->nmeas == 0) {
        rc->D = pred + e;
        rc->nmeas = 1;
        return;
    }
    /* Gear shift: stay wide until the clock difference has pulled in. */
    double al = 1.5 / (rc->nmeas + 2.0);
    const double floor_al = rc->nmeas < 300 ? 0.06 : rc->nmeas < 800 ? 0.025 : ALPHA_MIN;
    if (al < floor_al)
        al = floor_al;
    if (rc->nmeas < 1000)
        rc->nmeas++;
    rc->D += al * e;
    rc->rate += al * al * 0.5 * e / (double)n;
    const double rmax = 4.0 * 150e-6;   /* 150 ppm: a slip of D is a quarter code period */
    rc->rate = rc->rate > rmax ? rmax : rc->rate < -rmax ? -rmax : rc->rate;
}

const uint8_t *reclock_feed(reclock_t *rc, const uint16_t *s, uint32_t n, uint64_t start, bool discont, uint32_t *n_out,
                            uint64_t *k_start)
{
    if (rc->started && (start != rc->base + rc->len))
        discont = true;
    if (!rc->started || discont) {
        rc->base = start;
        rc->len = 0;
        rc->k_next = 0;
        if (rc->started)
            rc->nmeas = rc->nmeas ? 1 : 0;   /* after a gap: keep the rate, re-acquire the phase */
        rc->started = true;
    }
    if (rc->cap < rc->len + n) {
        rc->cap = rc->len + n + WIN;
        rc->buf = realloc(rc->buf, rc->cap);
        rc->out_cap = rc->cap;
        rc->out = realloc(rc->out, rc->out_cap);
    }
    uint8_t *restrict q = rc->buf + rc->len;
    if (rc->lane)
        for (uint32_t i = 0; i < n; i++)
            q[i] = (uint8_t)((s[i] >> 8) & 63);
    else
        for (uint32_t i = 0; i < n; i++)
            q[i] = (uint8_t)(s[i] & 63);
    const uint32_t fresh = rc->len;
    rc->len += n;

    /* Phase measurements over the new samples (first needs p[-1]). */
    const uint32_t every = rc->nmeas < 300 ? 1 : WIN_EVERY;
    for (uint32_t w = fresh ? fresh : 5; w + WIN <= rc->len; w += WIN * every)
        measure(rc, rc->buf + w, WIN, rc->base + w);

    /* Emit every code whose nearest sample is present. Within one block the
     * grid is a straight line: the loop moves far less than 0.01 sample. */
    reclock_state_t st;
    reclock_state(rc, &st);
    const double step = RECLOCK_RATIO / (1.0 + 0.25 * st.d1);
    uint64_t k = rc->k_next;
    if (!k) {
        const double k0 = ceil(reclock_from_capture(&st, (double)rc->base + 1.0));
        k = k0 < 1 ? 1 : (uint64_t)k0;
    }
    double pos = reclock_to_capture(&st, (double)k) - (double)rc->base;
    if (pos < 0)
        pos = 0;
    const double lim = (double)rc->len - 1.0;
    uint32_t m = 0;
    *k_start = k;
    if (pos < lim) {
        uint32_t cnt = (uint32_t)((lim - pos) / step) + 1;
        if (cnt > rc->out_cap)
            cnt = rc->out_cap;
        /* 32.32 fixed point: no per-sample float conversions */
        uint64_t ph = (uint64_t)((pos + 0.5) * 4294967296.0);
        const uint64_t dp = (uint64_t)(step * 4294967296.0 + 0.5);
        const uint8_t *b = rc->buf;
        uint8_t *o = rc->out;
        /* 12 codes come from 15 samples in one of four patterns, chosen by the
         * grid phase (the drift inside 15 samples is far below one sample). */
        static const uint8_t pat[4][16] = {
            {0, 1, 2, 3, 5, 6, 7, 8, 10, 11, 12, 13, 15, 15, 15, 15},
            {0, 1, 2, 4, 5, 6, 7, 9, 10, 11, 12, 14, 15, 15, 15, 15},
            {0, 1, 3, 4, 5, 6, 8, 9, 10, 11, 13, 14, 15, 15, 15, 15},
            {0, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15, 15, 15, 15},
        };
        for (; m + 16 <= cnt && (ph >> 32) + 16 <= rc->len; m += 12, ph += 12 * dp) {
            const uint8_t *src = b + (ph >> 32), *t = pat[(ph >> 30) & 3];
#ifdef __ARM_NEON
            vst1q_u8(o + m, vqtbl1q_u8(vld1q_u8(src), vld1q_u8(t)));
#else
            for (int k = 0; k < 12; k++)
                o[m + (uint32_t)k] = src[t[k]];
#endif
        }
        for (; m < cnt; m++, ph += dp)
            o[m] = b[ph >> 32];
    }
    rc->k_next = k + m;
    *n_out = m;

    /* Keep what the next code needs. */
    const uint32_t keep = TAIL < rc->len ? TAIL : rc->len;
    const uint32_t drop = rc->len - keep;
    memmove(rc->buf, rc->buf + drop, keep);
    rc->base += drop;
    rc->len = keep;
    return rc->out;
}
