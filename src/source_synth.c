/* Synthetic source: emulates one or two C5VRX receivers watching the same
 * VTX, captured by the Pi's SMI bus.
 *
 *  - composite from cvbs_synth, quantised like the C5 6-bit DAC
 *    (blank = code 20, sync depth configurable per receiver)
 *  - C5 code rate 20 MS/s, sampled asynchronously at cfg.fs
 *  - per-receiver gaussian noise, FM "clicks" and alternating fades
 *  - bit-2 skew glitches at code transitions
 *  - optional SMI read gaps with an inaccurate gap estimate
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "common.h"
#include "cvbs_synth.h"
#include "source.h"

#define C5_RATE 20e6
#define C5_BLANK 20.0f

typedef struct {
    cvbs_synth_t cs;
    uint64_t rng;
} synth_priv_t;

static inline uint64_t xorshift(uint64_t *s)
{
    uint64_t x = *s;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    return *s = x;
}

static inline float urand(uint64_t *s) { return (float)(xorshift(s) >> 40) * (1.0f / 16777216.0f); }

static inline float gauss(uint64_t *s)
{
    float u1 = urand(s) + 1e-7f, u2 = urand(s);
    return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

static void *synth_thread(void *arg)
{
    source_t *s = arg;
    synth_priv_t *p = s->priv;
    const source_cfg_t *c = &s->cfg;
    const double fs = c->fs;
    const int nrx = c->synth_nrx;
    const double sync_ratio = p->cs.sp->sync_ratio;
    const double line_s = p->cs.sp->line_us * 1e-6;
    const double total = c->seconds > 0 ? c->seconds * fs : 0;

    uint64_t idx_true = 0, idx_virt = 0;
    long m_last[2] = {-1, -1};
    const bool own_clocks = c->synth_ppm[0] != c->synth_ppm[1] || c->synth_phase[0] != c->synth_phase[1];
    double v_prev = 0;
    int code[2] = {20, 20}, prev[2] = {20, 20};
    double dip_start[2] = {0, 0}, dip_end[2] = {0, 0};
    int click_left[2] = {0, 0};
    float click_amp[2] = {0, 0};
    double next_gap = c->synth_gap_every > 0 ? c->synth_gap_every * fs : 0;
    /* Fake vertical syncs: three broad pulses half a line apart, starting
     * on a line boundary somewhere in the field (worst case for the decoder:
     * they pass the alignment check, only the vsync window rejects them). */
    double fake_t = c->synth_fakevs > 0 ? (0.05 + urand(&p->rng) / c->synth_fakevs) : 1e30;
    const double t0 = now_s();

    while (s->running && atomic_load(&g_running)) {
        if (total > 0 && (double)idx_true >= total)
            break;
        ring_block_t *b = ring_acquire(s->ring);
        if (next_gap > 0 && (double)idx_true >= next_gap) {
            uint64_t g = (uint64_t)(c->synth_gap_us * 1e-6 * fs);
            /* The real SMI source only knows the gap from wall-clock timing. */
            uint64_t est = (uint64_t)(g * (0.7 + 0.6 * urand(&p->rng)));
            idx_true += g;
            idx_virt += est;
            b->flags |= RING_F_DISCONT;
            b->gap_est = est;
            next_gap += c->synth_gap_every * fs;
            atomic_fetch_add(&s->gaps, 1);
        }
        const uint32_t n = s->ring->block_samples;
        b->start = idx_virt;
        b->n = n;

        for (uint32_t i = 0; i < n; i++) {
            double t = (double)(idx_true + i) / fs;
            uint16_t out = 0;
            for (int r = 0; r < nrx; r++) {
                /* Each C5 runs on its own clock. */
                const double rate = C5_RATE * (1.0 + c->synth_ppm[r] * 1e-6);
                const double tc = t * rate + c->synth_phase[r];
                const long m = (long)tc;
                if (m != m_last[r]) {
                    const double tm = ((double)m - c->synth_phase[r]) / rate;
                    double v = r ? v_prev : cvbs_synth_eval(&p->cs, tm);
                    if (r && own_clocks)
                        v = cvbs_synth_eval(&p->cs, tm);
                    v_prev = v;
                    if (tm >= fake_t) {
                        const double hl = 0.5 * line_s, ph = tm - fake_t;
                        if (ph < 3 * hl) {
                            if (fmod(ph, hl) < hl - 4.7e-6)
                                v = -1.0 / sync_ratio;
                        } else {
                            double next = tm + (0.5 + urand(&p->rng)) / c->synth_fakevs;
                            fake_t = ceil(next / line_s) * line_s;
                        }
                    }
                    float fade[2] = {0, 0};
                    if (c->synth_fade_period > 0) {
                        double ph = sin(2.0 * M_PI * tm / c->synth_fade_period);
                        fade[0] = ph > 0 ? (float)(ph * ph) : 0;
                        fade[1] = ph < 0 ? (float)(ph * ph) : 0;
                    }
                    {
                        float sigma = c->synth_noise[r] + c->synth_fade_noise * fade[r];
                        double vr = v;
                        const double dly = c->synth_delay_step > 0 && tm >= c->synth_delay_step ? c->synth_delay2_us
                                                                                                 : c->synth_delay_us;
                        if (r == 1 && dly != 0)
                            vr = cvbs_synth_eval(&p->cs, tm - dly * 1e-6);
                        if (c->synth_dip_every > 0) {
                            if (tm >= dip_end[r]) {
                                if (dip_end[r] > 0 || tm >= 0.15) {   /* leave time to lock first */
                                    dip_start[r] = tm + c->synth_dip_every * (0.3 + 1.4 * urand(&p->rng));
                                    dip_end[r] = dip_start[r] + c->synth_dip_len;
                                }
                            } else if (tm >= dip_start[r]) {
                                sigma = c->synth_dip_noise;
                                vr *= 0.3;
                            }
                        }
                        if (tm >= c->synth_off_from[r] && tm < c->synth_off_to[r]) {
                            vr = 0;
                            sigma = 15.0f;
                        }
                        float x = C5_BLANK + (float)(vr * c->synth_sync[r] * sync_ratio) + gauss(&p->rng) * sigma;
                        float pclick = c->synth_clicks[r] * (1.0f + 4.0f * fade[r]) / (float)(line_s * C5_RATE);
                        if (click_left[r] == 0 && pclick > 0 && urand(&p->rng) < pclick) {
                            click_left[r] = 1 + (int)(urand(&p->rng) * 2);
                            click_amp[r] = (urand(&p->rng) < 0.5f ? -1.0f : 1.0f) * (10.0f + 20.0f * urand(&p->rng));
                        }
                        if (click_left[r] > 0) {
                            x += click_amp[r];
                            click_left[r]--;
                        }
                        prev[r] = code[r];
                        code[r] = clampi((int)lrintf(x), 0, 63);
                    }
                    m_last[r] = m;
                }
                const double frac = tc - (double)m;
                {
                    int cc = code[r];
                    if (frac < c->synth_skew)
                        cc = (cc & ~4) | (prev[r] & 4);
                    out |= (uint16_t)(cc << (8 * r));
                }
            }
            b->data[i] = out;
        }
        idx_true += n;
        idx_virt += n;
        b->t_wall = now_s();
        ring_publish(s->ring, b);
        atomic_fetch_add(&s->samples, n);
        atomic_fetch_add(&s->blocks, 1);

        if (c->synth_realtime) {
            double due = t0 + (double)idx_true / fs;
            double now = now_s();
            if (due > now)
                usleep((useconds_t)((due - now) * 1e6));
            double el = now_s() - t0;
            if (el > 0)
                atomic_store(&s->rate_meas, (double)idx_true / el);
        }
    }
    ring_set_eof(s->ring);
    return NULL;
}

static void synth_close(source_t *s) { free(s->priv); }

int source_synth_init(source_t *s)
{
    synth_priv_t *p = calloc(1, sizeof(*p));
    if (!p)
        return -1;
    cvbs_synth_init(&p->cs, s->cfg.synth_std);
    p->rng = 0x9E3779B97F4A7C15ull;
    s->priv = p;
    s->thread_fn = synth_thread;
    s->close_fn = synth_close;
    s->realtime = s->cfg.synth_realtime;
    s->nlanes = s->cfg.synth_nrx;
    LOGI("synth source: %s, %d receiver(s), %.2f MS/s capture, C5 20 MS/s",
         std_name(s->cfg.synth_std), s->nlanes, s->cfg.fs / 1e6);
    return 0;
}
