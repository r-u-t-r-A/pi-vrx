#include "decoder.h"

#include <math.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"

#define GRID_PRE 72                 /* pixels before active start (covers burst) */
#define GRID_POST 16
#define GRID_N (GRID_PRE + OUT_W + GRID_POST)
#define LPF_TAPS 21                 /* reference FIR design (unused by the fast path) */
#define CL 9                        /* chroma running-sum length, pixels */
#define LPF_HALF (LPF_TAPS / 2)
#define PQ_CAP 2048
#define RS_TAPS 20                  /* anti-alias resampler: input rate -> 13.5 MHz grid */
#define RS_PH 64
#define SR_TAPS 8                   /* short resampler for clean 20 MS/s codes (no hold images) */
#define ACQ_N 8

enum { ST_ACQ = 0, ST_LOCK = 1 };

#define JOBQ 64
#define MAX_WORKERS 4

/* Scratch for decoding one line (one per worker, one for inline use). */
typedef struct {
    float v[GRID_N], zr[GRID_N], zi[GRID_N], lc[GRID_N], ls[GRID_N];
    float Cr[OUT_W], Ci[OUT_W], U[OUT_W], V[OUT_W];
    float Pr[GRID_N + 2], Pi[GRID_N + 2], Qr[GRID_N + 2], Qi[GRID_N + 2];
    float *fl;
} line_work_t;

/* Everything a worker needs to decode one active line on its own: the raw
 * samples and the parameters that the sequential state (levels, PLL,
 * colour reference) fixed for this line. */
typedef struct {
    field_t *fld;
    int row;
    float q;
    uint8_t *raw;      /* raw[k] = sample seg0 - 1 + k */
    int nseg;          /* seg1 - seg0 */
    float x0, dx;      /* grid point 0 relative to seg0, input samples per pixel */
    float bl, gain;
    float c0, s0;      /* subcarrier LO at grid point 0 */
    bool sub, colour, hq;
    float acc, rot_re, rot_im, sw, ys, yo;
} line_job_t;

typedef struct {
    double fall, rise;
} pulse_t;

struct decoder {
    decoder_cfg_t cfg;
    int id;
    field_sink_t sink;
    decoder_stats_t st;

    /* history: hb[i] = sample (base + i) */
    uint8_t *hb;
    size_t cap, len;
    uint64_t base;
    bool started;
    uint8_t med_a, med_b;

    /* slicer */
    int nb;
    uint64_t scan;
    int32_t sum;
    bool sum_ok;
    bool low;
    double cur_fall;
    bool have_pend;
    pulse_t pend;
    pulse_t pq[PQ_CAP];
    int pq_head, pq_n;

    /* levels, DAC codes */
    float tip, blank, thr;
    bool lvl_ok;
    float hist[256];
    int code_max;        /* 63 for raw C5 codes, 255 for the fused stream */

    /* standard */
    video_std_t std;
    const std_params_t *sp;
    int votes_pal, votes_ntsc;
    bool std_ok;         /* standard confirmed (or forced): fields may be emitted */

    /* line PLL */
    int state;
    double H;            /* line period, samples */
    double H_nom;
    double cur_h;        /* start (0H) of the line awaiting processing */
    bool cur_matched;
    double cur_width;
    int coast;
    bool have_unmatched;
    double last_unmatched;
    double acq[ACQ_N];
    int acq_n;
    bool resync_pending;
    uint64_t resync_pos;
    uint64_t gap_lo, gap_hi;

    /* vertical */
    bool have_k0;
    double k0;
    int parity;
    bool have_vs;
    bool vlock;          /* vertical timing confirmed: vsyncs only inside the window */
    int vcand_n;         /* consistent out-of-window vsyncs (re-acquisition) */
    double vcand_last;
    double last_vs;
    double lpf_meas;
    int broad_run;
    double first_broad, last_broad;

    /* subcarrier LO (absolute time, continuous) */
    double fs_cal;
    double lo_ref_idx;
    double lo_ref_cyc;

    /* colour */
    float Rr, Ri;
    bool R_ok;
    float Ab;
    float pbr, pbi;
    bool pb_ok;
    int pb_row;
    int pal_s;

    field_t *fld;
    uint32_t field_seq;

    /* line decoding: inline (workers == 0) or on worker threads */
    line_work_t main_work;
    line_job_t *jobs;
    uint8_t *job_raw;          /* JOBQ * raw_cap bytes */
    uint8_t job_state[JOBQ];   /* 0 free, 1 queued/running */
    size_t raw_cap;
    unsigned q_tail, q_next, q_ready;
    int pending;
    bool q_stop;
    pthread_mutex_t q_mu;
    pthread_cond_t q_job, q_done;
    pthread_t workers[MAX_WORKERS];
    int nworkers;
    double tb_a, tb_b;                    /* published position = tb_a * index + tb_b */
    decoder_line_t ll[DECODER_LINES];     /* line log since the last decoder_take_lines() */
    int nll;
    float lpf[LPF_TAPS];
    float rs[RS_PH + 1][RS_TAPS];
    float sr[RS_PH + 1][SR_TAPS];
    bool clean;                           /* input is one sample per C5 code */
    float pf_a, pf_b;                     /* fast-path prefilter taps [b a b] */
    float lo_tc[GRID_N], lo_ts[GRID_N];   /* cos/sin(j * dphi) for the current standard */
    long fl_cap;
    uint8_t *lane;                        /* lane-extracted input block */
    size_t lane_cap;
};

void decoder_cfg_default(decoder_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->lane = 0;
    c->std_force = STD_UNKNOWN;
    c->fs_nom = 25e6;
    c->reclock = true;
    c->mono = false;
    c->chroma_vavg = true;
    c->median3 = false;
    c->hq_resample = false;
    c->saturation = 1.0f;
    c->contrast = 1.0f;
    c->brightness = 0.0f;
    c->hshift_us = 0.0f;
    c->voffset = 0;
    c->pll_alpha = 0.35f;
    c->pll_beta = 0.02f;
    c->workers = 2;
}

/* ---------------------------------------------------------------- helpers */

static inline double spu_of(const decoder_t *d) { return d->H / d->sp->line_us; }

static void set_std(decoder_t *d, video_std_t s, double idx)
{
    d->std = s;
    d->sp = std_params(s);
    d->R_ok = false;
    d->pb_ok = false;
    d->Ab = 0;
    d->lpf_meas = d->sp->lines_per_field;
    d->lo_ref_idx = idx;
    d->lo_ref_cyc = 0;
    d->H_nom = d->sp->line_us * 1e-6 * d->cfg.fs_nom;
    d->votes_pal = d->votes_ntsc = 0;
    /* Pixel spacing is 1/13.5 MHz of true time, so the subcarrier phase step
     * per pixel is a per-standard constant: tabulate it once. */
    const double dph = 2.0 * M_PI * d->sp->fsc_hz / PX_RATE;
    for (int j = 0; j < GRID_N; j++) {
        d->lo_tc[j] = (float)cos(j * dph);
        d->lo_ts[j] = (float)sin(j * dph);
    }
    atomic_store(&d->st.std, (int)s);
}

static void pq_push(decoder_t *d, pulse_t p)
{
    if (d->pq_n == PQ_CAP) {
        d->pq_head = (d->pq_head + 1) % PQ_CAP;
        d->pq_n--;
    }
    d->pq[(d->pq_head + d->pq_n) % PQ_CAP] = p;
    d->pq_n++;
}

static inline pulse_t *pq_front(decoder_t *d) { return &d->pq[d->pq_head]; }

static pulse_t pq_pop(decoder_t *d)
{
    pulse_t p = d->pq[d->pq_head];
    d->pq_head = (d->pq_head + 1) % PQ_CAP;
    d->pq_n--;
    return p;
}

static int mean_var(const decoder_t *d, double a, double b, float *m, float *var)
{
    long i0 = (long)ceil(a), i1 = (long)floor(b);
    if (i0 < 0)
        i0 = 0;
    if (i1 >= (long)d->len)
        i1 = (long)d->len - 1;
    int n = 0;
    float s = 0, s2 = 0;
    for (long i = i0; i <= i1; i++) {
        float x = d->hb[i];
        s += x;
        s2 += x * x;
        n++;
    }
    if (n == 0) {
        *m = 0;
        *var = 0;
        return 0;
    }
    *m = s / n;
    *var = fmaxf(0.0f, s2 / n - (*m) * (*m));
    return n;
}

/* Same statistics on the prefiltered signal (what the picture path sees),
 * used for porch noise so single-sample glitches don't dominate. */
static int mean_var_pf(const decoder_t *d, double a, double b, float *m, float *var)
{
    long i0 = (long)ceil(a), i1 = (long)floor(b);
    if (i0 < 1)
        i0 = 1;
    if (i1 >= (long)d->len - 1)
        i1 = (long)d->len - 2;
    int n = 0;
    float s = 0, s2 = 0;
    for (long i = i0; i <= i1; i++) {
        float x = d->pf_b * ((float)d->hb[i - 1] + (float)d->hb[i + 1]) + d->pf_a * (float)d->hb[i];
        s += x;
        s2 += x * x;
        n++;
    }
    if (n == 0) {
        *m = 0;
        *var = 0;
        return 0;
    }
    *m = s / n;
    *var = fmaxf(0.0f, s2 / n - (*m) * (*m));
    return n;
}

static uint64_t keep_from(const decoder_t *d)
{
    uint64_t k = d->scan > (uint64_t)(d->nb + 8) ? d->scan - (uint64_t)(d->nb + 8) : 0;
    double back = 2.5 * d->H;
    double c = d->state == ST_LOCK ? d->cur_h - back : (double)d->scan - back;
    if (c < (double)k)
        k = c > 0 ? (uint64_t)c : 0;
    if (k < d->base)
        k = d->base;
    return k;
}

static void compact(decoder_t *d)
{
    uint64_t k = keep_from(d);
    size_t shift = (size_t)(k - d->base);
    if (shift == 0)
        return;
    if (shift > d->len)
        shift = d->len;
    memmove(d->hb, d->hb + shift, d->len - shift);
    d->len -= shift;
    d->base += shift;
}

static void ensure_cap(decoder_t *d, size_t extra)
{
    if (d->len + extra <= d->cap)
        return;
    compact(d);
    if (d->len + extra <= d->cap)
        return;
    size_t nc = d->cap;
    while (nc < d->len + extra)
        nc *= 2;
    d->hb = realloc(d->hb, nc);
    d->cap = nc;
}

/* ------------------------------------------------------------ fields */

static void wait_lines(decoder_t *d);

static void emit_field(decoder_t *d, uint64_t end)
{
    field_t *f = d->fld;
    if (!f)
        return;
    wait_lines(d);
    if (d->cfg.chroma_vavg) {
        /* Average each line's chroma with the line above (bottom up, so
         * every average uses the original neighbour): halves colour noise
         * and, for PAL, cancels phase errors between V-switched lines. */
        for (int r = f->lines - 1; r > 0; r--) {
            if (f->q[r] <= Q_INVALID || f->q[r - 1] <= Q_INVALID)
                continue;
            for (int m = 0; m < OUT_CW; m++) {
                f->cb[r][m] = (uint8_t)((f->cb[r][m] + f->cb[r - 1][m] + 1) >> 1);
                f->cr[r][m] = (uint8_t)((f->cr[r][m] + f->cr[r - 1][m] + 1) >> 1);
            }
        }
    }
    f->end_pos = (uint64_t)(d->tb_a * (double)end + d->tb_b);
    f->t_wall = now_s();
    double s = 0;
    int n = 0;
    for (int r = 0; r < f->lines; r++)
        if (f->q[r] > Q_INVALID) {
            s += f->q[r];
            n++;
        }
    f->valid_lines = n;
    f->snr_db = n ? (float)(s / n) : Q_INVALID;
    f->seq = d->field_seq++;
    d->fld = NULL;
    atomic_fetch_add(&d->st.fields, 1);
    d->sink.submit(d->sink.ctx, f);
}

static void start_field(decoder_t *d, double vs, int parity, double k0, bool synthetic)
{
    d->have_k0 = true;
    d->k0 = k0;
    d->parity = parity;
    field_t *f = d->sink.acquire ? d->sink.acquire(d->sink.ctx, d->id) : NULL;   /* NULL: sync-only */
    d->fld = f;
    if (!f)
        return;
    f->start_pos = (uint64_t)(d->tb_a * vs + d->tb_b);
    f->parity = parity;
    f->std = d->std;
    f->lines = d->sp->active_lines;
    f->rx = d->id;
    f->synthetic_vsync = synthetic;
    f->valid_lines = 0;
    for (int r = 0; r < MAX_FIELD_LINES; r++)
        f->q[r] = Q_INVALID;
}

static void update_fs_cal(decoder_t *d, double idx)
{
    double fs_new = d->H / (d->sp->line_us * 1e-6);
    if (fabs(fs_new / d->fs_cal - 1.0) > 2e-6) {
        double cyc = d->lo_ref_cyc + d->sp->fsc_hz * (idx - d->lo_ref_idx) / d->fs_cal;
        d->lo_ref_cyc = cyc - floor(cyc);
        d->lo_ref_idx = idx;
        d->fs_cal = fs_new;
    }
    atomic_store(&d->st.fs_est, (float)fs_new);
    atomic_store(&d->st.line_us, (float)(d->H / d->cfg.fs_nom * 1e6));
}

static void on_vsync(decoder_t *d, double vs, bool synthetic)
{
    double rel = (vs - d->cur_h) / d->H;
    double fr = rel - floor(rel);

    if (!synthetic) {
        /* A real vertical sync starts on a line or half-line boundary. Noise
         * at low C/N makes vsync-like pulse runs anywhere (live C5 capture,
         * 2026-11-23: fields of 144 instead of 240 lines). */
        const bool aligned = fr < 0.12 || fr > 0.88 || fabs(fr - 0.5) < 0.12;
        if (!aligned) {
            atomic_fetch_add(&d->st.vsync_rejected, 1);
            return;
        }
        /* Vertical flywheel window: once the field timing is known, a vsync
         * counts only within +-4 lines of where it is due; the flywheel fills
         * a missing one. Three consistent vsyncs elsewhere re-acquire (a
         * camera or VTX that really changed timing). */
        if (d->vlock) {
            const double lpf = d->lpf_meas > 0 ? d->lpf_meas : d->sp->lines_per_field;
            const double dev = (vs - (d->last_vs + lpf * d->H)) / d->H;
            if (fabs(dev) > 4.0) {
                if (d->vcand_n > 0 && fabs((vs - d->vcand_last) / d->H - lpf) < 3.0)
                    d->vcand_n++;
                else
                    d->vcand_n = 1;
                d->vcand_last = vs;
                if (d->vcand_n < 3) {
                    atomic_fetch_add(&d->st.vsync_rejected, 1);
                    return;
                }
                d->vlock = false;
                LOGD("rx%c: vertical timing re-acquired", 'A' + d->id);
            } else if (fabs(dev) <= 1.5) {
                /* Confirmed: the flywheel's prediction is steadier than a
                 * detection whose first broad pulses were lost in noise. */
                vs = d->last_vs + lpf * d->H;
            }
            d->vcand_n = 0;
        }
    }
    rel = (vs - d->cur_h) / d->H;
    fr = rel - floor(rel);
    const int parity = (fr > 0.25 && fr < 0.75) ? 1 : 0;
    const double k0 = d->cur_h + ceil(rel - 0.25) * d->H;

    /* With a crystal-accurate capture clock (25.000 MS/s measured on a Zero
     * 2W) the line period alone tells the standard: 63.556 us vs 64.0 us. */
    const double line_us = d->H / d->cfg.fs_nom * 1e6;
    if (fabs(line_us - 63.556) < 0.12) {
        d->votes_ntsc++;
        d->votes_pal = 0;
    } else if (fabs(line_us - 64.0) < 0.12) {
        d->votes_pal++;
        d->votes_ntsc = 0;
    }
    if (!synthetic && d->have_vs) {
        double lines = (vs - d->last_vs) / d->H;
        if (fabs(lines - 312.5) < 2.0) {
            d->votes_pal++;
            d->votes_ntsc = 0;
        } else if (fabs(lines - 262.5) < 2.0) {
            d->votes_ntsc++;
            d->votes_pal = 0;
        }
        /* Only a field length close to the standard's counts (interlaced
         * 312.5/262.5, progressive cameras one half-line less or more). */
        if (fabs(lines - d->sp->lines_per_field) < 3.0) {
            d->vlock = true;
            d->lpf_meas = lines;
        }
    }
    if (g_verbose >= 2)
        fprintf(stderr, "VS rx%c %s dlines=%.2f lpf=%.2f H=%.3f vlock=%d cur_h-vs=%.1f lines coast=%d\n", 'A' + d->id,
                synthetic ? "fly" : "real", d->have_vs ? (vs - d->last_vs) / d->H : -1.0, d->lpf_meas, d->H, d->vlock,
                (d->cur_h - vs) / d->H, d->coast);
    if (synthetic)
        atomic_fetch_add(&d->st.vsync_flywheel, 1);
    else
        atomic_fetch_add(&d->st.vsyncs, 1);
    d->have_vs = true;
    d->last_vs = vs;

    emit_field(d, (uint64_t)vs);

    if (d->cfg.std_force == STD_UNKNOWN) {
        video_std_t want = d->votes_pal >= 2 ? STD_PAL : d->votes_ntsc >= 2 ? STD_NTSC : STD_UNKNOWN;
        if (want != STD_UNKNOWN) {
            if (want != d->std || !d->std_ok)
                LOGI("rx%c: %s detected", 'A' + d->id, std_name(want));
            if (want != d->std)
                set_std(d, want, vs);
            d->std_ok = true;
        }
    }
    update_fs_cal(d, vs);
    if (d->std_ok) {
        start_field(d, vs, parity, k0, synthetic);
    } else {
        d->have_k0 = false;   /* wait until the line count per field confirms the standard */
    }
}

static void lose_lock(decoder_t *d)
{
    emit_field(d, (uint64_t)d->cur_h);
    d->state = ST_ACQ;
    d->acq_n = 0;
    d->coast = 0;
    d->have_k0 = false;
    d->have_vs = false;
    d->vlock = false;
    d->vcand_n = 0;
    d->broad_run = 0;
    d->R_ok = false;
    d->pb_ok = false;
    atomic_store(&d->st.locked, false);
    atomic_fetch_add(&d->st.lock_losses, 1);
    LOGD("rx%c: lock lost", 'A' + d->id);
}

/* ------------------------------------------------------------ slicer */

static double merge_gap(const decoder_t *d) { return 0.5 * spu_of(d); }

static void emit_pulse(decoder_t *d, pulse_t p)
{
    if ((p.rise - p.fall) / spu_of(d) < 1.0)
        return;   /* too short: noise */
    pq_push(d, p);
}

static void got_pulse(decoder_t *d, double fall, double rise)
{
    if (d->have_pend) {
        if (fall - d->pend.rise < merge_gap(d)) {
            d->pend.rise = rise;   /* a noise spike split one pulse */
            return;
        }
        emit_pulse(d, d->pend);
    }
    d->pend.fall = fall;
    d->pend.rise = rise;
    d->have_pend = true;
}

static float hysteresis(const decoder_t *d)
{
    return d->lvl_ok ? fmaxf(0.75f, 0.08f * (d->blank - d->tip)) : 1.0f;
}

/* Sub-sample position where the box sum crosses T, searching back from
 * index i (whose sum s already lies beyond T on the new side). */
static double find_cross(const uint8_t *hb, size_t i, int32_t s, int nb, int32_t T, bool falling)
{
    size_t k = i;
    int32_t sk = s, snext = s;
    const size_t stop = i > (size_t)(4 * nb) ? i - (size_t)(4 * nb) : 0;
    while (k > stop && k > (size_t)nb && (falling ? sk < T : sk > T)) {
        snext = sk;
        sk = sk - hb[k] + hb[k - nb];   /* sum ending at k-1 */
        k--;
    }
    if (falling ? sk < T : sk > T)
        return (double)i;               /* no crossing within reach */
    if (sk == snext)
        return (double)k;
    return (double)k + (double)(sk - T) / (double)(sk - snext);
}

/* Box-filtered Schmitt trigger. Tight scan loops; the exact crossing is
 * only worked out (backwards) when a transition is found. */
static void slice(decoder_t *d)
{
    const uint64_t end = d->base + d->len;
    const int nb = d->nb;
    const int32_t T = (int32_t)lrintf(d->thr * nb);
    const int32_t Hy = (int32_t)lrintf(hysteresis(d) * nb);
    const int32_t TL = T - Hy, TH = T + Hy;
    const double corr = 0.5 * (nb - 1);
    const uint8_t *hb = d->hb;
    const size_t iend = d->len;
    size_t i = (size_t)(d->scan - d->base);

    if (!d->sum_ok) {
        if (iend < (size_t)nb) {
            d->scan = end;
            return;
        }
        if (i < (size_t)nb - 1)
            i = (size_t)nb - 1;
        int32_t s0 = 0;
        for (int k = 0; k < nb; k++)
            s0 += hb[i - (size_t)k];
        d->sum = s0;
        d->sum_ok = true;
        d->low = false;
        i++;
    }
    int32_t s = d->sum;   /* box sum ending at i-1 */
    while (i < iend) {
        if (!d->low) {
            for (; i < iend; i++) {
                s += (int32_t)hb[i] - (int32_t)hb[i - (size_t)nb];
                if (s < TL)
                    break;
            }
            if (i == iend)
                break;
            d->cur_fall = (double)d->base + find_cross(hb, i, s, nb, T, true) - corr;
            d->low = true;
            i++;
        } else {
            for (; i < iend; i++) {
                s += (int32_t)hb[i] - (int32_t)hb[i - (size_t)nb];
                if (s > TH)
                    break;
            }
            if (i == iend)
                break;
            double rise = (double)d->base + find_cross(hb, i, s, nb, T, false) - corr;
            d->low = false;
            got_pulse(d, d->cur_fall, rise);
            i++;
        }
    }
    d->sum = s;
    d->scan = end;
    if (d->have_pend && !d->low && (double)end - corr > d->pend.rise + merge_gap(d)) {
        emit_pulse(d, d->pend);
        d->have_pend = false;
    }
}

/* All pulses with fall < horizon have been emitted. */
static double horizon(const decoder_t *d)
{
    double h = (double)d->scan - 0.5 * (d->nb - 1) - 1.0;
    if (d->low && d->cur_fall < h)
        h = d->cur_fall;
    if (d->have_pend && d->pend.fall < h)
        h = d->pend.fall;
    return h;
}

static void hist_threshold(decoder_t *d)
{
    float tot = 0;
    for (int i = 0; i <= d->code_max; i++)
        tot += d->hist[i];
    if (tot < 100)
        return;
    float acc = 0;
    int p1 = -1, p50 = -1;
    for (int i = 0; i <= d->code_max; i++) {
        acc += d->hist[i];
        if (p1 < 0 && acc >= 0.01f * tot)
            p1 = i;
        if (p50 < 0 && acc >= 0.5f * tot)
            p50 = i;
    }
    float thr = p1 + 0.45f * (p50 - p1);
    const float min_step = 2.0f * (float)(d->code_max + 1) / 64.0f;
    if (thr < p1 + min_step)
        thr = p1 + min_step;
    d->thr = thr;
}

/* ------------------------------------------------------------ colour/active */

/* Worker side: resample, demodulate and write one line of the field. */
static void run_line_job(const decoder_t *d, const line_job_t *jb, line_work_t *w)
{
    const uint8_t *raw = jb->raw;
    const int nseg = jb->nseg;
    if (jb->hq) {
        for (int k = 0; k < nseg; k++)
            w->fl[k] = ((float)raw[k + 1] - jb->bl) * jb->gain;
        for (int j = 0; j < GRID_N; j++) {
            const float x = jb->x0 + (float)j * jb->dx;
            const int i0 = (int)x;
            const int p = (int)((x - (float)i0) * RS_PH + 0.5f);
            const float *src = w->fl + i0 - (RS_TAPS / 2 - 1);
            const float *h = d->rs[p];
            float acc = 0;
            for (int k = 0; k < RS_TAPS; k++)
                acc += h[k] * src[k];
            w->v[j] = acc;
        }
    } else if (d->clean) {
        for (int k = 0; k < nseg; k++)
            w->fl[k] = ((float)raw[k + 1] - jb->bl) * jb->gain;
        for (int j = 0; j < GRID_N; j++) {
            const float x = jb->x0 + (float)j * jb->dx;
            const int i0 = (int)x;
            const float *src = w->fl + i0 - (SR_TAPS / 2 - 1);
            const float *h = d->sr[(int)((x - (float)i0) * RS_PH + 0.5f)];
            float acc = 0;
            for (int k = 0; k < SR_TAPS; k++)
                acc += h[k] * src[k];
            w->v[j] = acc;
        }
    } else {
        /* 3-tap prefilter with its zero on the C5 hold image (20 - fsc MHz
         * folded to fs - 15.57 MHz), then linear interpolation. */
        const float b0 = d->pf_b * jb->gain, a0 = d->pf_a * jb->gain, off = jb->bl * jb->gain;
        for (int k = 0; k < nseg; k++)
            w->fl[k] = b0 * ((float)raw[k] + (float)raw[k + 2]) + a0 * (float)raw[k + 1] - off;
        for (int j = 0; j < GRID_N; j++) {
            const float x = jb->x0 + (float)j * jb->dx;
            const int i0 = (int)x;
            const float f = x - (float)i0;
            w->v[j] = w->fl[i0] + f * (w->fl[i0 + 1] - w->fl[i0]);
        }
    }

    uint8_t *Y = jb->fld->y[jb->row];
    const float ys = jb->ys, yo = jb->yo;
    if (!jb->sub) {
        for (int j = 0; j < OUT_W; j++)
            Y[j] = u8f(yo + ys * w->v[j + GRID_PRE]);
        memset(jb->fld->cb[jb->row], 128, OUT_CW);
        memset(jb->fld->cr[jb->row], 128, OUT_CW);
        jb->fld->q[jb->row] = jb->q;
        return;
    }

    /* Local oscillator: the per-standard table rotated by the phase at grid 0. */
    for (int j = 0; j < GRID_N; j++) {
        const float c = jb->c0 * d->lo_tc[j] - jb->s0 * d->lo_ts[j];
        const float sn = jb->s0 * d->lo_tc[j] + jb->c0 * d->lo_ts[j];
        w->lc[j] = c;
        w->ls[j] = sn;
        w->zr[j] = 2.0f * w->v[j] * c;
        w->zi[j] = -2.0f * w->v[j] * sn;
    }

    /* Chroma low-pass: two cascaded centred running sums of CL pixels (a
     * triangle, -6 dB near 0.65 MHz, nulls the 2*fsc mixing product).
     * Narrower than a broadcast decoder; with the C5's FM noise the lower
     * chroma bandwidth is also less colour speckle. A few adds per pixel. */
    const int h = CL / 2, g0 = GRID_PRE - 2 * h, g1 = GRID_PRE + OUT_W + 2 * h;
    float pr = 0, pi = 0;
    w->Pr[g0 - h] = w->Pi[g0 - h] = 0;
    for (int g = g0 - h; g < g1 + h; g++) {
        pr += w->zr[g];
        pi += w->zi[g];
        w->Pr[g + 1] = pr;
        w->Pi[g + 1] = pi;
    }
    float qr = 0, qi = 0;
    w->Qr[g0] = w->Qi[g0] = 0;
    for (int g = g0; g < g1; g++) {
        qr += w->Pr[g + h + 1] - w->Pr[g - h];
        qi += w->Pi[g + h + 1] - w->Pi[g - h];
        w->Qr[g + 1] = qr;
        w->Qi[g + 1] = qi;
    }
    const float inv = 1.0f / (float)(CL * CL);
    for (int j = 0; j < OUT_W; j++) {
        const int g = j + GRID_PRE;
        w->Cr[j] = (w->Qr[g + h + 1] - w->Qr[g - h]) * inv;
        w->Ci[j] = (w->Qi[g + h + 1] - w->Qi[g - h]) * inv;
    }

    /* Luma by chroma remodulation/subtraction; U/V from the burst-locked
     * reference (rot = j * conj(R)), PAL V-switch, ACC. */
    const float acc = jb->acc, rre = jb->rot_re, rim = jb->rot_im, sw = jb->sw;
    for (int j = 0; j < OUT_W; j++) {
        const int g = j + GRID_PRE;
        const float cr = w->Cr[j], ci = w->Ci[j];
        Y[j] = u8f(yo + ys * (w->v[g] - (cr * w->lc[g] - ci * w->ls[g])));
        if (jb->colour) {
            w->U[j] = -(cr * rim + ci * rre) * acc;
            w->V[j] = (cr * rre - ci * rim) * acc * sw;
        }
    }
    uint8_t *CB = jb->fld->cb[jb->row], *CR = jb->fld->cr[jb->row];
    if (jb->colour) {
        for (int m = 0; m < OUT_CW; m++) {
            CB[m] = u8f(128.0f + 128.45f * (w->U[2 * m] + w->U[2 * m + 1]));
            CR[m] = u8f(128.0f + 91.1f * (w->V[2 * m] + w->V[2 * m + 1]));
        }
    } else {
        memset(CB, 128, OUT_CW);
        memset(CR, 128, OUT_CW);
    }
    jb->fld->q[jb->row] = jb->q;
}

static void *line_worker(void *arg)
{
    decoder_t *d = arg;
    line_work_t w;
    memset(&w, 0, sizeof(w));
    w.fl = malloc(sizeof(float) * (size_t)d->fl_cap);
    for (;;) {
        pthread_mutex_lock(&d->q_mu);
        while (d->q_next == d->q_ready && !d->q_stop)
            pthread_cond_wait(&d->q_job, &d->q_mu);
        if (d->q_next == d->q_ready && d->q_stop) {
            pthread_mutex_unlock(&d->q_mu);
            break;
        }
        const unsigned slot = d->q_next++ % JOBQ;
        pthread_mutex_unlock(&d->q_mu);
        run_line_job(d, &d->jobs[slot], &w);
        pthread_mutex_lock(&d->q_mu);
        d->job_state[slot] = 0;
        d->pending--;
        pthread_cond_broadcast(&d->q_done);
        pthread_mutex_unlock(&d->q_mu);
    }
    free(w.fl);
    return NULL;
}

/* Wait until every queued line has been written (before a field leaves). */
static void wait_lines(decoder_t *d)
{
    if (!d->nworkers)
        return;
    pthread_mutex_lock(&d->q_mu);
    while (d->pending)
        pthread_cond_wait(&d->q_done, &d->q_mu);
    pthread_mutex_unlock(&d->q_mu);
}

/* Prefiltered input sample at history index i (luma units, blank removed):
 * the same value the worker interpolates. */
static inline float pf_at(const decoder_t *d, size_t i, float gain, float bl)
{
    return d->pf_b * gain * ((float)d->hb[i - 1] + (float)d->hb[i + 1]) + d->pf_a * gain * (float)d->hb[i] - bl * gain;
}

/* Decoder side: fix this line's parameters (sequential colour state from
 * the burst), copy its raw samples and hand it to a worker. */
static void decode_active(decoder_t *d, double x0, double spu, int row, float q)
{
    const std_params_t *sp = d->sp;
    const float amp = fmaxf(d->blank - d->tip, 2.0f);
    const float gain = 1.0f / (amp * (float)sp->sync_ratio);
    const float bl = d->blank;
    const double dx = spu * 1e6 / PX_RATE;
    const double xs = x0 + (sp->h_active_us + d->cfg.hshift_us) * spu;
    const double xa = xs - GRID_PRE * dx;
    const double xb = xs + (OUT_W + GRID_POST) * dx;
    if (xa - RS_TAPS < 0 || xb + RS_TAPS >= (double)d->len)
        return;
    const long seg0 = (long)xa - (RS_TAPS / 2 - 1);
    const long seg1 = (long)xb + RS_TAPS / 2 + 1;
    if (seg1 - seg0 > d->fl_cap || (size_t)(seg1 - seg0 + 2) > d->raw_cap)
        return;

    /* Subcarrier phase at grid point 0, continuous in absolute time. */
    const double absx = (double)d->base + xa;
    double cyc = d->lo_ref_cyc + sp->fsc_hz * (absx - d->lo_ref_idx) / d->fs_cal;
    cyc -= floor(cyc);
    const float c0 = (float)cos(2.0 * M_PI * cyc), s0 = (float)sin(2.0 * M_PI * cyc);

    /* Burst, on its few grid points only. */
    int jb0 = GRID_PRE + (int)ceil((sp->burst_start_us - sp->h_active_us - d->cfg.hshift_us) * 13.5);
    int jb1 = GRID_PRE + (int)floor((sp->burst_end_us - sp->h_active_us - d->cfg.hshift_us) * 13.5);
    jb0 = clampi(jb0, 0, GRID_N - 1);
    jb1 = clampi(jb1, 0, GRID_N - 1);
    float br = 0, bi = 0;
    int nbst = 0;
    for (int j = jb0; j <= jb1; j++) {
        const double x = xa + j * dx;
        const size_t i0 = (size_t)x;
        const float f = (float)(x - (double)i0);
        float v = 0;
        if (d->clean) {
            const float *h = d->sr[(int)(f * RS_PH + 0.5f)];
            for (int k = 0; k < SR_TAPS; k++)
                v += h[k] * ((float)d->hb[i0 + (size_t)k - (SR_TAPS / 2 - 1)] - bl) * gain;
        } else {
            const float a = pf_at(d, i0, gain, bl), b = pf_at(d, i0 + 1, gain, bl);
            v = a + f * (b - a);
        }
        const float c = c0 * d->lo_tc[j] - s0 * d->lo_ts[j];
        const float sn = s0 * d->lo_tc[j] + c0 * d->lo_ts[j];
        br += 2.0f * v * c;
        bi += -2.0f * v * sn;
        nbst++;
    }
    float bm = 0;
    if (nbst > 0) {
        br /= nbst;
        bi /= nbst;
        bm = hypotf(br, bi);
    }
    if (bm > 0.02f) {
        float ur = br / bm, ui = bi / bm;
        if (sp->pal) {
            /* Consecutive PAL bursts swing +-45 deg around the reference. */
            if (d->pb_ok && d->pb_row == row - 1) {
                float sr = ur + d->pbr, si = ui + d->pbi;
                float m = hypotf(sr, si);
                if (m > 0.7f) {
                    sr /= m;
                    si /= m;
                    if (!d->R_ok) {
                        d->Rr = sr;
                        d->Ri = si;
                        d->R_ok = true;
                    } else {
                        d->Rr += 0.25f * (sr - d->Rr);
                        d->Ri += 0.25f * (si - d->Ri);
                        float n = hypotf(d->Rr, d->Ri);
                        d->Rr /= n;
                        d->Ri /= n;
                    }
                }
            }
            if (d->R_ok)
                d->pal_s = (ui * d->Rr - ur * d->Ri) < 0 ? 1 : -1;
        } else {
            if (!d->R_ok) {
                d->Rr = ur;
                d->Ri = ui;
                d->R_ok = true;
            } else {
                d->Rr += 0.25f * (ur - d->Rr);
                d->Ri += 0.25f * (ui - d->Ri);
                float n = hypotf(d->Rr, d->Ri);
                d->Rr /= n;
                d->Ri /= n;
            }
        }
        d->pbr = ur;
        d->pbi = ui;
        d->pb_ok = true;
        d->pb_row = row;
        d->Ab = d->Ab <= 0 ? bm : d->Ab + 0.1f * (bm - d->Ab);
    } else {
        d->pb_ok = false;
    }
    atomic_store(&d->st.burst, d->Ab / (float)sp->burst_nom);

    /* Colour killer at 15% of nominal burst: the C5 hold, prefilter and
     * interpolation already cost ~45% of subcarrier amplitude, and ACC
     * restores chroma gain from the measured burst. */
    line_job_t jb;
    jb.fld = d->fld;
    jb.row = row;
    jb.q = q;
    jb.nseg = (int)(seg1 - seg0);
    jb.x0 = (float)(xa - (double)seg0);
    jb.dx = (float)dx;
    jb.bl = bl;
    jb.gain = gain;
    jb.c0 = c0;
    jb.s0 = s0;
    jb.sub = d->R_ok && d->Ab > 0.15f * (float)sp->burst_nom;
    jb.colour = jb.sub && !d->cfg.mono;
    jb.hq = d->cfg.hq_resample;
    jb.acc = clampf((float)sp->burst_nom / fmaxf(d->Ab, 1e-3f), 0.25f, 6.0f) * d->cfg.saturation;
    jb.rot_re = d->Ri;   /* j * conj(R) */
    jb.rot_im = d->Rr;
    jb.sw = sp->pal ? (float)d->pal_s : 1.0f;
    jb.ys = 219.0f * d->cfg.contrast;
    jb.yo = 16.0f + d->cfg.brightness;
    const uint8_t *src = d->hb + seg0 - 1;
    const size_t nraw = (size_t)(seg1 - seg0 + 2);

    if (!d->nworkers) {
        jb.raw = (uint8_t *)src;
        run_line_job(d, &jb, &d->main_work);
        return;
    }
    pthread_mutex_lock(&d->q_mu);
    const unsigned slot = d->q_tail % JOBQ;
    while (d->job_state[slot])
        pthread_cond_wait(&d->q_done, &d->q_mu);
    d->job_state[slot] = 1;
    pthread_mutex_unlock(&d->q_mu);
    jb.raw = d->job_raw + (size_t)slot * d->raw_cap;
    memcpy(jb.raw, src, nraw);
    d->jobs[slot] = jb;
    pthread_mutex_lock(&d->q_mu);
    d->q_tail++;
    d->q_ready = d->q_tail;
    d->pending++;
    pthread_cond_signal(&d->q_job);
    pthread_mutex_unlock(&d->q_mu);
}

static void process_line(decoder_t *d)
{
    const std_params_t *sp = d->sp;
    const double spu = spu_of(d);
    const double t0 = d->cur_h;
    const double x0 = t0 - (double)d->base;
    atomic_fetch_add(&d->st.lines, 1);

    bool gap = d->gap_hi > d->gap_lo && t0 - 2.0 * spu < (double)d->gap_hi && t0 + d->H > (double)d->gap_lo;
    double wus = d->cur_width / spu;
    bool hs = d->cur_matched && wus >= 3.5 && wus <= 7.5;

    float fpm, fpv, bpm, bpv;
    int nfp = mean_var_pf(d, x0 + sp->fp_start_us * spu, x0 + sp->fp_end_us * spu, &fpm, &fpv);
    int nbp = mean_var_pf(d, x0 + sp->bp_start_us * spu, x0 + sp->bp_end_us * spu, &bpm, &bpv);
    if (hs && !gap && nfp > 4 && nbp > 4) {
        float tm, tv;
        int nt = mean_var(d, x0 + 1.0 * spu, x0 + 3.7 * spu, &tm, &tv);
        float bl = 0.5f * (fpm + bpm);
        float amp = bl - tm;
        if (nt > 4 && amp > 3.0f * (float)(d->code_max + 1) / 64.0f && amp < (float)d->code_max) {
            if (!d->lvl_ok) {
                d->tip = tm;
                d->blank = bl;
                d->lvl_ok = true;
            } else {
                d->tip += 0.05f * (tm - d->tip);
                d->blank += 0.05f * (bl - d->blank);
            }
            d->thr = 0.5f * (d->tip + d->blank);
            atomic_store(&d->st.sync_amp, d->blank - d->tip);
            atomic_store(&d->st.blank, d->blank);
        }
    }

    float amp = fmaxf(d->blank - d->tip, 2.0f);
    float sig = sqrtf(0.5f * (fpv + bpv));
    if (sig < 0.3f)
        sig = 0.3f;
    float q = 20.0f * log10f(amp / sig);
    if (!d->cur_matched)
        q -= 10.0f;
    if (gap)
        q = Q_INVALID;
    else
        atomic_store(&d->st.snr_db, 0.98f * atomic_load(&d->st.snr_db) + 0.02f * q);
    if (d->nll < DECODER_LINES)
        d->ll[d->nll++] = (decoder_line_t){t0, gap ? 4.0f : powf(10.0f, -0.1f * q)};

    if (!d->have_k0 || !d->fld || gap)
        return;
    int k = (int)lrint((t0 - d->k0) / d->H);
    int row = k - sp->first_active_k + d->cfg.voffset;
    if (row < 0 || row >= sp->active_lines)
        return;
    decode_active(d, x0, spu, row, q);
}

/* ------------------------------------------------------------ PLL engine */

static bool acquire(decoder_t *d)
{
    const double spu0 = d->fs_cal / 1e6;
    hist_threshold(d);
    while (d->pq_n) {
        pulse_t p = pq_pop(d);
        double w = (p.rise - p.fall) / spu0;
        if (w < 3.5 || w > 7.5)
            continue;
        if (d->acq_n == ACQ_N) {
            memmove(d->acq, d->acq + 1, sizeof(double) * (ACQ_N - 1));
            d->acq_n--;
        }
        d->acq[d->acq_n++] = p.fall;
        if (d->acq_n < ACQ_N)
            continue;
        double iv[ACQ_N - 1], srt[ACQ_N - 1];
        for (int i = 0; i < ACQ_N - 1; i++)
            srt[i] = iv[i] = d->acq[i + 1] - d->acq[i];
        for (int i = 1; i < ACQ_N - 1; i++)   /* insertion sort */
            for (int j = i; j > 0 && srt[j - 1] > srt[j]; j--) {
                double t = srt[j];
                srt[j] = srt[j - 1];
                srt[j - 1] = t;
            }
        double m = srt[(ACQ_N - 1) / 2];
        bool ok = m > 55.0 * spu0 && m < 72.0 * spu0;
        for (int i = 0; i < ACQ_N - 1 && ok; i++)
            if (fabs(iv[i] - m) > 0.01 * m)
                ok = false;
        if (!ok)
            continue;
        d->H = m;
        d->cur_h = d->acq[ACQ_N - 1];
        d->cur_matched = true;
        d->cur_width = p.rise - p.fall;
        d->state = ST_LOCK;
        d->coast = 0;
        d->have_unmatched = false;
        d->broad_run = 0;
        d->have_vs = false;
        d->have_k0 = false;
        atomic_store(&d->st.locked, true);
        LOGD("rx%c: locked, line %.3f us", 'A' + d->id, m / spu0);
        return true;
    }
    return false;
}

static void classify(decoder_t *d, const pulse_t *p, bool matched)
{
    double w = (p->rise - p->fall) / spu_of(d);
    /* Broad (vertical sync) pulses last half a line minus 4.7 us: ~27 us.
     * The window is tight because a noisy dark picture near the sync
     * threshold produces long pulses of any width (real C5 capture,
     * 2026-11-23: 15 000 "broad" pulses in 3 s with a 10-31 us window). */
    if (w >= 22.0 && w <= 31.0) atomic_fetch_add(&d->st.pulses_broad, 1);
    else if (w >= 3.5 && w <= 7.5) atomic_fetch_add(&d->st.pulses_hsync, 1);
    else if (w >= 1.5 && w < 3.5) atomic_fetch_add(&d->st.pulses_eq, 1);
    else atomic_fetch_add(&d->st.pulses_other, 1);
    if (w >= 22.0 && w <= 31.0) {
        /* A vertical sync is a run of broad pulses half a line apart; three
         * of the five or six are required. */
        const double dt = p->fall - d->last_broad;
        if (d->broad_run > 0 && fabs(dt - 0.5 * d->H) < 1.5 * spu_of(d)) {
            d->broad_run++;
        } else {
            d->broad_run = 1;
            d->first_broad = p->fall;
        }
        d->last_broad = p->fall;
        if (d->broad_run == 3)
            on_vsync(d, d->first_broad, false);
    } else if (w >= 3.5 && w <= 7.5 && !matched) {
        d->have_unmatched = true;
        d->last_unmatched = p->fall;
    }
}

static void engine(decoder_t *d)
{
    for (;;) {
        if (d->state == ST_ACQ) {
            if (!acquire(d))
                return;
            continue;
        }
        const double spu = spu_of(d);
        const double pred = d->cur_h + d->H;
        if ((double)(d->base + d->len) < pred + 2.0 * spu)
            return;   /* need the whole line plus margin */
        if (d->resync_pending && pred + 3.0 * spu >= (double)d->resync_pos) {
            if (d->coast < 2)
                d->coast = 2;
            d->resync_pending = false;
            d->R_ok = false;
            d->pb_ok = false;
            atomic_fetch_add(&d->st.resyncs, 1);
        }
        const double win = (d->coast >= 2 ? 3.0 : 1.0) * spu;

        while (d->pq_n && pq_front(d)->fall < pred - win) {
            pulse_t p = pq_pop(d);
            classify(d, &p, false);
        }
        bool matched = false;
        pulse_t mp = {0, 0};
        if (d->pq_n && pq_front(d)->fall <= pred + win) {
            mp = pq_pop(d);
            matched = true;
        } else if (horizon(d) <= pred + win) {
            return;   /* a pulse may still appear in the window */
        }

        double new_h;
        if (matched) {
            double err = mp.fall - pred;
            if (d->coast >= 2) {
                new_h = mp.fall;
            } else {
                new_h = pred + d->cfg.pll_alpha * err;
                d->H += d->cfg.pll_beta * err;
                double lo = 0.97 * d->H_nom, hi = 1.03 * d->H_nom;
                if (d->H < lo) d->H = lo;
                if (d->H > hi) d->H = hi;
            }
            d->coast = 0;
            d->have_unmatched = false;
            classify(d, &mp, true);
        } else {
            new_h = pred;
            d->coast++;
            atomic_fetch_add(&d->st.coasted, 1);
            if (d->coast >= 2 && d->have_unmatched && pred - d->last_unmatched < 1.5 * d->H) {
                double n = ceil((d->cur_h + 0.5 * d->H - d->last_unmatched) / d->H);
                new_h = d->last_unmatched + n * d->H;
                d->have_unmatched = false;
                d->coast = 2;
                atomic_fetch_add(&d->st.resyncs, 1);
            }
        }

        process_line(d);
        d->cur_h = new_h;
        d->cur_matched = matched;
        d->cur_width = matched ? mp.rise - mp.fall : 0;

        if (d->have_vs && d->vlock) {   /* flywheel only with confirmed vertical timing */
            double lpf = d->lpf_meas > 0 ? d->lpf_meas : d->sp->lines_per_field;
            if (d->cur_h > d->last_vs + (lpf + 4.5) * d->H)   /* just past the vsync window */
                on_vsync(d, d->last_vs + lpf * d->H, true);
        }
        if (d->coast > 50) {
            lose_lock(d);
            return;
        }
    }
}

/* ------------------------------------------------------------ public */

static double bessel_i0(double x)
{
    double s = 1.0, t = 1.0;
    for (int k = 1; k < 30; k++) {
        t *= (x / (2.0 * k)) * (x / (2.0 * k));
        s += t;
    }
    return s;
}

decoder_t *decoder_new(const decoder_cfg_t *cfg, int rx_id, field_sink_t sink)
{
    decoder_t *d = calloc(1, sizeof(*d));
    if (!d)
        return NULL;
    d->cfg = *cfg;
    d->id = rx_id;
    d->sink = sink;
    d->cap = 1u << 20;
    d->hb = malloc(d->cap);
    /* one line's resampling span: GRID_N pixels at fs/13.5 MHz input samples each */
    d->fl_cap = (long)(GRID_N * (cfg->fs_nom / PX_RATE) * 1.1) + 4 * RS_TAPS;
    d->main_work.fl = malloc(sizeof(float) * (size_t)d->fl_cap);
    d->raw_cap = (size_t)d->fl_cap + 4;
    d->jobs = calloc(JOBQ, sizeof(line_job_t));
    d->job_raw = malloc(JOBQ * d->raw_cap);
    if (!d->hb || !d->main_work.fl || !d->jobs || !d->job_raw) {
        free(d->hb);
        free(d->main_work.fl);
        free(d->jobs);
        free(d->job_raw);
        free(d);
        return NULL;
    }
    pthread_mutex_init(&d->q_mu, NULL);
    d->tb_a = 1.0;
    d->code_max = cfg->code_max > 0 ? cfg->code_max : 63;
    pthread_cond_init(&d->q_job, NULL);
    pthread_cond_init(&d->q_done, NULL);
    /* Slicer box: 0.6 us. The C5's FM-demodulated sync can be only ~8 codes
     * deep with ~4 codes of noise; the narrowest pulse (equalising) is 2.35 us. */
    d->nb = (int)lrint(0.6e-6 * cfg->fs_nom);
    if (d->nb < 3)
        d->nb = 3;
    d->fs_cal = cfg->fs_nom;
    set_std(d, cfg->std_force == STD_UNKNOWN ? STD_PAL : cfg->std_force, 0);
    d->std_ok = cfg->std_force != STD_UNKNOWN;
    d->H = d->H_nom;
    {
        const float k = (float)(d->code_max + 1) / 64.0f;   /* C5 nominal: tip 0, blank 20 */
        d->tip = 0;
        d->blank = 20.0f * k;
        d->thr = 10.0f * k;
    }
    d->pb_row = -10;
    d->pal_s = 1;

    const double fc = 1.2e6 / PX_RATE;
    double sum = 0;
    for (int t = 0; t < LPF_TAPS; t++) {
        int n = t - LPF_HALF;
        double h = n == 0 ? 2.0 * fc : sin(2.0 * M_PI * fc * n) / (M_PI * n);
        double w = 0.54 - 0.46 * cos(2.0 * M_PI * t / (LPF_TAPS - 1));
        d->lpf[t] = (float)(h * w);
        sum += h * w;
    }
    for (int t = 0; t < LPF_TAPS; t++)
        d->lpf[t] = (float)(d->lpf[t] / sum);

    /* Fast prefilter [b a b]: zero at the folded C5 hold image, unity DC. */
    {
        double fz = fabs(cfg->fs_nom - (20e6 - 4.43e6));
        if (fz > cfg->fs_nom / 2)
            fz = cfg->fs_nom - fz;
        double cz = cos(2.0 * M_PI * fz / cfg->fs_nom);
        double a = -2.0 * cz;   /* with b = 1 */
        if (a < 0.5)
            a = 0.5;            /* zero not placeable: fall back to a mild lowpass */
        d->pf_a = (float)(a / (a + 2.0));
        d->pf_b = (float)(1.0 / (a + 2.0));
    }

    /* Resampler: Kaiser-windowed sinc, ~6.5 MHz cutoff (keeps chroma's
     * upper sideband, rejects the 20 MS/s hold images near 9.4 MHz). */
    double rfc = 6.5e6 / cfg->fs_nom;
    if (rfc > 0.45)
        rfc = 0.45;
    const double beta = 6.0, half = RS_TAPS / 2.0;
    for (int p = 0; p <= RS_PH; p++) {
        double frac = (double)p / RS_PH, acc = 0;
        for (int k = 0; k < RS_TAPS; k++) {
            double x = (double)(k - (RS_TAPS / 2 - 1)) - frac;
            double sx = 2.0 * rfc * x;
            double h = 2.0 * rfc * (fabs(sx) < 1e-9 ? 1.0 : sin(M_PI * sx) / (M_PI * sx));
            double r = x / half;
            double w = fabs(r) < 1.0 ? bessel_i0(beta * sqrt(1.0 - r * r)) / bessel_i0(beta) : 0.0;
            d->rs[p][k] = (float)(h * w);
            acc += h * w;
        }
        for (int k = 0; k < RS_TAPS; k++)
            d->rs[p][k] = (float)(d->rs[p][k] / acc);
    }
    d->clean = cfg->fs_nom < 22e6;
    for (int p = 0; p <= RS_PH; p++) {
        const double frac = (double)p / RS_PH, fcs = 6.0e6 / cfg->fs_nom;
        double acc = 0;
        for (int k = 0; k < SR_TAPS; k++) {
            const double x = (double)(k - (SR_TAPS / 2 - 1)) - frac, sx = 2.0 * fcs * x, r = x / (SR_TAPS / 2.0);
            double h = fabs(sx) < 1e-9 ? 1.0 : sin(M_PI * sx) / (M_PI * sx);
            h *= fabs(r) < 1.0 ? bessel_i0(3.5 * sqrt(1.0 - r * r)) / bessel_i0(3.5) : 0.0;
            d->sr[p][k] = (float)h;
            acc += h;
        }
        for (int k = 0; k < SR_TAPS; k++)
            d->sr[p][k] = (float)(d->sr[p][k] / acc);
    }
    atomic_store(&d->st.snr_db, 0.0f);
    int nw = cfg->workers < 0 ? 0 : cfg->workers > MAX_WORKERS ? MAX_WORKERS : cfg->workers;
    for (int k = 0; k < nw; k++)
        if (pthread_create(&d->workers[k], NULL, line_worker, d) == 0)
            d->nworkers++;
    return d;
}

void decoder_free(decoder_t *d)
{
    if (!d)
        return;
    if (d->nworkers) {
        pthread_mutex_lock(&d->q_mu);
        d->q_stop = true;
        pthread_cond_broadcast(&d->q_job);
        pthread_mutex_unlock(&d->q_mu);
        for (int k = 0; k < d->nworkers; k++)
            pthread_join(d->workers[k], NULL);
    }
    pthread_mutex_destroy(&d->q_mu);
    pthread_cond_destroy(&d->q_job);
    pthread_cond_destroy(&d->q_done);
    free(d->hb);
    free(d->main_work.fl);
    free(d->jobs);
    free(d->job_raw);
    free(d->lane);
    free(d);
}

decoder_stats_t *decoder_stats(decoder_t *d) { return &d->st; }

static void reset_stream(decoder_t *d, uint64_t start)
{
    if (d->state == ST_LOCK)
        lose_lock(d);
    d->base = start;
    d->len = 0;
    d->scan = start;
    d->sum_ok = false;
    d->low = false;
    d->have_pend = false;
    d->pq_n = 0;
    d->pq_head = 0;
    d->acq_n = 0;
}

static void feed(decoder_t *d, const uint16_t *s, const uint8_t *s8, uint32_t n, uint64_t start, bool discont)
{
    if (!d->started) {
        d->started = true;
        d->base = start;
        d->scan = start;
        d->lo_ref_idx = (double)start;
    }
    uint64_t end = d->base + d->len;
    if (start > end) {
        uint64_t gap = start - end;
        atomic_fetch_add(&d->st.gaps, 1);
        if ((double)gap < 8.0 * d->H) {
            ensure_cap(d, (size_t)gap);
            memset(d->hb + d->len, (int)lrintf(d->blank), (size_t)gap);
            d->len += (size_t)gap;
            d->gap_lo = end;
            d->gap_hi = start;
            d->resync_pending = true;
            d->resync_pos = end;
        } else {
            reset_stream(d, start);
        }
    } else if (start < end) {
        uint64_t skip = end - start;
        if (skip >= n)
            return;
        if (s) s += skip;
        if (s8) s8 += skip;
        n -= (uint32_t)skip;
        start = end;
    } else if (discont) {
        atomic_fetch_add(&d->st.gaps, 1);
        d->gap_lo = end;
        d->gap_hi = end + 1;
        d->resync_pending = true;
        d->resync_pos = end;
    }

    ensure_cap(d, n);
    uint8_t *o = d->hb + d->len;
    const int sh = d->cfg.lane ? 8 : 0;
    if (d->cfg.median3) {
        /* lane[0..1] = last two samples of the previous block */
        if (d->lane_cap < (size_t)n + 2) {
            d->lane_cap = (size_t)n + 2;
            d->lane = realloc(d->lane, d->lane_cap);
        }
        uint8_t *L = d->lane;
        L[0] = d->med_a;
        L[1] = d->med_b;
        if (s8)
            memcpy(L + 2, s8, n);
        else
            for (uint32_t i = 0; i < n; i++)
                L[i + 2] = (uint8_t)((s[i] >> sh) & 63);
        for (uint32_t i = 0; i < n; i++) {
            uint8_t a = L[i], b = L[i + 1], c = L[i + 2];
            uint8_t lo = a < b ? a : b, hi = a < b ? b : a;
            uint8_t m = hi < c ? hi : c;
            o[i] = lo > m ? lo : m;
        }
        d->med_a = L[n];
        d->med_b = L[n + 1];
    } else if (s8) {
        memcpy(o, s8, n);
    } else {
        for (uint32_t i = 0; i < n; i++)
            o[i] = (uint8_t)((s[i] >> sh) & 63);
    }
    if (d->state == ST_ACQ || !d->lvl_ok) {   /* initial threshold only */
        for (int i = 0; i <= d->code_max; i++)
            d->hist[i] *= 0.7f;
        for (uint32_t i = 0; i < n; i += 4)
            d->hist[o[i]] += 1.0f;
    }
    d->len += n;

    slice(d);
    engine(d);

    if (keep_from(d) - d->base > d->cap / 2)
        compact(d);
    atomic_store(&d->st.progress, (uint64_t)fmax(0.0, d->tb_a * (double)d->scan + d->tb_b));
}

void decoder_feed(decoder_t *d, const uint16_t *s, uint32_t n, uint64_t start, bool discont)
{
    feed(d, s, NULL, n, start, discont);
}

void decoder_feed_u8(decoder_t *d, const uint8_t *s, uint32_t n, uint64_t start, bool discont)
{
    feed(d, NULL, s, n, start, discont);
}

void decoder_get_timing(const decoder_t *d, decoder_timing_t *t)
{
    t->locked = d->state == ST_LOCK;
    t->vlock = d->vlock;
    t->H = d->H;
    t->line_pos = d->cur_h;
    t->last_vs = d->last_vs;
    t->lines_per_field = d->lpf_meas > 0 ? d->lpf_meas : d->sp->lines_per_field;
    t->blank = d->blank;
    t->amp = fmaxf(d->blank - d->tip, 1.0f);
    t->snr_db = atomic_load(&d->st.snr_db);
}

int decoder_take_lines(decoder_t *d, decoder_line_t out[DECODER_LINES])
{
    const int n = d->nll;
    memcpy(out, d->ll, sizeof(*out) * (size_t)n);
    d->nll = 0;
    return n;
}

void decoder_set_timebase(decoder_t *d, double scale, double offset)
{
    d->tb_a = scale;
    d->tb_b = offset;
}

void decoder_flush(decoder_t *d)
{
    emit_field(d, (uint64_t)d->cur_h);
}
