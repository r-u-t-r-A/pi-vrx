/* End-to-end tests on the synthetic C5 source (no hardware needed). */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"
#include "cvbs_synth.h"
#include "decoder.h"
#include "diversity.h"
#include "frame.h"
#include "pipeline.h"
#include "ring.h"
#include "source.h"

int g_verbose = 0;
atomic_bool g_running = true;
static bool g_flywheel = true;   /* decoder flywheel for the next run() */

typedef struct {
    frame_t last;
    uint64_t frames;
    int skip;                   /* frames ignored by the error metric (lock-in) */
    double se_y, se_c;          /* squared error sums over the colour bars */
    uint64_t n_y, n_c;
    double worst_y;             /* worst single-frame luma RMS error */
    double bar_sum[8];          /* per-bar luma error sums: the mean is bias (gain/level), the rest noise */
    uint64_t bar_n[8];
    uint64_t rows, good_rows;   /* bar rows, and those that show the bars in the right place */
} capture_t;

/* Picture error against the synthetic reference, in the (static) colour
 * bars, away from bar edges. */
static void add_error(capture_t *c, const frame_t *f)
{
    double fy = 0;
    uint64_t fn = 0;
    /* Rows that show the bars at the right level and position (scraps of
     * signal decoded in place); a shifted or noisy row fails. */
    for (int y = (int)(0.05 * f->h); y < (int)(0.55 * f->h); y++) {
        double se = 0;
        int n = 0;
        for (int i = 0; i < 8; i++) {
            double Ye, Ue, Ve;
            cvbs_synth_picture((i + 0.5) / 8.0, 0.3, 0, &Ye, &Ue, &Ve);
            for (int x = 90 * i + 25; x < 90 * i + 65; x += 2, n++) {
                const double e = f->y[(size_t)y * f->w + x] - (16 + 219 * Ye);
                se += e * e;
            }
        }
        c->rows++;
        c->good_rows += sqrt(se / n) < 12.0;
    }
    for (int i = 0; i < 8; i++) {
        double Ye, Ue, Ve;
        cvbs_synth_picture((i + 0.5) / 8.0, 0.3, 0, &Ye, &Ue, &Ve);
        const double ey = 16 + 219 * Ye, eu = 128 + 256.9 * Ue, ev = 128 + 182.2 * Ve;
        const int x0 = 90 * i + 25, x1 = 90 * i + 65;
        for (int y = (int)(0.05 * f->h); y < (int)(0.55 * f->h); y++)
            for (int x = x0; x < x1; x++) {
                const double e = f->y[(size_t)y * f->w + x] - ey;
                fy += e * e;
                fn++;
                c->bar_sum[i] += e;
                c->bar_n[i]++;
            }
        for (int y = (int)(0.025 * f->h); y < (int)(0.275 * f->h); y++)
            for (int x = x0 / 2; x < x1 / 2; x++) {
                const double e1 = f->u[(size_t)y * (f->w / 2) + x] - eu, e2 = f->v[(size_t)y * (f->w / 2) + x] - ev;
                c->se_c += 0.5 * (e1 * e1 + e2 * e2);
                c->n_c++;
            }
    }
    c->se_y += fy;
    c->n_y += fn;
    const double r = sqrt(fy / (double)fn);
    if (r > c->worst_y)
        c->worst_y = r;
}

static void cap_listener(void *ctx, const frame_t *f)
{
    capture_t *c = ctx;
    if (f->no_signal)
        return;
    frame_copy(&c->last, f);
    if ((int)c->frames++ >= c->skip)
        add_error(c, f);
}

typedef struct {
    capture_t cap;
    bool locked[2];
    video_std_t std[2];
    float snr[2];
    uint64_t fields_out, lines_from[3], resyncs[2], lock_losses[2];
    /* fuse mode */
    float fused_snr, delay_us;
    bool aligned;
    uint64_t realigns, fused_losses;
    uint64_t fly_entries, fly_resyncs;      /* the decoder that makes the picture */
    double good;                            /* bar rows shown right, per row of every field expected */
    double rms_y, rms_c, bias_y, noise_y;   /* rms_y^2 = bias_y^2 + noise_y^2 */
} result_t;

static int run(const char *spec, source_cfg_t sc, div_cfg_t vc, int nrx, result_t *res)
{
    memset(res, 0, sizeof(*res));
    res->cap.skip = 10;
    frame_alloc(&res->cap.last);
    sc.synth_nrx = nrx;
    sc.synth_realtime = false;
    source_t *src = source_open(spec, &sc);
    if (!src)
        return -1;
    ring_t ring;
    ring_init(&ring, 32, sc.block_samples, true);
    frame_pub_t pub;
    frame_pub_init(&pub, DEINT_BOB, false);
    frame_pub_add_listener(&pub, cap_listener, &res->cap);
    pipeline_cfg_t pc = {.div = vc, .nrx = nrx};
    pc.div.lossless = true;
    pc.div.realtime = false;
    decoder_cfg_default(&pc.dec);
    pc.dec.fs_nom = sc.fs;
    pc.dec.flywheel = g_flywheel;
    pipeline_t *pl = pipeline_new(&pc, &ring, &pub);
    pipeline_start(pl);
    source_start(src, &ring);
    pipeline_join(pl);
    source_stop(src);

    for (int r = 0; r < nrx; r++) {
        decoder_stats_t *s = decoder_stats(pipeline_rx(pl, r));
        res->locked[r] = atomic_load(&s->locked);
        res->std[r] = (video_std_t)atomic_load(&s->std);
        res->snr[r] = atomic_load(&s->snr_db);
        res->resyncs[r] = atomic_load(&s->resyncs);
        res->lock_losses[r] = atomic_load(&s->lock_losses);
    }
    if (pipeline_fusion(pl)) {
        fusion_stats_t *fs = fusion_stats(pipeline_fusion(pl));
        decoder_stats_t *s = decoder_stats(pipeline_fused(pl));
        res->fused_snr = atomic_load(&s->snr_db);
        res->fused_losses = atomic_load(&s->lock_losses);
        res->delay_us = atomic_load(&fs->delay_us);
        res->aligned = atomic_load(&fs->aligned);
        res->realigns = atomic_load(&fs->realigns);
    }
    {
        decoder_t *pd = pipeline_fused(pl) ? pipeline_fused(pl) : pipeline_rx(pl, 0);
        res->fly_entries = atomic_load(&decoder_stats(pd)->fly_entries);
        res->fly_resyncs = atomic_load(&decoder_stats(pd)->fly_resyncs);
    }
    /* A frozen picture shows nothing new: count against all the fields that should have come. */
    {
        const std_params_t *sp = std_params(sc.synth_std);
        const double expect = sc.seconds * sp->field_rate_num / sp->field_rate_den - res->cap.skip;
        const double per = res->cap.frames > (uint64_t)res->cap.skip
                               ? (double)res->cap.rows / (double)(res->cap.frames - (uint64_t)res->cap.skip) : 0;
        res->good = per > 0 && expect > 0 ? (double)res->cap.good_rows / (per * expect) : 0;
    }
    div_stats_t *ds = diversity_stats(pipeline_diversity(pl));
    res->fields_out = atomic_load(&ds->fields_out);
    for (int i = 0; i < 3; i++)
        res->lines_from[i] = atomic_load(&ds->lines_from[i]);
    res->rms_y = res->cap.n_y ? sqrt(res->cap.se_y / (double)res->cap.n_y) : 999;
    res->rms_c = res->cap.n_c ? sqrt(res->cap.se_c / (double)res->cap.n_c) : 999;
    for (int i = 0; i < 8; i++) {
        const double b = res->cap.bar_n[i] ? res->cap.bar_sum[i] / (double)res->cap.bar_n[i] : 0;
        res->bias_y += b * b / 8;
    }
    res->noise_y = sqrt(fmax(0, res->rms_y * res->rms_y - res->bias_y));
    res->bias_y = sqrt(res->bias_y);
    pipeline_free(pl);
    frame_pub_destroy(&pub);
    source_close(src);
    ring_free(&ring);
    return 0;
}

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("  FAIL: " __VA_ARGS__); printf("\n"); } \
                              else { printf("  ok:   " __VA_ARGS__); printf("\n"); } } while (0)

static void region_mean(const frame_t *f, int cx, int cy, int r, float *Y, float *U, float *V)
{
    double sy = 0, su = 0, sv = 0;
    int n = 0, nc = 0;
    for (int y = cy - r; y <= cy + r; y++)
        for (int x = cx - r; x <= cx + r; x++) {
            sy += f->y[(size_t)y * f->w + x];
            n++;
        }
    for (int y = (cy - r) / 2; y <= (cy + r) / 2; y++)
        for (int x = (cx - r) / 2; x <= (cx + r) / 2; x++) {
            su += f->u[(size_t)y * (f->w / 2) + x];
            sv += f->v[(size_t)y * (f->w / 2) + x];
            nc++;
        }
    *Y = (float)(sy / n);
    *U = (float)(su / nc);
    *V = (float)(sv / nc);
}

static void check_bars(const frame_t *f, const char *tag)
{
    static const char *names[8] = {"white", "yellow", "cyan", "green", "magenta", "red", "blue", "black"};
    int worst_y = 0, worst_c = 0;
    int cy = (int)(0.3 * f->h);
    for (int i = 0; i < 8; i++) {
        double Ye, Ue, Ve;
        cvbs_synth_picture((i + 0.5) / 8.0, 0.3, 0, &Ye, &Ue, &Ve);
        float ey = (float)(16 + 219 * Ye), eu = (float)(128 + 256.9 * Ue), ev = (float)(128 + 182.2 * Ve);
        float y, u, v;
        region_mean(f, 45 + 90 * i, cy, 8, &y, &u, &v);
        int dy = abs((int)lrintf(y - ey)), dc = abs((int)lrintf(u - eu));
        if (abs((int)lrintf(v - ev)) > dc)
            dc = abs((int)lrintf(v - ev));
        if (dy > worst_y) worst_y = dy;
        if (dc > worst_c) worst_c = dc;
        if (g_verbose)
            printf("    %-8s Y %5.1f/%5.1f  Cb %5.1f/%5.1f  Cr %5.1f/%5.1f\n", names[i], y, ey, u, eu, v, ev);
    }
    CHECK(worst_y <= 12, "%s colour bars luma within 12 (worst %d)", tag, worst_y);
    CHECK(worst_c <= 18, "%s colour bars chroma within 18 (worst %d)", tag, worst_c);
    float y, u, v;
    region_mean(f, 180, (int)(0.65 * f->h), 6, &y, &u, &v);
    float ey = 16 + 219 * 0.3f;
    CHECK(fabsf(y - ey) < 12 && fabsf(u - 128) < 10 && fabsf(v - 128) < 10,
          "%s grey ramp Y %.1f (exp %.1f), chroma %.0f/%.0f neutral", tag, y, ey, u, v);
}

static void test_standard(video_std_t std)
{
    const char *tag = std == STD_PAL ? "PAL" : "NTSC";
    printf("[%s clean, two receivers]\n", tag);
    source_cfg_t sc;
    source_cfg_default(&sc);
    sc.seconds = 0.7;
    div_cfg_t vc;
    div_cfg_default(&vc);
    result_t r;
    run(std == STD_PAL ? "synth:pal" : "synth:ntsc", sc, vc, 2, &r);
    const std_params_t *sp = std_params(std);
    CHECK(r.locked[0] && r.locked[1], "both decoders locked");
    CHECK(r.std[0] == std && r.std[1] == std, "standard detected as %s", tag);
    double expect = 0.7 * sp->field_rate_num / sp->field_rate_den;
    CHECK(r.fields_out >= expect - 8, "fields out %llu (expected ~%.0f)", (unsigned long long)r.fields_out, expect);
    CHECK(r.cap.last.h == sp->frame_h, "frame height %d", r.cap.last.h);
    CHECK(r.snr[0] > 25 && r.snr[1] > 25, "line SNR A %.1f dB, B %.1f dB", r.snr[0], r.snr[1]);
    check_bars(&r.cap.last, tag);
    frame_release(&r.cap.last);
}

static void test_diversity(void)
{
    printf("[diversity: A noisy with clicks, B clean]\n");
    source_cfg_t sc;
    source_cfg_default(&sc);
    sc.seconds = 0.5;
    sc.synth_noise[0] = 5.0f;
    sc.synth_clicks[0] = 0.5f;
    sc.synth_noise[1] = 0.6f;
    div_cfg_t vc;
    div_cfg_default(&vc);
    result_t r;
    run("synth:pal", sc, vc, 2, &r);
    uint64_t tot = r.lines_from[0] + r.lines_from[1] + r.lines_from[2];
    double fb = tot ? (double)r.lines_from[1] / tot : 0;
    CHECK(r.snr[1] > r.snr[0] + 6, "B measured better: A %.1f dB, B %.1f dB", r.snr[0], r.snr[1]);
    CHECK(fb > 0.9, "B selected for %.1f%% of lines", 100 * fb);
    check_bars(&r.cap.last, "diversity");
    frame_release(&r.cap.last);

    printf("[diversity: alternating fades]\n");
    source_cfg_default(&sc);
    sc.seconds = 1.0;
    sc.synth_fade_period = 0.4;
    sc.synth_fade_noise = 10.0f;
    run("synth:pal", sc, vc, 2, &r);
    tot = r.lines_from[0] + r.lines_from[1] + r.lines_from[2];
    CHECK(r.lines_from[0] > tot / 5 && r.lines_from[1] > tot / 5,
          "both receivers used: A %llu, B %llu, blended %llu lines", (unsigned long long)r.lines_from[0],
          (unsigned long long)r.lines_from[1], (unsigned long long)r.lines_from[2]);
    CHECK(r.lock_losses[0] + r.lock_losses[1] < 6, "lock losses A %llu, B %llu", (unsigned long long)r.lock_losses[0],
          (unsigned long long)r.lock_losses[1]);
    frame_release(&r.cap.last);
}

static void test_gaps(void)
{
    printf("[SMI read gaps every 5 ms, inaccurate gap estimates]\n");
    source_cfg_t sc;
    source_cfg_default(&sc);
    sc.seconds = 0.6;
    sc.synth_gap_every = 0.005;
    sc.synth_gap_us = 30;
    div_cfg_t vc;
    div_cfg_default(&vc);
    result_t r;
    run("synth:pal", sc, vc, 2, &r);
    CHECK(r.locked[0] && r.locked[1], "decoders stay locked");
    CHECK(r.lock_losses[0] == 0 && r.lock_losses[1] == 0, "no lock losses (resyncs A %llu)",
          (unsigned long long)r.resyncs[0]);
    CHECK(r.fields_out >= 0.6 * 50 - 8, "fields out %llu", (unsigned long long)r.fields_out);
    check_bars(&r.cap.last, "gaps");
    frame_release(&r.cap.last);
}

static void test_fake_vsync(void)
{
    printf("[fake vertical syncs: 20 per second, line-aligned]\n");
    source_cfg_t sc;
    source_cfg_default(&sc);
    sc.seconds = 1.0;
    sc.synth_fakevs = 20;
    div_cfg_t vc;
    div_cfg_default(&vc);
    result_t r;
    run("synth:pal", sc, vc, 1, &r);
    CHECK(r.fields_out >= 45 && r.fields_out <= 52, "fields out %llu (50 expected, none extra)",
          (unsigned long long)r.fields_out);
    double lpf = r.fields_out ? (double)r.lines_from[0] / (double)r.fields_out : 0;
    CHECK(lpf > 250, "%.0f valid lines per field (288 max)", lpf);
    frame_release(&r.cap.last);
}

static void test_single(void)
{
    printf("[single receiver, 8-bit lane]\n");
    source_cfg_t sc;
    source_cfg_default(&sc);
    sc.seconds = 0.4;
    div_cfg_t vc;
    div_cfg_default(&vc);
    result_t r;
    run("synth:pal", sc, vc, 1, &r);
    CHECK(r.locked[0], "decoder locked");
    CHECK(r.fields_out >= 0.4 * 50 - 8, "fields out %llu", (unsigned long long)r.fields_out);
    frame_release(&r.cap.last);
}

static void test_reclock(void)
{
    printf("[C5 code clock recovery]\n");
    static const double ppms[] = {0, 6, -12, 40, -40};
    for (size_t k = 0; k < sizeof(ppms) / sizeof(ppms[0]); k++) {
        source_cfg_t sc;
        source_cfg_default(&sc);
        sc.seconds = 0.7;
        sc.synth_ppm[0] = ppms[k];
        sc.synth_phase[0] = 0.13 * (double)k;
        div_cfg_t vc;
        div_cfg_default(&vc);
        result_t r;
        run("synth:pal", sc, vc, 1, &r);
        CHECK(r.locked[0] && r.rms_y < 3.0, "clock error %+.0f ppm: luma error %.1f (raw samples: ~9.7)", ppms[k], r.rms_y);
        frame_release(&r.cap.last);
    }
}

/* Two receivers with independent C5 clocks, fused. */
static source_cfg_t fuse_cfg(double seconds, double delay_us)
{
    source_cfg_t sc;
    source_cfg_default(&sc);
    sc.seconds = seconds;
    sc.synth_delay_us = delay_us;
    sc.synth_ppm[0] = 6;
    sc.synth_ppm[1] = -9;
    sc.synth_phase[1] = 0.37;
    return sc;
}

static void test_fuse(void)
{
    div_cfg_t vc;
    div_cfg_default(&vc);
    vc.mode = DIV_FUSE;
    result_t r;
    source_cfg_t sc;

    printf("[fuse: alignment, independent clocks]\n");
    static const double delays[] = {0, 0.31, 37.3, -250.8, 1234.5};
    for (size_t k = 0; k < sizeof(delays) / sizeof(delays[0]); k++) {
        sc = fuse_cfg(0.7, delays[k]);
        run(k & 1 ? "synth:ntsc" : "synth:pal", sc, vc, 2, &r);
        CHECK(r.aligned && fabs(r.delay_us - delays[k]) < 0.06, "delay %.2f us measured as %.2f", delays[k], r.delay_us);
        CHECK(r.noise_y < 3.0 && r.rms_c < 3.0, "picture error %.1f / %.1f", r.noise_y, r.rms_c);
        CHECK(r.realigns == 0, "no realignment");
        frame_release(&r.cap.last);
    }

    printf("[fuse: equal noise]\n");
    sc = fuse_cfg(1.0, 3.3);
    sc.synth_noise[0] = sc.synth_noise[1] = 4;
    run("synth:pal", sc, vc, 2, &r);
    float best = fmaxf(r.snr[0], r.snr[1]);
    CHECK(r.fused_snr > best + 1.8f, "A %.1f dB, B %.1f dB, fused %.1f dB", r.snr[0], r.snr[1], r.fused_snr);
    frame_release(&r.cap.last);

    printf("[fuse: A bad (noise and clicks), B clean]\n");
    sc = fuse_cfg(1.0, 3.3);
    sc.synth_noise[0] = 8;
    sc.synth_clicks[0] = 4;
    run("synth:pal", sc, vc, 2, &r);
    CHECK(r.rms_y < 4.0, "luma error %.1f (B alone ~2.4)", r.rms_y);
    frame_release(&r.cap.last);

    printf("[fuse: clicks on both]\n");
    sc = fuse_cfg(1.0, 3.3);
    sc.synth_clicks[0] = sc.synth_clicks[1] = 4;
    run("synth:pal", sc, vc, 2, &r);
    const double both = r.rms_y;
    frame_release(&r.cap.last);
    run("synth:pal", sc, vc, 1, &r);
    CHECK(both < 0.8 * r.rms_y, "luma error %.1f, one receiver %.1f", both, r.rms_y);
    frame_release(&r.cap.last);

    printf("[fuse: short independent dips]\n");
    sc = fuse_cfg(1.0, 3.3);
    sc.synth_dip_every = 0.02;
    sc.synth_dip_len = 0.002;
    sc.synth_dip_noise = 20;
    run("synth:pal", sc, vc, 2, &r);
    CHECK(r.rms_y < 9.0 && r.fused_losses == 0, "luma error %.1f (one receiver ~21), lock losses %llu", r.rms_y,
          (unsigned long long)r.fused_losses);
    frame_release(&r.cap.last);

    printf("[fuse: alternating slow fades]\n");
    sc = fuse_cfg(1.2, 3.3);
    sc.synth_fade_period = 0.4;
    sc.synth_fade_noise = 14;
    run("synth:pal", sc, vc, 2, &r);
    CHECK(r.rms_y < 5.0 && r.fused_losses == 0, "luma error %.1f, lock losses %llu", r.rms_y,
          (unsigned long long)r.fused_losses);
    frame_release(&r.cap.last);

    for (int lane = 0; lane < 2; lane++) {
        printf("[fuse: receiver %c lost for 0.5 s]\n", 'A' + lane);
        sc = fuse_cfg(1.5, 37.3);
        sc.synth_off_from[lane] = 0.5;
        sc.synth_off_to[lane] = 1.0;
        run("synth:pal", sc, vc, 2, &r);
        CHECK(r.fields_out >= 70, "fields out %llu of 75", (unsigned long long)r.fields_out);
        CHECK(r.fused_losses == 0 && r.cap.worst_y < 8, "fused lock losses %llu, worst frame error %.1f",
              (unsigned long long)r.fused_losses, r.cap.worst_y);
        CHECK(r.aligned && r.locked[lane], "receiver back and aligned");
        frame_release(&r.cap.last);
    }

    for (int lane = 0; lane < 2; lane++) {
        printf("[fuse: receiver %c never has a signal]\n", 'A' + lane);
        sc = fuse_cfg(1.0, 37.3);
        sc.synth_off_to[lane] = 10;
        run("synth:pal", sc, vc, 2, &r);
        CHECK(r.fields_out >= 40 && r.rms_y < 4, "fields out %llu, luma error %.1f", (unsigned long long)r.fields_out, r.rms_y);
        frame_release(&r.cap.last);
    }

    printf("[fuse: delay changes mid-run (a C5 restarted)]\n");
    static const double to[] = {45.0, 37.3 + 64.0 * 3};   /* sub-line, whole lines */
    for (int k = 0; k < 2; k++) {
        sc = fuse_cfg(2.0, 37.3);
        sc.synth_delay_step = 1.0;
        sc.synth_delay2_us = to[k];
        run("synth:pal", sc, vc, 2, &r);
        CHECK(r.aligned && fabs(r.delay_us - to[k]) < 0.06, "now %.2f us (%.2f expected)", r.delay_us, to[k]);
        CHECK(r.realigns >= 1 && r.realigns <= 3, "realignments %llu", (unsigned long long)r.realigns);
        frame_release(&r.cap.last);
    }

    printf("[fuse: capture gaps]\n");
    sc = fuse_cfg(1.0, 3.3);
    sc.synth_gap_every = 0.1;
    run("synth:pal", sc, vc, 2, &r);
    CHECK(r.fields_out >= 42 && r.aligned, "fields out %llu", (unsigned long long)r.fields_out);
    frame_release(&r.cap.last);
}

/* Signal only in short bursts ("scraps"), noise with no sync in between. */
static source_cfg_t scraps_cfg(double seconds, double burst, double dip)
{
    source_cfg_t sc = fuse_cfg(seconds, 3.3);
    sc.synth_dip_every = burst;     /* mean burst length (0.3..1.7 x) */
    sc.synth_dip_len = dip;
    sc.synth_dip_noise = 20;
    sc.synth_dip_keep = 0;
    return sc;
}

static void test_flywheel(void)
{
    div_cfg_t vc;
    div_cfg_default(&vc);
    result_t r, off;
    static const struct { const char *name; double burst, dip; } cases[] = {
        {"50 %", 0.004, 0.004}, {"20 %", 0.002, 0.008}, {"10 %", 0.0015, 0.0135},
    };
    for (size_t k = 0; k < sizeof(cases) / sizeof(cases[0]); k++) {
        printf("[flywheel: one receiver, signal %s of the time in 1-4 ms scraps]\n", cases[k].name);
        const source_cfg_t sc = scraps_cfg(2.0, cases[k].burst, cases[k].dip);
        const double duty = cases[k].burst / (cases[k].burst + cases[k].dip);
        g_flywheel = false;
        run("synth:pal", sc, vc, 1, &off);
        g_flywheel = true;
        run("synth:pal", sc, vc, 1, &r);
        CHECK(r.fields_out >= 95, "fields out %llu of 100 (without flywheel %llu)", (unsigned long long)r.fields_out,
              (unsigned long long)off.fields_out);
        CHECK(r.good > 0.7 * duty, "%.0f %% of rows show the picture in place (signal %.0f %%; without flywheel %.0f %%)",
              100 * r.good, 100 * duty, 100 * off.good);
        CHECK(r.fly_resyncs == 0, "no phase jumps (re-syncs %llu; the timing never changed)", (unsigned long long)r.fly_resyncs);
        frame_release(&off.cap.last);
        frame_release(&r.cap.last);
    }

    printf("[flywheel: two receivers fused, 20 %% scraps each, independent]\n");
    vc.mode = DIV_FUSE;
    run("synth:pal", scraps_cfg(2.0, 0.002, 0.008), vc, 2, &r);
    CHECK(r.fields_out >= 95 && r.good > 0.15, "fields out %llu, %.0f %% of rows in place", (unsigned long long)r.fields_out,
          100 * r.good);
    frame_release(&r.cap.last);
    vc.mode = DIV_AUTO;

    printf("[flywheel: no signal for 2 s, then back]\n");
    source_cfg_t sc = fuse_cfg(3.0, 3.3);
    sc.synth_off_from[0] = 0.5;
    sc.synth_off_to[0] = 2.5;
    run("synth:pal", sc, vc, 1, &r);
    CHECK(r.fields_out >= 145, "fields out %llu of 150", (unsigned long long)r.fields_out);
    CHECK(r.locked[0] && r.fly_entries >= 1, "locked again (flywheel entered %llu times)", (unsigned long long)r.fly_entries);
    check_bars(&r.cap.last, "after the outage");
    frame_release(&r.cap.last);

    printf("[flywheel: NTSC lost, comes back as PAL (another camera)]\n");
    sc = fuse_cfg(2.0, 3.3);
    sc.synth_off_from[0] = 0.5;
    sc.synth_off_to[0] = 0.8;
    sc.synth_std_at = 0.65;
    run("synth:ntsc", sc, vc, 1, &r);
    CHECK(r.locked[0] && r.std[0] == STD_PAL, "locked, %s detected", std_name(r.std[0]));
    CHECK(r.fields_out >= 100, "fields out %llu (~30 NTSC + ~60 PAL + the outage)", (unsigned long long)r.fields_out);
    check_bars(&r.cap.last, "after the switch");
    frame_release(&r.cap.last);

    static const double jumps[] = {23.7, 5 * 64.0 + 23.7};   /* new line phase; new line and field phase */
    for (int k = 0; k < 2; k++) {
        printf("[flywheel: VTX restart, video %.1f us later]\n", jumps[k]);
        sc = fuse_cfg(2.0, 3.3);
        sc.synth_off_from[0] = 0.5;
        sc.synth_off_to[0] = 0.8;
        sc.synth_jump_at = 0.65;
        sc.synth_jump_us = jumps[k];
        run("synth:pal", sc, vc, 1, &r);
        CHECK(r.fields_out >= 95, "fields out %llu of 100", (unsigned long long)r.fields_out);
        CHECK(r.locked[0] && r.fly_resyncs == 1, "locked, one re-sync (%llu)", (unsigned long long)r.fly_resyncs);
        check_bars(&r.cap.last, "after the restart");
        frame_release(&r.cap.last);
    }
}

/* Mode comparison on picture error: `test_pipeline bench`. */
static void bench(const char *only)
{
    static const struct { const char *name; float na, nb, clicks, fade, dip; double burst, len; } cases[] = {
        {"clean", 0.6f, 0.6f, 0, 0, 0, 0, 0},       {"noise 4/4", 4, 4, 0, 0, 0, 0, 0},
        {"noise 8/8", 8, 8, 0, 0, 0, 0, 0},         {"noise 4/8", 4, 8, 0, 0, 0, 0, 0},
        {"clicks 2/line", 1.5f, 1.5f, 2, 0, 0, 0, 0}, {"clicks 8/line", 1.5f, 1.5f, 8, 0, 0, 0, 0},
        {"slow fades", 1, 1, 0, 14, 0, 0, 0},       {"2 ms dips", 1, 1, 0, 0, 20, 0.02, 0.002},
        {"scraps 50%", 1, 1, 0, 0, 20, 0.004, 0.004}, {"scraps 20%", 1, 1, 0, 0, 20, 0.002, 0.008},
    };
    static const struct { const char *name; div_mode_t m; int nrx; } modes[] = {
        {"A only", DIV_FORCE_A, 2}, {"B only", DIV_FORCE_B, 2}, {"select", DIV_SELECT, 2},
        {"auto", DIV_AUTO, 2},      {"fuse", DIV_FUSE, 2},
    };
    for (size_t k = 0; k < sizeof(cases) / sizeof(cases[0]); k++) {
        if (only && !strstr(cases[k].name, only))
            continue;
        printf("%s\n", cases[k].name);
        for (size_t m = 0; m < sizeof(modes) / sizeof(modes[0]); m++) {
            source_cfg_t sc;
            source_cfg_default(&sc);
            sc.seconds = 1.5;
            sc.synth_delay_us = 3.3;
            sc.synth_noise[0] = cases[k].na;
            sc.synth_noise[1] = cases[k].nb;
            sc.synth_clicks[0] = sc.synth_clicks[1] = cases[k].clicks;
            if (cases[k].fade > 0) {
                sc.synth_fade_period = 0.4;
                sc.synth_fade_noise = cases[k].fade;
            }
            if (cases[k].dip > 0) {
                sc.synth_dip_every = cases[k].burst;
                sc.synth_dip_len = cases[k].len;
                sc.synth_dip_noise = cases[k].dip;
                if (cases[k].len > cases[k].burst)
                    sc.synth_dip_keep = 0;   /* scraps: nothing but noise in between */
            }
            div_cfg_t vc;
            div_cfg_default(&vc);
            vc.mode = modes[m].m;
            result_t r;
            run("synth:pal", sc, vc, modes[m].nrx, &r);
            printf("  %-7s luma bias %5.1f noise %5.1f  chroma %4.1f  worst %5.1f [%llu] in place %3.0f%%  losses A %llu B %llu fused %llu\n",
                   modes[m].name, r.bias_y, r.noise_y, r.rms_c, r.cap.worst_y, (unsigned long long)r.cap.frames, 100 * r.good,
                   (unsigned long long)r.lock_losses[0], (unsigned long long)r.lock_losses[1], (unsigned long long)r.fused_losses);
            fflush(stdout);
            frame_release(&r.cap.last);
        }
    }
}

int main(int argc, char **argv)
{
    if (argc > 1 && !strcmp(argv[1], "-v"))
        g_verbose = 1;
    if (argc > 1 && !strcmp(argv[1], "bench")) {
        /* bench [case] [nofly] */
        const char *only = argc > 2 && strcmp(argv[2], "nofly") ? argv[2] : NULL;
        g_flywheel = !(argc > 2 && !strcmp(argv[argc - 1], "nofly"));
        printf("flywheel %s\n", g_flywheel ? "on" : "off");
        bench(only);
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "fly")) {
        g_verbose = argc > 2;
        test_flywheel();
        return failures != 0;
    }
    if (argc > 1 && !strcmp(argv[1], "fuse")) {
        g_verbose = argc > 2;
        test_reclock();
        test_fuse();
        return failures != 0;
    }
    test_standard(STD_PAL);
    test_standard(STD_NTSC);
    test_diversity();
    test_gaps();
    test_fake_vsync();
    test_single();
    test_reclock();
    test_fuse();
    test_flywheel();
    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
