#include "probe.h"

#include <stdlib.h>
#include <string.h>

#include "common.h"

static const char *k_c5_pin[6] = {"D4", "D5", "D3", "D7", "D8", "D9"};
static const int k_pi_gpio[2][6] = {{8, 9, 10, 11, 12, 13}, {16, 17, 18, 19, 20, 21}};
static const int k_pi_pin[2][6] = {{24, 21, 19, 23, 32, 33}, {36, 11, 12, 35, 38, 40}};

typedef struct {
    field_t f[2];
    uint64_t fields[2];
    double snr[2];
} probe_sink_t;

static field_t *ps_acquire(void *ctx, int rx) { return &((probe_sink_t *)ctx)->f[rx]; }

static void ps_submit(void *ctx, field_t *f)
{
    probe_sink_t *p = ctx;
    p->fields[f->rx]++;
    if (f->valid_lines > 0)
        p->snr[f->rx] = f->snr_db;
}

static int percentile(const uint64_t *h, uint64_t n, double q)
{
    uint64_t acc = 0;
    for (int i = 0; i < 64; i++) {
        acc += h[i];
        if ((double)acc >= q * (double)n)
            return i;
    }
    return 63;
}

int run_probe(source_t *src, ring_t *ring, double seconds, const decoder_cfg_t *dcfg)
{
    probe_sink_t *ps = calloc(1, sizeof(*ps));
    field_sink_t sink = {.acquire = ps_acquire, .submit = ps_submit, .ctx = ps};
    const int nl = src->nlanes;
    decoder_t *dec[2] = {NULL, NULL};
    for (int l = 0; l < nl; l++) {
        decoder_cfg_t c = *dcfg;
        c.lane = l;
        dec[l] = decoder_new(&c, l, sink);
    }
    uint64_t hist[2][64] = {{0}}, tog[2][6] = {{0}};
    uint64_t within[2] = {0, 0}, across[2] = {0, 0};
    uint64_t n = 0;
    uint16_t prev = 0;
    /* Activity timeline per receiver: one character per 250 ms,
     * '#' video-like signal, '.' DAC idle (code 20: the C5's TX is off),
     * '0' all zero (not connected/powered). */
    char timeline[2][512];
    unsigned tl_n = 0;
    uint64_t tl_samples = 0, tl_idle[2] = {0, 0}, tl_zero[2] = {0, 0};
    bool have_prev = false;
    const double fs = src->cfg.fs;
    const double want = seconds * fs;

    int cid = ring_add_consumer(ring);
    if (source_start(src, ring)) {
        LOGE("cannot start source");
        return 1;
    }
    double t0 = now_s();
    while (atomic_load(&g_running) && (double)n < want) {
        const ring_block_t *b;
        bool ovr;
        ring_status_t st = ring_next(ring, cid, &b, &ovr, 1000);
        if (st == RING_EOF)
            break;
        if (st == RING_TIMEOUT) {
            if (now_s() - t0 > seconds + 3) {
                LOGE("no data from source");
                break;
            }
            continue;
        }
        const uint16_t *s = b->data;
        for (uint32_t i = 0; i < b->n; i++) {
            uint16_t x = s[i];
            for (int l = 0; l < nl; l++) {
                int c = (x >> (8 * l)) & 63;
                hist[l][c]++;
                if (have_prev) {
                    int pc = (prev >> (8 * l)) & 63;
                    int t = c ^ pc;
                    for (int k = 0; k < 6; k++)
                        tog[l][k] += (t >> k) & 1;
                    unsigned dabs = (unsigned)abs(c - pc);
                    if (i & 1) within[l] += dabs; else across[l] += dabs;
                }
            }
            prev = x;
            have_prev = true;
        }
        for (int l = 0; l < nl; l++) {
            for (uint32_t i = 0; i < b->n; i += 16) {
                int c = (s[i] >> (8 * l)) & 63;
                tl_idle[l] += c == 20;
                tl_zero[l] += c == 0;
            }
        }
        tl_samples += (b->n + 15) / 16;
        if (tl_samples * 16 >= (uint64_t)(0.25 * fs)) {
            if (tl_n < sizeof(timeline[0]) - 1) {
                for (int l = 0; l < nl; l++)
                    timeline[l][tl_n] = tl_zero[l] * 10 > tl_samples * 9 ? '0'
                                      : tl_idle[l] * 10 > tl_samples * 9 ? '.' : '#';
                tl_n++;
            }
            tl_samples = 0;
            tl_idle[0] = tl_idle[1] = tl_zero[0] = tl_zero[1] = 0;
        }
        for (int l = 0; l < nl; l++)
            decoder_feed(dec[l], b->data, b->n, b->start, (b->flags & RING_F_DISCONT) || ovr);
        n += b->n;
        ring_release(ring, cid);
    }
    double el = now_s() - t0;
    ring_remove_consumer(ring, cid);
    atomic_store(&g_running, false);
    source_stop(src);

    printf("\n=== c5div probe (%s source) ===\n", src->kind);
    printf("captured %.2f M samples in %.2f s", n / 1e6, el);
    if (src->realtime)
        printf("  ->  %.3f MS/s measured (nominal --fs %.3f)", atomic_load(&src->rate_meas) / 1e6, fs / 1e6);
    printf("\nblocks %llu, gaps %llu (avg %.1f us), short reads %llu\n",
           (unsigned long long)atomic_load(&src->blocks), (unsigned long long)atomic_load(&src->gaps),
           atomic_load(&src->gap_us_avg), (unsigned long long)atomic_load(&src->short_reads));
    if (n == 0) {
        printf("no samples: check the SMI overlay and permissions\n");
        return 1;
    }
    for (int l = 0; l < nl; l++) {
        printf("\nreceiver %c (Pi GPIO %d-%d)\n", 'A' + l, k_pi_gpio[l][0], k_pi_gpio[l][5]);
        int mn = 0, mx = 63;
        while (mn < 63 && !hist[l][mn]) mn++;
        while (mx > 0 && !hist[l][mx]) mx--;
        printf("  codes: min %d  p1 %d  median %d  p99 %d  max %d\n", mn, percentile(hist[l], n, 0.01),
               percentile(hist[l], n, 0.5), percentile(hist[l], n, 0.99), mx);
        printf("  bit toggles / s:");
        for (int k = 0; k < 6; k++)
            printf(" b%d %.2gM", k, tog[l][k] / el / 1e6);
        printf("\n");
        bool any_dead = false;
        for (int k = 0; k < 6; k++)
            if (tog[l][k] == 0) {
                any_dead = true;
                printf("  !! bit %d never changes: check C5 %s -> series R -> Pi GPIO%d (header pin %d)\n", k,
                       k_c5_pin[k], k_pi_gpio[l][k], k_pi_pin[l][k]);
            }
        if (mx == 0)
            printf("  !! lane reads all zeros: receiver not connected/powered, or C5VRX not running\n");
        double ratio = within[l] ? (double)across[l] / (double)within[l] : 0;
        timeline[l][tl_n] = 0;
        printf("  activity (250 ms/char, # video . C5 output idle 0 zero): %s\n", timeline[l]);
        printf("  sample-order check: %.2f %s\n", ratio,
               ratio > 1.4 ? "(SUSPECT swapped 16-bit pairs: try --smi-swap)" : "(ok)");
        decoder_stats_t *st = decoder_stats(dec[l]);
        bool locked = atomic_load(&st->locked);
        printf("  decoder: %s", locked ? "LOCKED" : "no lock");
        if (locked || atomic_load(&st->fields)) {
            printf(", %s, line %.3f us (capture rate est. %.4f MS/s)\n", std_name((video_std_t)atomic_load(&st->std)),
                   atomic_load(&st->line_us), atomic_load(&st->fs_est) / 1e6);
            printf("           sync depth %.1f codes, blank %.1f, burst %.2f x nominal, line SNR %.1f dB\n",
                   atomic_load(&st->sync_amp), atomic_load(&st->blank), atomic_load(&st->burst),
                   atomic_load(&st->snr_db));
            printf("           pulses: hsync %llu, equalising %llu, broad %llu, other %llu; vsyncs %llu, rejected %llu, "
                   "flywheel %llu\n",
                   (unsigned long long)atomic_load(&st->pulses_hsync), (unsigned long long)atomic_load(&st->pulses_eq),
                   (unsigned long long)atomic_load(&st->pulses_broad), (unsigned long long)atomic_load(&st->pulses_other),
                   (unsigned long long)atomic_load(&st->vsyncs), (unsigned long long)atomic_load(&st->vsync_rejected),
                   (unsigned long long)atomic_load(&st->vsync_flywheel));
            printf("           fields %llu, lines %llu, coasted %llu, resyncs %llu, lock losses %llu\n",
                   (unsigned long long)ps->fields[l], (unsigned long long)atomic_load(&st->lines),
                   (unsigned long long)atomic_load(&st->coasted), (unsigned long long)atomic_load(&st->resyncs),
                   (unsigned long long)atomic_load(&st->lock_losses));
        } else {
            printf("\n");
            if (!any_dead && mx > 0)
                printf("  hint: bits toggle but no sync found - check bit order (D4..D9 = LSB..MSB),\n"
                       "        common ground, and that the VTX channel matches the C5VRX channel\n");
        }
    }
    for (int l = 0; l < nl; l++)
        decoder_free(dec[l]);
    free(ps);
    return 0;
}

int run_record(source_t *src, ring_t *ring, double seconds, const char *path)
{
    FILE *f = fopen(path, "wb");
    if (!f) {
        LOGE("cannot write %s", path);
        return 1;
    }
    raw_header_t h;
    memset(&h, 0, sizeof(h));
    memcpy(h.magic, RAW_MAGIC, 8);
    h.version = 1;
    h.sample_bits = 16;
    h.fs = src->cfg.fs;
    h.nlanes = (uint32_t)src->nlanes;
    fwrite(&h, sizeof(h), 1, f);

    int cid = ring_add_consumer(ring);
    if (source_start(src, ring)) {
        fclose(f);
        return 1;
    }
    uint64_t n = 0;
    const double want = seconds * src->cfg.fs;
    while (atomic_load(&g_running) && (double)n < want) {
        const ring_block_t *b;
        bool ovr;
        ring_status_t st = ring_next(ring, cid, &b, &ovr, 1000);
        if (st == RING_EOF)
            break;
        if (st == RING_TIMEOUT)
            continue;
        raw_block_header_t bh = {.start = b->start, .gap_est = b->gap_est, .n = b->n,
                                 .flags = b->flags | (ovr ? RING_F_DISCONT : 0)};
        fwrite(&bh, sizeof(bh), 1, f);
        fwrite(b->data, sizeof(uint16_t), b->n, f);
        n += b->n;
        ring_release(ring, cid);
    }
    ring_remove_consumer(ring, cid);
    atomic_store(&g_running, false);
    source_stop(src);
    fclose(f);
    LOGI("recorded %.2f M samples (%.2f s) to %s", n / 1e6, n / src->cfg.fs, path);
    return 0;
}
