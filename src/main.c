/* c5div - dual ESP32-C5 (C5VRX) diversity video receiver for Raspberry Pi.
 *
 *   c5div run    [options]   receive and display (default command)
 *   c5div probe  [options]   wiring / signal diagnostics
 *   c5div record [options] -O FILE   save raw capture for offline work
 */
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#include "chanlink.h"
#include "common.h"
#include "decoder.h"
#include "diversity.h"
#include "frame.h"
#include "jumpers.h"
#include "outputs.h"
#include "pipeline.h"
#include "probe.h"
#include "ring.h"
#include "source.h"

int g_verbose = 0;
atomic_bool g_running = true;

static void on_signal(int sig)
{
    (void)sig;
    atomic_store(&g_running, false);
}

typedef struct {
    pipeline_t *pl;
    int nrx;
    diversity_t *dv;
    source_t *src;
    div_mode_t mode;
    double t_last;
    uint64_t f_last;
    uint64_t skip_last;
    bool chan_on;           /* channel link configured */
    atomic_int channel;     /* 1..8, 0 unknown */
    float fps;
} app_t;

static const char *mode_name(div_mode_t m)
{
    switch (m) {
    case DIV_AUTO: return "auto";
    case DIV_SELECT: return "select";
    case DIV_BLEND: return "blend";
    case DIV_FORCE_A: return "A only";
    case DIV_FORCE_B: return "B only";
    case DIV_FUSE: return "fuse";
    }
    return "?";
}

static void update_fps(app_t *a)
{
    double now = now_s();
    uint64_t f = atomic_load(&diversity_stats(a->dv)->fields_out);
    if (now - a->t_last >= 1.0) {
        a->fps = (float)((f - a->f_last) / (now - a->t_last));
        a->f_last = f;
        a->t_last = now;
    }
}

static size_t status_json(void *ctx, char *buf, size_t n)
{
    app_t *a = ctx;
    div_stats_t *ds = diversity_stats(a->dv);
    fusion_t *fu = pipeline_fusion(a->pl);
    size_t o = 0;
    o += (size_t)snprintf(buf + o, n - o, "{\"rx\":[");
    for (int r = 0; r < a->nrx; r++) {
        decoder_stats_t *s = decoder_stats(pipeline_rx(a->pl, r));
        o += (size_t)snprintf(buf + o, n - o,
                              "%s{\"name\":\"%c\",\"locked\":%s,\"fly\":%s,\"std\":\"%s\",\"snr_db\":%.1f,\"use\":%.3f,"
                              "\"sync_amp\":%.1f,\"burst\":%.2f,\"fields\":%llu,\"lock_losses\":%llu}",
                              r ? "," : "", 'A' + r, atomic_load(&s->locked) ? "true" : "false",
                              atomic_load(&s->fly) ? "true" : "false",
                              std_name((video_std_t)atomic_load(&s->std)), atomic_load(&s->snr_db),
                              pipeline_share(a->pl, r), atomic_load(&s->sync_amp), atomic_load(&s->burst),
                              (unsigned long long)atomic_load(&s->fields),
                              (unsigned long long)atomic_load(&s->lock_losses));
        if (o >= n) return 0;
    }
    o += (size_t)snprintf(buf + o, n - o, "]");
    if (fu) {
        fusion_stats_t *fs = fusion_stats(fu);
        decoder_stats_t *s = decoder_stats(pipeline_fused(a->pl));
        o += (size_t)snprintf(buf + o, n - o,
                              ",\"fused\":{\"locked\":%s,\"fly\":%s,\"snr_db\":%.1f,\"aligned\":%s,\"delay_us\":%.3f}",
                              atomic_load(&s->locked) ? "true" : "false", atomic_load(&s->fly) ? "true" : "false",
                              atomic_load(&s->snr_db),
                              atomic_load(&fs->aligned) ? "true" : "false", atomic_load(&fs->delay_us));
        if (o >= n) return 0;
    }
    if (a->chan_on) {
        const int ch = atomic_load(&a->channel);
        if (ch)
            o += (size_t)snprintf(buf + o, n - o, ",\"channel\":\"R%d\"", ch);
        else
            o += (size_t)snprintf(buf + o, n - o, ",\"channel\":null");
        if (o >= n) return 0;
    }
    o += (size_t)snprintf(buf + o, n - o,
                          ",\"fps\":%.1f,\"mode\":\"%s\",\"capture_msps\":%.3f,\"gap_us\":%.1f,\"dropped\":%llu}",
                          a->fps, mode_name(a->mode), atomic_load(&a->src->rate_meas) / 1e6,
                          atomic_load(&a->src->gap_us_avg),
                          (unsigned long long)atomic_load(&ds->dropped));
    return o < n ? o : 0;
}

/* ------------------------------------------------------------ outputs */

typedef enum { VID_NONE, VID_HDMI, VID_AV } vid_out_t;

typedef struct {
    frame_pub_t *pub;
    drm_cfg_t dc;
    http_cfg_t hc;
    vid_out_t vid;
    out_drm_t *drm;
    out_http_t *http;
} outputs_t;

/* Start/stop outputs to match; safe to call while running. */
static void set_outputs(outputs_t *o, vid_out_t vid, bool web)
{
    if (vid != o->vid) {
#ifdef HAVE_DRM
        out_drm_stop(o->drm);
        o->drm = NULL;
        if (vid != VID_NONE) {
            o->dc.av = vid == VID_AV;
            o->drm = out_drm_start(&o->dc, o->pub);
        }
#else
        if (vid != VID_NONE)
            LOGW("built without libdrm: hdmi/av output unavailable");
#endif
        o->vid = vid;
    }
#ifdef HAVE_JPEG
    if (web && !o->http)
        o->http = out_http_start(&o->hc, o->pub);
    if (!web && o->http) {
        out_http_stop(o->http);
        o->http = NULL;
    }
#else
    if (web)
        LOGW("built without libjpeg: web output unavailable");
#endif
}

static void poll_jumpers(outputs_t *o, const jumper_io_t *io, const char *sim, jumper_filter_t *f, bool web_always)
{
    jumpers_t j;
    if (jumper_read(io, sim, &j) || !jumper_filter(f, j, now_s()))
        return;
    LOGI("jumpers: %s, web viewer %s", j.av ? "AV" : "HDMI", j.wifi || web_always ? "on" : "off");
    set_outputs(o, j.av ? VID_AV : VID_HDMI, j.wifi || web_always);
}

static void print_status(app_t *a)
{
    decoder_t *fused = pipeline_fused(a->pl);
    char line[512];
    int o = 0;
    if (a->chan_on && atomic_load(&a->channel))
        o += snprintf(line + o, sizeof(line) - (size_t)o, "R%d | ", atomic_load(&a->channel));
    for (int r = 0; r < a->nrx; r++) {
        decoder_stats_t *s = decoder_stats(pipeline_rx(a->pl, r));
        if (atomic_load(&s->locked))
            o += snprintf(line + o, sizeof(line) - (size_t)o, "%c: %s %4.1fdB use %3.0f%% | ", 'A' + r,
                          std_name((video_std_t)atomic_load(&s->std)), atomic_load(&s->snr_db),
                          100.0f * pipeline_share(a->pl, r));
        else if (atomic_load(&s->fly))
            o += snprintf(line + o, sizeof(line) - (size_t)o, "%c: flywheel         | ", 'A' + r);
        else
            o += snprintf(line + o, sizeof(line) - (size_t)o, "%c: no lock          | ", 'A' + r);
    }
    if (fused && atomic_load(&decoder_stats(fused)->locked))
        o += snprintf(line + o, sizeof(line) - (size_t)o, "fused %4.1fdB | ", atomic_load(&decoder_stats(fused)->snr_db));
    else if (fused && atomic_load(&decoder_stats(fused)->fly))
        o += snprintf(line + o, sizeof(line) - (size_t)o, "fused flywheel | ");
    uint64_t skip = fused ? atomic_load(&decoder_stats(fused)->fields_skipped) : 0;
    for (int r = 0; !fused && r < a->nrx; r++)
        skip += atomic_load(&decoder_stats(pipeline_rx(a->pl, r))->fields_skipped);
    if (skip != a->skip_last)
        o += snprintf(line + o, sizeof(line) - (size_t)o, "CPU overload, %llu fields skipped | ",
                      (unsigned long long)(skip - a->skip_last));
    a->skip_last = skip;
    snprintf(line + o, sizeof(line) - (size_t)o, "%.1f fps", a->fps);
    LOGI("%s", line);
}

static void usage(void)
{
    fprintf(stderr,
"usage: c5div [run|probe|record] [options]\n"
"\n"
"source\n"
"  -s, --source SPEC       smi (gapless DMA) | smidev (kernel reads, gaps) | synth[:pal|ntsc] |\n"
"                          file:PATH                                        (default smi)\n"
"      --rx AB|A           receivers wired (A only = 8-bit SMI)            (default AB)\n"
"      --fs HZ             capture sample rate                              (default 25e6)\n"
"      --smi-timing S,T,H[,P]  SMI setup,strobe,hold,pace clock cycles      (default 1,3,1,0)\n"
"      --smi-swap          smidev: swap 16-bit sample pairs (if probe says so)\n"
"      --dma-chan N        smi: DMA channel used by neither Linux nor firmware (default 7)\n"
"      --dma-force         smi: use the channel even if it shows earlier use (careful)\n"
"      --block N           samples per capture block                        (default 32768)\n"
"      --seconds S         stop after S seconds of signal\n"
"      --fast              synth/file: run as fast as possible, never drop\n"
"\n"
"output\n"
"  -o, --output LIST       comma list of hdmi, av, web, jumpers, y4m:PATH, none  (default hdmi,web)\n"
"                          av: composite video (instead of hdmi; they cannot run together)\n"
"                          jumpers: GPIO4-GND fitted = av, open = hdmi; GPIO5-GND fitted = web\n"
"      --channel-uart DEV  serial port the master receiver's channel arrives on, or none\n"
"                                                                    (default /dev/serial0 with -s smi)\n"
"      --av-std S          auto (same as the camera) | pal | ntsc            (default auto)\n"
"      --av-progressive    288p/240p instead of 576i/480i\n"
"      --hdmi-mode M       auto | WxH@HZ                                     (default auto)\n"
"      --stretch           fill the screen (default keeps 4:3)\n"
"      --web-port N        (default 80 and 8080)   --web-fps N (30)   --jpeg-quality N (70)\n"
"      --deint bob|weave   (default bob: 50/60 fps, lowest latency)\n"
"      --no-osd            hide the receiver signal bars\n"
"\n"
"decoding\n"
"      --std auto|pal|ntsc     (default auto)\n"
"      --div fuse|auto|select|blend|a|b   diversity mode                  (default fuse)\n"
"                          fuse: align + combine the two composite streams, one decoder\n"
"                          auto/select/blend: two full decoders, per-line choice (more CPU)\n"
"      --hyst DB  --blend-db DB      select hysteresis (1.5) / auto blend window (3)\n"
"      --mono  --sat X  --contrast X  --brightness X  --hshift US  --voffset N\n"
"      --median            3-tap median on input samples (more CPU)\n"
"      --workers N         line-decoding threads per receiver (default 2; 0 = inline)\n"
"      --hq                20-tap sinc resampler instead of prefilter+linear (more CPU)\n"
"      --no-chroma-avg\n"
"\n"
"synthetic source (testing without hardware)\n"
"      --synth-noise A,B   noise rms in DAC codes   --synth-clicks A,B  clicks per line\n"
"      --synth-fade SEC    A and B fade alternately with this period\n"
"      --synth-gaps SEC    emulate SMI read gaps every SEC seconds\n"
"      --no-reclock        skip C5 code-clock recovery (it needs 25 MS/s captures)\n"
"      --no-flywheel       stop decoding when the sync is lost (default: keep the line and field\n"
"                          timing going and show what arrives, picture scraps or noise)\n"
"      --synth-ppm A,B     C5 clock errors\n"
"      --synth-fakevs N    inject N line-aligned fake vertical syncs per second\n"
"      --synth-delay US    receiver B sees the video US microseconds later\n"
"\n"
"record\n"
"  -O, --out FILE          raw capture file\n"
"  -v, --verbose\n");
}

static bool parse_pair(const char *s, float *a, float *b)
{
    char *e;
    *a = strtof(s, &e);
    if (*e == ',') {
        *b = strtof(e + 1, &e);
        return *e == 0;
    }
    *b = *a;
    return *e == 0;
}

int main(int argc, char **argv)
{
    const char *cmd = "run";
    if (argc > 1 && argv[1][0] != '-') {
        cmd = argv[1];
        argv++;
        argc--;
    }
    if (strcmp(cmd, "run") && strcmp(cmd, "probe") && strcmp(cmd, "record")) {
        usage();
        return 2;
    }

    const char *src_spec = "smi", *outputs = "hdmi,web", *rec_path = NULL, *hdmi_mode = "auto";
    bool rx_b = true, fast = false, osd = true, stretch = false;
    int web_port = 0, web_fps = 30, jpeg_q = 70;
    deint_t deint = DEINT_BOB;
    bool div_given = false;
    video_std_t av_std = STD_UNKNOWN;
    bool av_prog = false;
    const char *jumper_sim = NULL, *chan_uart = NULL;
    source_cfg_t scfg;
    source_cfg_default(&scfg);
    decoder_cfg_t dcfg;
    decoder_cfg_default(&dcfg);
    div_cfg_t vcfg;
    div_cfg_default(&vcfg);
    if (!strcmp(cmd, "probe"))
        scfg.seconds = 2.0;

    enum {
        O_RX = 256, O_FS, O_SMI_TIMING, O_SMI_SWAP, O_BLOCK, O_SECONDS, O_FAST, O_HDMI_MODE, O_STRETCH,
        O_WEB_PORT, O_WEB_FPS, O_JPEG_Q, O_DEINT, O_NO_OSD, O_STD, O_DIV, O_HYST, O_BLEND_DB, O_MONO, O_SAT,
        O_DMA_CHAN, O_DMA_FORCE, O_WORKERS, O_CONTRAST, O_BRIGHT, O_HSHIFT, O_VOFF, O_NO_MEDIAN, O_HQ, O_NO_CAVG, O_SN, O_SC, O_SF, O_SG, O_SFV, O_SDL, O_RECLK, O_NOFLY, O_SPPM, O_AVSTD, O_AVPROG, O_JSIM, O_CHAN,
    };
    static const struct option opts[] = {
        {"source", required_argument, 0, 's'}, {"output", required_argument, 0, 'o'},
        {"out", required_argument, 0, 'O'}, {"verbose", no_argument, 0, 'v'}, {"help", no_argument, 0, 'h'},
        {"rx", required_argument, 0, O_RX}, {"fs", required_argument, 0, O_FS},
        {"smi-timing", required_argument, 0, O_SMI_TIMING}, {"smi-swap", no_argument, 0, O_SMI_SWAP}, {"dma-chan", required_argument, 0, O_DMA_CHAN}, {"dma-force", no_argument, 0, O_DMA_FORCE}, {"workers", required_argument, 0, O_WORKERS},
        {"block", required_argument, 0, O_BLOCK}, {"seconds", required_argument, 0, O_SECONDS},
        {"fast", no_argument, 0, O_FAST}, {"hdmi-mode", required_argument, 0, O_HDMI_MODE},
        {"stretch", no_argument, 0, O_STRETCH}, {"web-port", required_argument, 0, O_WEB_PORT},
        {"web-fps", required_argument, 0, O_WEB_FPS}, {"jpeg-quality", required_argument, 0, O_JPEG_Q},
        {"deint", required_argument, 0, O_DEINT}, {"no-osd", no_argument, 0, O_NO_OSD},
        {"std", required_argument, 0, O_STD}, {"div", required_argument, 0, O_DIV},
        {"hyst", required_argument, 0, O_HYST}, {"blend-db", required_argument, 0, O_BLEND_DB},
        {"mono", no_argument, 0, O_MONO}, {"sat", required_argument, 0, O_SAT},
        {"contrast", required_argument, 0, O_CONTRAST}, {"brightness", required_argument, 0, O_BRIGHT},
        {"hshift", required_argument, 0, O_HSHIFT}, {"voffset", required_argument, 0, O_VOFF},
        {"median", no_argument, 0, O_NO_MEDIAN}, {"hq", no_argument, 0, O_HQ}, {"no-chroma-avg", no_argument, 0, O_NO_CAVG},
        {"synth-noise", required_argument, 0, O_SN}, {"synth-clicks", required_argument, 0, O_SC},
        {"synth-fade", required_argument, 0, O_SF}, {"synth-gaps", required_argument, 0, O_SG}, {"synth-fakevs", required_argument, 0, O_SFV}, {"synth-delay", required_argument, 0, O_SDL}, {"av-std", required_argument, 0, O_AVSTD}, {"channel-uart", required_argument, 0, O_CHAN}, {"av-progressive", no_argument, 0, O_AVPROG}, {"jumper-sim", required_argument, 0, O_JSIM},
        {"no-reclock", no_argument, 0, O_RECLK}, {"no-flywheel", no_argument, 0, O_NOFLY},
        {"synth-ppm", required_argument, 0, O_SPPM},
        {0, 0, 0, 0},
    };
    int c;
    while ((c = getopt_long(argc, argv, "s:o:O:vh", opts, NULL)) != -1) {
        switch (c) {
        case 's': src_spec = optarg; break;
        case 'o': outputs = optarg; break;
        case 'O': rec_path = optarg; break;
        case 'v': g_verbose++; break;
        case O_RX:
            if (!strcasecmp(optarg, "A")) rx_b = false;
            else if (!strcasecmp(optarg, "AB")) rx_b = true;
            else { LOGE("--rx must be AB or A"); return 2; }
            break;
        case O_FS: scfg.fs = atof(optarg); break;
        case O_SMI_TIMING:
            if (sscanf(optarg, "%d,%d,%d,%d", &scfg.smi_setup, &scfg.smi_strobe, &scfg.smi_hold, &scfg.smi_pace) < 3) {
                LOGE("--smi-timing S,T,H[,P]");
                return 2;
            }
            break;
        case O_SMI_SWAP: scfg.smi_swap_pairs = true; break;
        case O_DMA_CHAN: scfg.dma_chan = atoi(optarg); break;
        case O_DMA_FORCE: scfg.dma_force = true; break;
        case O_WORKERS: dcfg.workers = atoi(optarg); break;
        case O_BLOCK: scfg.block_samples = (uint32_t)atoi(optarg); break;
        case O_SECONDS: scfg.seconds = atof(optarg); break;
        case O_FAST: fast = true; break;
        case O_HDMI_MODE: hdmi_mode = optarg; break;
        case O_STRETCH: stretch = true; break;
        case O_WEB_PORT: web_port = atoi(optarg); break;
        case O_WEB_FPS: web_fps = atoi(optarg); break;
        case O_JPEG_Q: jpeg_q = atoi(optarg); break;
        case O_DEINT: deint = !strcmp(optarg, "weave") ? DEINT_WEAVE : DEINT_BOB; break;
        case O_NO_OSD: osd = false; break;
        case O_STD:
            dcfg.std_force = !strcmp(optarg, "pal") ? STD_PAL : !strcmp(optarg, "ntsc") ? STD_NTSC : STD_UNKNOWN;
            break;
        case O_DIV:
            vcfg.mode = !strcmp(optarg, "select") ? DIV_SELECT : !strcmp(optarg, "blend") ? DIV_BLEND
                      : !strcmp(optarg, "fuse") ? DIV_FUSE : !strcasecmp(optarg, "a") ? DIV_FORCE_A
                      : !strcasecmp(optarg, "b") ? DIV_FORCE_B : DIV_AUTO;
            div_given = true;
            break;
        case O_HYST: vcfg.hyst_db = (float)atof(optarg); break;
        case O_BLEND_DB: vcfg.blend_db = (float)atof(optarg); break;
        case O_MONO: dcfg.mono = true; break;
        case O_SAT: dcfg.saturation = (float)atof(optarg); break;
        case O_CONTRAST: dcfg.contrast = (float)atof(optarg); break;
        case O_BRIGHT: dcfg.brightness = (float)atof(optarg); break;
        case O_HSHIFT: dcfg.hshift_us = (float)atof(optarg); break;
        case O_VOFF: dcfg.voffset = atoi(optarg); break;
        case O_NO_MEDIAN: dcfg.median3 = true; break;
        case O_HQ: dcfg.hq_resample = true; break;
        case O_NO_CAVG: dcfg.chroma_vavg = false; break;
        case O_SN: parse_pair(optarg, &scfg.synth_noise[0], &scfg.synth_noise[1]); break;
        case O_SC: parse_pair(optarg, &scfg.synth_clicks[0], &scfg.synth_clicks[1]); break;
        case O_SF: scfg.synth_fade_period = atof(optarg); break;
        case O_SG: scfg.synth_gap_every = atof(optarg); break;
        case O_SFV: scfg.synth_fakevs = atof(optarg); break;
        case O_AVSTD:
            av_std = !strcmp(optarg, "pal") ? STD_PAL : !strcmp(optarg, "ntsc") ? STD_NTSC : STD_UNKNOWN;
            break;
        case O_CHAN: chan_uart = optarg; break;
        case O_AVPROG: av_prog = true; break;
        case O_JSIM: jumper_sim = optarg; break;    /* testing: file instead of GPIOs */
        case O_RECLK: dcfg.reclock = false; break;
        case O_NOFLY: dcfg.flywheel = false; break;
        case O_SPPM: {
            double pa = 0, pb = 0;
            sscanf(optarg, "%lf,%lf", &pa, &pb);
            scfg.synth_ppm[0] = pa;
            scfg.synth_ppm[1] = pb;
            break;
        }
        case O_SDL: scfg.synth_delay_us = atof(optarg); break;
        case 'h': usage(); return 0;
        default: usage(); return 2;
        }
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    scfg.smi_width = rx_b ? 16 : 8;
    scfg.synth_nrx = rx_b ? 2 : 1;
    scfg.synth_realtime = !fast;
    scfg.file_realtime = !fast;

    source_t *src = source_open(src_spec, &scfg);
    if (!src)
        return 1;
    dcfg.fs_nom = src->cfg.fs;
    const int nrx = src->nlanes < 2 ? 1 : (rx_b ? 2 : 1);

    ring_t ring;
    /* About 0.3 s of buffering at 25 MS/s (256 x 32768 samples, 16 MiB). */
    uint32_t nblocks = (uint32_t)(0.3 * src->cfg.fs / src->cfg.block_samples);
    if (nblocks < 16) nblocks = 16;
    if (nblocks > 1024) nblocks = 1024;
    if (ring_init(&ring, nblocks, src->cfg.block_samples, !src->realtime)) {
        LOGE("out of memory");
        return 1;
    }

    if (!strcmp(cmd, "probe")) {
        int rc = run_probe(src, &ring, scfg.seconds > 0 ? scfg.seconds : 2.0, &dcfg);
        source_close(src);
        ring_free(&ring);
        return rc;
    }
    if (!strcmp(cmd, "record")) {
        if (!rec_path) {
            LOGE("record needs -O FILE");
            return 2;
        }
        int rc = run_record(src, &ring, scfg.seconds > 0 ? scfg.seconds : 5.0, rec_path);
        source_close(src);
        ring_free(&ring);
        return rc;
    }

    /* run */
    frame_pub_t pub;
    if (frame_pub_init(&pub, deint, osd)) {
        LOGE("out of memory");
        return 1;
    }
    vcfg.lossless = !src->realtime;
    vcfg.realtime = src->realtime;
    if (!div_given && nrx == 2)
        vcfg.mode = DIV_FUSE;
    pipeline_cfg_t pc = {.dec = dcfg, .div = vcfg, .nrx = nrx, .realtime = src->realtime};
    if (!chan_uart && (!strcmp(src_spec, "smi") || !strcmp(src_spec, "smidev")))
        chan_uart = "/dev/serial0";
    chanlink_t chan;
    chanlink_init(&chan, chan_uart);
    pipeline_t *pl = pipeline_new(&pc, &ring, &pub);
    if (!pl)
        return 1;
    app_t app = {.pl = pl, .nrx = nrx, .dv = pipeline_diversity(pl), .src = src, .mode = pipeline_mode(pl),
                 .t_last = now_s(), .chan_on = chan.path != NULL};

    out_y4m_t *y4m = NULL;
    outputs_t out = {.pub = &pub};
    out.dc = (drm_cfg_t){.mode = hdmi_mode, .stretch = stretch, .av_std = av_std, .av_progressive = av_prog};
#ifdef HAVE_JPEG
    out.hc = (http_cfg_t){.port = web_port, .fps = web_fps, .quality = jpeg_q, .status = status_json, .status_ctx = &app};
#endif
    bool want_hdmi = false, want_av = false, want_web = false, use_jumpers = false;
    char *olist = strdup(outputs);
    for (char *tok = strtok(olist, ","); tok; tok = strtok(NULL, ",")) {
        if (!strcmp(tok, "hdmi"))
            want_hdmi = true;
        else if (!strcmp(tok, "av"))
            want_av = true;
        else if (!strcmp(tok, "web"))
            want_web = true;
        else if (!strcmp(tok, "jumpers"))
            use_jumpers = true;
        else if (!strncmp(tok, "y4m:", 4))
            y4m = out_y4m_open(tok + 4, &pub);
        else if (strcmp(tok, "none") != 0)
            LOGW("unknown output '%s'", tok);
    }
    free(olist);
    if (want_hdmi && want_av) {
        LOGW("hdmi and av cannot run together: using av");
        want_hdmi = false;
    }
    jumper_io_t jio = {.fd = -1};
    jumper_filter_t jf = {0};
    if (use_jumpers && !jumper_sim && jumper_open(&jio)) {
        LOGW("jumpers not readable: using hdmi and web");
        use_jumpers = false;
        want_hdmi = !want_av;
        want_web = true;
    }
    if (use_jumpers)
        poll_jumpers(&out, &jio, jumper_sim, &jf, want_web);
    else
        set_outputs(&out, want_av ? VID_AV : want_hdmi ? VID_HDMI : VID_NONE, want_web);

    if (pipeline_start(pl) || source_start(src, &ring)) {
        LOGE("cannot start capture");
        atomic_store(&g_running, false);
    }
    LOGI("running: %d receiver(s), diversity %s, %s deinterlace", nrx, mode_name(app.mode),
         deint == DEINT_BOB ? "bob" : "weave");

    double next_print = now_s() + 2.0;
    while (!frame_pub_is_eof(&pub)) {
        usleep(100000);
        update_fps(&app);
        if (use_jumpers)
            poll_jumpers(&out, &jio, jumper_sim, &jf, want_web);
        const double t_now = now_s();
        chanlink_poll(&chan, t_now);
        const int ch = chanlink_channel(&chan, t_now);
        if (ch != atomic_load(&app.channel)) {
            atomic_store(&app.channel, ch);
            if (ch)
                LOGI("channel R%d", ch);
            else
                LOGW("channel link: nothing heard for %.0f s", CHAN_STALE_S);
        }
        if (now_s() >= next_print) {
            next_print += 2.0;
            print_status(&app);
        }
        if (!atomic_load(&g_running) && src->realtime)
            break;
    }

    atomic_store(&g_running, false);
    source_stop(src);
    ring_set_eof(&ring);
    pipeline_join(pl);
    frame_pub_set_eof(&pub);
    set_outputs(&out, VID_NONE, false);
    jumper_close(&jio);
    chanlink_close(&chan);
    out_y4m_close(y4m);

    div_stats_t *ds = diversity_stats(app.dv);
    LOGI("done: %llu fields out (%llu paired, %llu single), lines A %llu / B %llu / blended %llu, dropped %llu",
         (unsigned long long)atomic_load(&ds->fields_out), (unsigned long long)atomic_load(&ds->fields_paired),
         (unsigned long long)atomic_load(&ds->fields_solo), (unsigned long long)atomic_load(&ds->lines_from[0]),
         (unsigned long long)atomic_load(&ds->lines_from[1]), (unsigned long long)atomic_load(&ds->lines_from[2]),
         (unsigned long long)atomic_load(&ds->dropped));
    for (int r = 0; r < nrx; r++) {
        decoder_stats_t *st = decoder_stats(pipeline_rx(pl, r));
        LOGD("rx%c: decoder fields %llu, vsyncs %llu (rejected %llu, flywheel %llu), lock losses %llu, lines %llu, "
             "sync lost %llu times (%llu lines on the flywheel, %llu re-syncs)",
             'A' + r, (unsigned long long)atomic_load(&st->fields), (unsigned long long)atomic_load(&st->vsyncs),
             (unsigned long long)atomic_load(&st->vsync_rejected), (unsigned long long)atomic_load(&st->vsync_flywheel),
             (unsigned long long)atomic_load(&st->lock_losses), (unsigned long long)atomic_load(&st->lines),
             (unsigned long long)atomic_load(&st->fly_entries), (unsigned long long)atomic_load(&st->fly_lines),
             (unsigned long long)atomic_load(&st->fly_resyncs));
    }
    if (pipeline_fusion(pl)) {
        fusion_stats_t *fs = fusion_stats(pipeline_fusion(pl));
        decoder_stats_t *st = decoder_stats(pipeline_fused(pl));
        LOGD("fused: skipped fields %llu, sync lost %llu times (%llu lines on the flywheel, %llu re-syncs)",
             (unsigned long long)atomic_load(&st->fields_skipped), (unsigned long long)atomic_load(&st->fly_entries),
             (unsigned long long)atomic_load(&st->fly_lines), (unsigned long long)atomic_load(&st->fly_resyncs));
        LOGD("fused: fields %llu, vsyncs %llu (rejected %llu, flywheel %llu), lock losses %llu, snr %.1f dB, "
             "B delay %.3f us, outliers A %llu B %llu, realigns %llu",
             (unsigned long long)atomic_load(&st->fields), (unsigned long long)atomic_load(&st->vsyncs),
             (unsigned long long)atomic_load(&st->vsync_rejected), (unsigned long long)atomic_load(&st->vsync_flywheel),
             (unsigned long long)atomic_load(&st->lock_losses), atomic_load(&st->snr_db), atomic_load(&fs->delay_us),
             (unsigned long long)atomic_load(&fs->outliers[0]), (unsigned long long)atomic_load(&fs->outliers[1]),
             (unsigned long long)atomic_load(&fs->realigns));
    }
    pipeline_free(pl);
    frame_pub_destroy(&pub);
    source_close(src);
    ring_free(&ring);
    return 0;
}
