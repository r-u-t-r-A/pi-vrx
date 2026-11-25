/* Output tests without a display: out_drm.c against a fake libdrm that looks
 * like vc4 on a Zero 2W with composite enabled, and the jumper debounce. */
#include <errno.h>
#include <pty.h>
#include <termios.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <drm_fourcc.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include "chanlink.h"
#include "common.h"
#include "frame.h"
#include "jumpers.h"
#include "outputs.h"

int g_verbose = 0;
atomic_bool g_running = true;

static int failures;
#define CHECK(c, ...) do { const bool ok_ = (c); if (!ok_) failures++; printf("  %s", ok_ ? "ok:   " : "FAIL: "); \
                           printf(__VA_ARGS__); printf("\n"); } while (0)

/* ------------------------------------------------------------ fake libdrm */

enum { CRTC = 50, ENC_HDMI = 60, ENC_VEC = 61, HDMI = 70, VEC = 71, PLANE = 80 };
enum { P_CRTC_ID = 1, P_MODE_ID, P_ACTIVE, P_TV, P_TV_LEGACY, P_TYPE, P_FB, P_PCRTC, P_SX, P_SY, P_SW, P_SH, P_CX, P_CY,
       P_CW, P_CH, P_MAX };
static const char *prop_names[P_MAX] = {"", "CRTC_ID", "MODE_ID", "ACTIVE", "TV mode", "mode", "type", "FB_ID", "CRTC_ID",
                                        "SRC_X", "SRC_Y", "SRC_W", "SRC_H", "CRTC_X", "CRTC_Y", "CRTC_W", "CRTC_H"};
static bool have_vec = true, hdmi_plugged = true, legacy_tv;
static drmModeModeInfo blob;
static uint64_t crtc_of[2];     /* HDMI, VEC */
static atomic_int nmodeset, nflip;
static struct { uint64_t tv, hdmi_crtc, vec_crtc, cx, cy, cw, ch, sh; bool tv_set, hdmi_set; drmModeModeInfo mode; } last;
static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static void *flip_data;
static struct { uint32_t obj, prop; uint64_t val; } rq[64];
static int nrq;

int __real_open(const char *p, int fl, ...);
int __wrap_open(const char *p, int fl, ...)
{
    if (strncmp(p, "/dev/dri/", 9) != 0) {
        va_list a;
        va_start(a, fl);
        int m = va_arg(a, int);
        va_end(a);
        return __real_open(p, fl, m);
    }
    if (strcmp(p, "/dev/dri/card0") != 0) {
        errno = ENOENT;
        return -1;
    }
    int fd = memfd_create("drm", 0);
    if (ftruncate(fd, 8 << 20)) return -1;
    return fd;
}
int drmSetClientCap(int fd, uint64_t c, uint64_t v) { return 0; }
int drmIoctl(int fd, unsigned long rq_, void *arg)
{
    if (rq_ == DRM_IOCTL_MODE_CREATE_DUMB) {
        struct drm_mode_create_dumb *c = arg;
        c->handle = 1;
        c->pitch = c->width * c->bpp / 8;
        c->size = (uint64_t)c->pitch * c->height;
    } else if (rq_ == DRM_IOCTL_MODE_MAP_DUMB) {
        ((struct drm_mode_map_dumb *)arg)->offset = 0;
    }
    return 0;
}
drmModeResPtr drmModeGetResources(int fd)
{
    drmModeRes *r = calloc(1, sizeof(*r));
    r->count_connectors = have_vec ? 2 : 1;
    r->connectors = calloc(2, 4);
    r->connectors[0] = have_vec ? VEC : HDMI;   /* composite listed first */
    r->connectors[1] = HDMI;
    r->count_crtcs = 1;
    r->crtcs = calloc(1, 4);
    r->crtcs[0] = CRTC;
    return r;
}
void drmModeFreeResources(drmModeResPtr r) { free(r->connectors); free(r->crtcs); free(r); }
static void mode(drmModeModeInfo *m, int w, int h, int ht, int r, int fl, int pref)
{
    m->hdisplay = (uint16_t)w; m->vdisplay = (uint16_t)h; m->htotal = (uint16_t)ht; m->vrefresh = (uint32_t)r;
    m->flags = (uint32_t)fl; m->type = pref ? DRM_MODE_TYPE_PREFERRED : DRM_MODE_TYPE_DRIVER;
}
drmModeConnectorPtr drmModeGetConnector(int fd, uint32_t id)
{
    drmModeConnector *c = calloc(1, sizeof(*c));
    c->connector_id = id;
    c->connector_type = id == HDMI ? DRM_MODE_CONNECTOR_HDMIA : DRM_MODE_CONNECTOR_Composite;
    c->connection = id == HDMI ? (hdmi_plugged ? DRM_MODE_CONNECTED : DRM_MODE_DISCONNECTED) : DRM_MODE_UNKNOWNCONNECTION;
    c->modes = calloc(4, sizeof(drmModeModeInfo));
    if (id == VEC) {
        c->count_modes = 4;
        mode(&c->modes[0], 720, 480, 858, 60, DRM_MODE_FLAG_INTERLACE, 1);
        mode(&c->modes[1], 720, 576, 864, 50, DRM_MODE_FLAG_INTERLACE, 0);
        mode(&c->modes[2], 720, 240, 858, 60, 0, 0);
        mode(&c->modes[3], 720, 288, 864, 50, 0, 0);
    } else if (hdmi_plugged) {
        c->count_modes = 3;
        mode(&c->modes[0], 1920, 1080, 2200, 60, 0, 1);
        mode(&c->modes[1], 720, 576, 864, 50, DRM_MODE_FLAG_INTERLACE, 0);
        mode(&c->modes[2], 720, 576, 864, 50, 0, 0);
    }
    c->count_encoders = 1;
    c->encoders = calloc(1, 4);
    c->encoders[0] = id == HDMI ? ENC_HDMI : ENC_VEC;
    return c;
}
void drmModeFreeConnector(drmModeConnectorPtr c) { if (c) { free(c->modes); free(c->encoders); free(c); } }
drmModeEncoderPtr drmModeGetEncoder(int fd, uint32_t id)
{
    drmModeEncoder *e = calloc(1, sizeof(*e));
    e->encoder_id = id;
    e->possible_crtcs = 1;
    return e;
}
void drmModeFreeEncoder(drmModeEncoderPtr e) { free(e); }
drmModePlaneResPtr drmModeGetPlaneResources(int fd)
{
    drmModePlaneRes *r = calloc(1, sizeof(*r));
    r->count_planes = 1;
    r->planes = calloc(1, 4);
    r->planes[0] = PLANE;
    return r;
}
void drmModeFreePlaneResources(drmModePlaneResPtr r) { free(r->planes); free(r); }
drmModePlanePtr drmModeGetPlane(int fd, uint32_t id)
{
    drmModePlane *p = calloc(1, sizeof(*p));
    p->plane_id = id;
    p->possible_crtcs = 1;
    p->count_formats = 1;
    p->formats = calloc(1, 4);
    p->formats[0] = DRM_FORMAT_YUV420;
    return p;
}
void drmModeFreePlane(drmModePlanePtr p) { free(p->formats); free(p); }
drmModeObjectPropertiesPtr drmModeObjectGetProperties(int fd, uint32_t id, uint32_t type)
{
    drmModeObjectProperties *o = calloc(1, sizeof(*o));
    o->props = calloc(P_MAX, 4);
    o->prop_values = calloc(P_MAX, 8);
    int n = 0;
#define ADD(p, v) do { o->props[n] = p; o->prop_values[n++] = v; } while (0)
    if (type == DRM_MODE_OBJECT_CONNECTOR) {
        ADD(P_CRTC_ID, crtc_of[id == VEC]);
        if (id == VEC) {
            if (!legacy_tv) ADD(P_TV, 0);
            ADD(P_TV_LEGACY, 0);
        }
    } else if (type == DRM_MODE_OBJECT_CRTC) {
        ADD(P_MODE_ID, 0);
        ADD(P_ACTIVE, 0);
    } else {
        ADD(P_TYPE, DRM_PLANE_TYPE_PRIMARY);
        for (int p = P_FB; p < P_MAX; p++) ADD(p, 0);
    }
    o->count_props = (uint32_t)n;
    return o;
}
void drmModeFreeObjectProperties(drmModeObjectPropertiesPtr o) { free(o->props); free(o->prop_values); free(o); }
drmModePropertyPtr drmModeGetProperty(int fd, uint32_t id)
{
    drmModePropertyRes *p = calloc(1, sizeof(*p));
    p->prop_id = id;
    snprintf(p->name, sizeof(p->name), "%s", prop_names[id]);
    if (id == P_TV || id == P_TV_LEGACY) {      /* different numbering in the two */
        static const char *n[] = {"NTSC", "NTSC-443", "NTSC-J", "PAL"};
        p->count_enums = 4;
        p->enums = calloc(4, sizeof(*p->enums));
        for (int i = 0; i < 4; i++) {
            snprintf(p->enums[i].name, sizeof(p->enums[i].name), "%s", n[i]);
            p->enums[i].value = (uint64_t)(id == P_TV ? 10 + i : 20 + i);
        }
    }
    return p;
}
void drmModeFreeProperty(drmModePropertyPtr p) { if (p) { free(p->enums); free(p); } }
int drmModeAddFB2(int fd, uint32_t w, uint32_t h, uint32_t f, const uint32_t a[4], const uint32_t b[4], const uint32_t c[4],
                  uint32_t *id, uint32_t fl) { *id = 5; return 0; }
int drmModeRmFB(int fd, uint32_t id) { return 0; }
int drmModeCreatePropertyBlob(int fd, const void *d, size_t n, uint32_t *id) { memcpy(&blob, d, sizeof(blob)); *id = 9; return 0; }
int drmModeDestroyPropertyBlob(int fd, uint32_t id) { return 0; }
drmModeAtomicReqPtr drmModeAtomicAlloc(void) { nrq = 0; return (void *)&nrq; }
void drmModeAtomicFree(drmModeAtomicReqPtr r) {}
int drmModeAtomicAddProperty(drmModeAtomicReqPtr r, uint32_t o, uint32_t p, uint64_t v)
{
    rq[nrq].obj = o; rq[nrq].prop = p; rq[nrq++].val = v;
    return nrq;
}
int drmModeAtomicCommit(int fd, drmModeAtomicReqPtr r, uint32_t flags, void *data)
{
    pthread_mutex_lock(&mu);
    if (flags & DRM_MODE_ATOMIC_ALLOW_MODESET) {
        uint64_t c[2] = {crtc_of[0], crtc_of[1]};
        memset(&last, 0, sizeof(last));
        for (int i = 0; i < nrq; i++) {
            if (rq[i].prop == P_CRTC_ID && rq[i].obj == HDMI) c[0] = rq[i].val;
            if (rq[i].prop == P_CRTC_ID && rq[i].obj == VEC) c[1] = rq[i].val;
            if (rq[i].prop == P_TV || rq[i].prop == P_TV_LEGACY) { last.tv = rq[i].val; last.tv_set = true; }
            if (rq[i].prop == P_CX) last.cx = rq[i].val;
            if (rq[i].prop == P_CY) last.cy = rq[i].val;
            if (rq[i].prop == P_CW) last.cw = rq[i].val;
            if (rq[i].prop == P_CH) last.ch = rq[i].val;
            if (rq[i].prop == P_SH) last.sh = rq[i].val >> 16;
        }
        if (c[0] && c[1]) {         /* vc4: both on one CRTC is refused */
            pthread_mutex_unlock(&mu);
            return -EINVAL;
        }
        crtc_of[0] = c[0];
        crtc_of[1] = c[1];
        last.mode = blob;
        atomic_fetch_add(&nmodeset, 1);
    } else {
        flip_data = data;
        atomic_fetch_add(&nflip, 1);
    }
    pthread_mutex_unlock(&mu);
    return 0;
}
int drmHandleEvent(int fd, drmEventContextPtr e)
{
    usleep(2000);
    e->page_flip_handler(fd, 0, 0, 0, flip_data);
    return 0;
}

/* ------------------------------------------------------------------ tests */

static void blank(frame_pub_t *pub, video_std_t std, int n)
{
    for (int i = 0; i < n; i++) {
        frame_pub_put_blank(pub, std, NULL);
        usleep(6000);
    }
}

static out_drm_t *start(frame_pub_t *pub, drm_cfg_t c, video_std_t std)
{
    atomic_store(&nmodeset, 0);
    atomic_store(&nflip, 0);
    out_drm_t *o = out_drm_start(&c, pub);
    blank(pub, std, 10);
    return o;
}

#define FULL(w, h) (last.cx == 0 && last.cy == 0 && last.cw == (w) && last.ch == (h))

static int feed_chan(chan_parser_t *p, const char *s)
{
    int r = 0;
    for (; *s; s++) {
        const int c = chan_feed(p, (uint8_t)*s);
        if (c)
            r = c;
    }
    return r;
}

static void test_channel(void)
{
    printf("[channel messages]\n");
    chan_parser_t p = {0};
    CHECK(feed_chan(&p, "$C5,R3,5732*14\r\n") == 3, "the firmware test's message");
    CHECK(feed_chan(&p, "$C5,R3,5732*15\r\n") == 0 && feed_chan(&p, "$C5,R3,5769*1A\r\n") == 0, "bad checksum or frequency refused");
    CHECK(feed_chan(&p, "ESP-ROM:esp32c5\r\n$C5,R$C5,R3,5732*14\r\n") == 3, "after boot log and a torn message");
    CHECK(feed_chan(&p, "$AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\r\n") == 0, "overlong line");

    printf("[channel link on a pseudo terminal]\n");
    int m, sl;
    char name[64];
    if (openpty(&m, &sl, name, NULL, NULL)) {
        CHECK(false, "openpty");
        return;
    }
    close(sl);
    chanlink_t c;
    chanlink_init(&c, name);
    CHECK(!chanlink_poll(&c, 0) && c.fd >= 0 && chanlink_channel(&c, 0) == 0, "opens, channel unknown");
    (void)!write(m, "$C5,R3,57", 9);
    usleep(20000);
    CHECK(!chanlink_poll(&c, 1.0), "half a message");
    (void)!write(m, "32*14\r\n", 7);
    usleep(20000);
    CHECK(chanlink_poll(&c, 1.1) && chanlink_channel(&c, 1.1) == 3, "rest of it: R3");
    CHECK(chanlink_channel(&c, 12.9) == 3 && chanlink_channel(&c, 13.2) == 0, "unknown again after 12 s of silence");
    close(m);
    chanlink_poll(&c, 14);
    chanlink_poll(&c, 15);
    CHECK(c.channel == 3, "port gone: no crash");
    chanlink_close(&c);
    chanlink_init(&c, "/dev/does-not-exist");
    CHECK(!chanlink_poll(&c, 0) && c.fd < 0, "missing port is not fatal");
    chanlink_init(&c, "none");
    CHECK(!chanlink_poll(&c, 0) && !c.path, "off");
}

int main(void)
{
    test_channel();
    frame_pub_t pub;
    frame_pub_init(&pub, DEINT_BOB, false);
    out_drm_t *o;

    printf("[jumper debounce]\n");
    jumper_filter_t jf = {0};
    CHECK(jumper_filter(&jf, (jumpers_t){.av = true}, 0) && jf.state.av, "first reading counts at once");
    bool any = false;
    for (int i = 1; i < 20; i++)
        any |= jumper_filter(&jf, (jumpers_t){.av = i & 1}, i * 0.1);
    CHECK(!any && jf.state.av, "bouncing contact changes nothing");
    int changes = 0;
    for (int i = 20; i < 40; i++)
        changes += jumper_filter(&jf, (jumpers_t){.av = false, .wifi = true}, i * 0.1);
    CHECK(changes == 1 && !jf.state.av && jf.state.wifi, "stable change reported once");

    printf("[AV, PAL camera; HDMI plugged in and active]\n");
    crtc_of[0] = CRTC;
    crtc_of[1] = 0;
    o = start(&pub, (drm_cfg_t){.av = true}, STD_PAL);
    CHECK(nmodeset == 1 && crtc_of[1] == CRTC && crtc_of[0] == 0, "composite took the CRTC from HDMI");
    CHECK(last.mode.vdisplay == 576 && last.mode.htotal == 864 && (last.mode.flags & DRM_MODE_FLAG_INTERLACE), "576i");
    CHECK(last.tv_set && last.tv == 13, "colour system PAL");
    CHECK(FULL(720, 576), "720x576 unscaled, full frame");
    CHECK(nflip > 3, "%d flips", nflip);

    printf("[AV: standard changes]\n");
    blank(&pub, STD_NTSC, 8);
    blank(&pub, STD_PAL, 5);
    CHECK(nmodeset == 1, "short misdetection ignored");
    blank(&pub, STD_NTSC, 45);
    CHECK(nmodeset == 2 && last.mode.vdisplay == 480 && last.mode.htotal == 858 && last.tv == 10, "480i, NTSC");
    CHECK(FULL(720, 480), "720x480 unscaled, full frame");
    out_drm_stop(o);

    printf("[back to HDMI]\n");
    o = start(&pub, (drm_cfg_t){.mode = "auto"}, STD_PAL);
    CHECK(nmodeset == 1 && crtc_of[0] == CRTC && crtc_of[1] == 0, "HDMI took the CRTC, composite (listed first) ignored");
    CHECK(last.mode.vdisplay == 576 && !(last.mode.flags & DRM_MODE_FLAG_INTERLACE) && !last.tv_set && FULL(720, 576), "576p50");
    out_drm_stop(o);
    o = start(&pub, (drm_cfg_t){.mode = "1920x1080@60"}, STD_PAL);
    CHECK(last.cx == 240 && last.cw == 1440 && last.ch == 1080, "1080p: 4:3 pillarbox");
    out_drm_stop(o);

    printf("[HDMI unplugged]\n");
    hdmi_plugged = false;
    o = start(&pub, (drm_cfg_t){.mode = "auto"}, STD_PAL);
    CHECK(nmodeset == 0, "composite is not used by mistake");
    out_drm_stop(o);
    hdmi_plugged = true;

    printf("[AV options]\n");
    o = start(&pub, (drm_cfg_t){.av = true, .av_std = STD_PAL}, STD_NTSC);
    CHECK(last.mode.vdisplay == 576 && last.tv == 13 && last.sh == 480 && FULL(720, 576), "forced PAL, NTSC camera scaled");
    out_drm_stop(o);
    o = start(&pub, (drm_cfg_t){.av = true, .av_progressive = true}, STD_NTSC);
    CHECK(last.mode.vdisplay == 240 && !(last.mode.flags & DRM_MODE_FLAG_INTERLACE) && FULL(720, 240), "240p");
    out_drm_stop(o);
    legacy_tv = true;
    o = start(&pub, (drm_cfg_t){.av = true}, STD_PAL);
    CHECK(last.tv == 23, "older kernel: legacy colour system property");
    out_drm_stop(o);

    printf("[composite not enabled in config.txt]\n");
    have_vec = false;
    crtc_of[0] = CRTC;
    crtc_of[1] = 0;
    o = start(&pub, (drm_cfg_t){.av = true}, STD_PAL);
    CHECK(nmodeset == 0 && crtc_of[0] == CRTC, "HDMI left alone");
    out_drm_stop(o);

    frame_pub_set_eof(&pub);
    frame_pub_destroy(&pub);
    printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASSED", failures);
    return failures != 0;
}
