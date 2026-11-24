/* HDMI or composite AV output via DRM/KMS atomic modesetting (vc4 on the Pi).
 *
 * AV uses the analog TV encoder at 13.5 MHz, 720 pixels per line: the grid the
 * decoder resamples to, so pixels pass through 1:1. The standard follows the
 * camera (576i50 PAL / 480i60 NTSC) unless forced.
 *
 * Prefers a YUV420 framebuffer so the hardware scaler (HVS) does colour
 * conversion and scaling for free; falls back to XRGB8888 with CPU
 * conversion. For PAL it asks for 720x576@50 (576p) and for NTSC
 * 720x480@60 when the display offers them, so no rate conversion happens;
 * otherwise it uses the preferred mode and scales.
 * Page flips are non-blocking; the newest frame always wins.
 */
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <drm_fourcc.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include "common.h"
#include "outputs.h"

#define NBUF 3

typedef struct {
    uint32_t handle, fb, pitch;
    uint64_t size;
    uint8_t *map;
} dbuf_t;

struct out_drm {
    drm_cfg_t cfg;
    frame_pub_t *pub;
    pthread_t th;
    atomic_bool run;

    int fd;
    bool ready;
    uint32_t conn_id, crtc_id, plane_id;
    int crtc_idx;
    drmModeModeInfo mode;
    uint32_t mode_blob;
    bool yuv;
    int fw, fh;
    dbuf_t buf[NBUF];
    int front, pending_buf;
    bool flip_pending;
    int dx, dy, dw, dh;

    uint32_t p_fb_id, p_crtc_id, p_src_x, p_src_y, p_src_w, p_src_h, p_crtc_x, p_crtc_y, p_crtc_w, p_crtc_h;
    uint32_t c_conn_crtc, c_mode_id, c_active;
    uint32_t c_tv_mode;         /* AV: colour standard property and value */
    uint64_t tv_mode;
    struct { uint32_t conn, prop; } steal[8];   /* connectors to take off our CRTC */
    int nsteal;
    const char *name;
    int std_diff;               /* consecutive frames of another standard */

    frame_t tmp;
    uint64_t flips, busy;
};

static uint32_t prop_id(int fd, uint32_t obj, uint32_t type, const char *name, uint64_t *value)
{
    drmModeObjectProperties *props = drmModeObjectGetProperties(fd, obj, type);
    uint32_t id = 0;
    if (!props)
        return 0;
    for (uint32_t i = 0; i < props->count_props && !id; i++) {
        drmModePropertyRes *p = drmModeGetProperty(fd, props->props[i]);
        if (p && strcmp(p->name, name) == 0) {
            id = p->prop_id;
            if (value)
                *value = props->prop_values[i];
        }
        drmModeFreeProperty(p);
    }
    drmModeFreeObjectProperties(props);
    return id;
}

static bool plane_has_format(drmModePlane *pl, uint32_t fmt)
{
    for (uint32_t i = 0; i < pl->count_formats; i++)
        if (pl->formats[i] == fmt)
            return true;
    return false;
}

static void teardown(out_drm_t *o)
{
    for (int i = 0; i < NBUF; i++) {
        dbuf_t *b = &o->buf[i];
        if (b->fb)
            drmModeRmFB(o->fd, b->fb);
        if (b->map)
            munmap(b->map, b->size);
        if (b->handle) {
            struct drm_mode_destroy_dumb dd = {.handle = b->handle};
            drmIoctl(o->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd);
        }
        memset(b, 0, sizeof(*b));
    }
    if (o->mode_blob)
        drmModeDestroyPropertyBlob(o->fd, o->mode_blob);
    o->mode_blob = 0;
    if (o->fd >= 0)
        close(o->fd);
    o->fd = -1;
    o->ready = false;
    o->flip_pending = false;
}

static int create_buffers(out_drm_t *o, int W, int H)
{
    for (int i = 0; i < NBUF; i++) {
        dbuf_t *b = &o->buf[i];
        struct drm_mode_create_dumb cd = {0};
        cd.width = (uint32_t)W;
        cd.height = o->yuv ? (uint32_t)(H + H / 2) : (uint32_t)H;
        cd.bpp = o->yuv ? 8 : 32;
        if (drmIoctl(o->fd, DRM_IOCTL_MODE_CREATE_DUMB, &cd))
            return -1;
        b->handle = cd.handle;
        b->pitch = cd.pitch;
        b->size = cd.size;
        struct drm_mode_map_dumb md = {.handle = b->handle};
        if (drmIoctl(o->fd, DRM_IOCTL_MODE_MAP_DUMB, &md))
            return -1;
        b->map = mmap(NULL, b->size, PROT_READ | PROT_WRITE, MAP_SHARED, o->fd, (off_t)md.offset);
        if (b->map == MAP_FAILED) {
            b->map = NULL;
            return -1;
        }
        uint32_t handles[4] = {0}, pitches[4] = {0}, offsets[4] = {0};
        if (o->yuv) {
            memset(b->map, 16, (size_t)b->pitch * H);
            memset(b->map + (size_t)b->pitch * H, 128, (size_t)b->pitch * (H / 2));
            handles[0] = handles[1] = handles[2] = b->handle;
            pitches[0] = b->pitch;
            pitches[1] = pitches[2] = b->pitch / 2;
            offsets[0] = 0;
            offsets[1] = b->pitch * (uint32_t)H;
            offsets[2] = offsets[1] + (b->pitch / 2) * (uint32_t)(H / 2);
            if (drmModeAddFB2(o->fd, (uint32_t)W, (uint32_t)H, DRM_FORMAT_YUV420, handles, pitches, offsets,
                              &b->fb, 0))
                return -1;
        } else {
            memset(b->map, 0, b->size);
            handles[0] = b->handle;
            pitches[0] = b->pitch;
            if (drmModeAddFB2(o->fd, (uint32_t)W, (uint32_t)H, DRM_FORMAT_XRGB8888, handles, pitches, offsets,
                              &b->fb, 0))
                return -1;
        }
    }
    o->fw = W;
    o->fh = H;
    return 0;
}

static int commit(out_drm_t *o, int bi, uint32_t flags, bool modeset)
{
    drmModeAtomicReq *req = drmModeAtomicAlloc();
    if (!req)
        return -ENOMEM;
    if (modeset) {
        for (int i = 0; i < o->nsteal; i++)
            drmModeAtomicAddProperty(req, o->steal[i].conn, o->steal[i].prop, 0);
        if (o->c_tv_mode)
            drmModeAtomicAddProperty(req, o->conn_id, o->c_tv_mode, o->tv_mode);
        drmModeAtomicAddProperty(req, o->conn_id, o->c_conn_crtc, o->crtc_id);
        drmModeAtomicAddProperty(req, o->crtc_id, o->c_mode_id, o->mode_blob);
        drmModeAtomicAddProperty(req, o->crtc_id, o->c_active, 1);
    }
    drmModeAtomicAddProperty(req, o->plane_id, o->p_fb_id, o->buf[bi].fb);
    drmModeAtomicAddProperty(req, o->plane_id, o->p_crtc_id, o->crtc_id);
    drmModeAtomicAddProperty(req, o->plane_id, o->p_src_x, 0);
    drmModeAtomicAddProperty(req, o->plane_id, o->p_src_y, 0);
    drmModeAtomicAddProperty(req, o->plane_id, o->p_src_w, (uint64_t)o->fw << 16);
    drmModeAtomicAddProperty(req, o->plane_id, o->p_src_h, (uint64_t)o->fh << 16);
    drmModeAtomicAddProperty(req, o->plane_id, o->p_crtc_x, (uint64_t)o->dx);
    drmModeAtomicAddProperty(req, o->plane_id, o->p_crtc_y, (uint64_t)o->dy);
    drmModeAtomicAddProperty(req, o->plane_id, o->p_crtc_w, (uint64_t)o->dw);
    drmModeAtomicAddProperty(req, o->plane_id, o->p_crtc_h, (uint64_t)o->dh);
    int ret = drmModeAtomicCommit(o->fd, req, flags, o);
    drmModeAtomicFree(req);
    return ret;
}

static bool is_analog(uint32_t type)
{
    return type == DRM_MODE_CONNECTOR_Composite || type == DRM_MODE_CONNECTOR_SVIDEO || type == DRM_MODE_CONNECTOR_TV;
}

/* Value of an enum property of the connector, by name. */
static bool enum_value(int fd, uint32_t prop, const char *name, uint64_t *value)
{
    drmModePropertyRes *p = drmModeGetProperty(fd, prop);
    bool ok = false;
    for (int i = 0; p && i < p->count_enums && !ok; i++)
        if (!strcmp(p->enums[i].name, name)) {
            *value = p->enums[i].value;
            ok = true;
        }
    drmModeFreeProperty(p);
    return ok;
}

/* AV: the 13.5 MHz mode and the colour standard must belong together. */
static bool pick_av_mode(out_drm_t *o, drmModeConnector *c, video_std_t std)
{
    const bool pal = std != STD_NTSC;
    const int want_h = (pal ? 576 : 480) / (o->cfg.av_progressive ? 2 : 1);
    for (int i = 0; i < c->count_modes; i++) {
        const drmModeModeInfo *m = &c->modes[i];
        if (m->hdisplay == 720 && m->vdisplay == want_h && m->htotal == (pal ? 864 : 858) &&
            !(m->flags & DRM_MODE_FLAG_INTERLACE) == o->cfg.av_progressive) {
            o->mode = *m;
            o->c_tv_mode = prop_id(o->fd, o->conn_id, DRM_MODE_OBJECT_CONNECTOR, "TV mode", NULL);
            if (!o->c_tv_mode)      /* kernels before 6.2 */
                o->c_tv_mode = prop_id(o->fd, o->conn_id, DRM_MODE_OBJECT_CONNECTOR, "mode", NULL);
            if (!o->c_tv_mode || !enum_value(o->fd, o->c_tv_mode, pal ? "PAL" : "NTSC", &o->tv_mode)) {
                LOGW("av: no TV standard property, output may be in the wrong colour system");
                o->c_tv_mode = 0;
            }
            return true;
        }
    }
    LOGE("av: the driver offers no %s mode", pal ? "PAL" : "NTSC");
    return false;
}

static bool pick_mode(out_drm_t *o, drmModeConnector *c, int W, int H, video_std_t std)
{
    o->c_tv_mode = 0;
    if (o->cfg.av)
        return pick_av_mode(o, c, o->cfg.av_std != STD_UNKNOWN ? o->cfg.av_std : std);
    int mw = 0, mh = 0, mr = 0;
    if (o->cfg.mode && strcmp(o->cfg.mode, "auto") != 0 && sscanf(o->cfg.mode, "%dx%d@%d", &mw, &mh, &mr) >= 2) {
        for (int i = 0; i < c->count_modes; i++) {
            drmModeModeInfo *m = &c->modes[i];
            if (m->hdisplay == mw && m->vdisplay == mh && (mr == 0 || (int)m->vrefresh == mr) &&
                !(m->flags & DRM_MODE_FLAG_INTERLACE)) {
                o->mode = *m;
                return true;
            }
        }
        LOGW("hdmi: mode %s not offered by the display, using auto", o->cfg.mode);
    }
    int want_r = std == STD_NTSC ? 60 : 50;
    for (int i = 0; i < c->count_modes; i++) {
        drmModeModeInfo *m = &c->modes[i];
        if (m->hdisplay == W && m->vdisplay == H && (int)m->vrefresh == want_r &&
            !(m->flags & DRM_MODE_FLAG_INTERLACE)) {
            o->mode = *m;
            return true;
        }
    }
    for (int i = 0; i < c->count_modes; i++)
        if (c->modes[i].type & DRM_MODE_TYPE_PREFERRED) {
            o->mode = c->modes[i];
            return true;
        }
    if (c->count_modes > 0) {
        o->mode = c->modes[0];
        return true;
    }
    return false;
}

static int setup(out_drm_t *o, int W, int H, video_std_t std)
{
    drmModeRes *res = NULL;
    drmModeConnector *conn = NULL;
    for (int i = 0; i < 8 && !conn; i++) {
        char path[32];
        snprintf(path, sizeof(path), "/dev/dri/card%d", i);
        int fd = open(path, O_RDWR | O_CLOEXEC);
        if (fd < 0)
            continue;
        res = drmModeGetResources(fd);
        if (!res || res->count_connectors == 0 || drmSetClientCap(fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1) ||
            drmSetClientCap(fd, DRM_CLIENT_CAP_ATOMIC, 1)) {
            if (res)
                drmModeFreeResources(res);
            res = NULL;
            close(fd);
            continue;
        }
        for (int c = 0; c < res->count_connectors && !conn; c++) {
            drmModeConnector *cc = drmModeGetConnector(fd, res->connectors[c]);
            /* Nothing can detect an analog monitor: its status is "unknown". */
            const bool ok = cc && cc->count_modes > 0 &&
                            (o->cfg.av ? is_analog(cc->connector_type) && cc->connection != DRM_MODE_DISCONNECTED
                                       : !is_analog(cc->connector_type) && cc->connection == DRM_MODE_CONNECTED);
            if (ok)
                conn = cc;
            else if (cc)
                drmModeFreeConnector(cc);
        }
        if (conn) {
            o->fd = fd;
            LOGI("%s: using %s", o->name, path);
        } else {
            drmModeFreeResources(res);
            res = NULL;
            close(fd);
        }
    }
    if (!conn) {
        if (o->cfg.av)
            LOGW("av: no composite connector (config.txt needs dtoverlay=vc4-kms-v3d,composite)");
        else
            LOGD("hdmi: no connected display");
        return -1;
    }
    int rc = -1;
    o->conn_id = conn->connector_id;
    if (!pick_mode(o, conn, W, H, std))
        goto out;

    /* CRTC */
    o->crtc_id = 0;
    if (conn->encoder_id) {
        drmModeEncoder *e = drmModeGetEncoder(o->fd, conn->encoder_id);
        if (e) {
            o->crtc_id = e->crtc_id;
            drmModeFreeEncoder(e);
        }
    }
    for (int i = 0; i < conn->count_encoders && !o->crtc_id; i++) {
        drmModeEncoder *e = drmModeGetEncoder(o->fd, conn->encoders[i]);
        if (!e)
            continue;
        for (int k = 0; k < res->count_crtcs; k++)
            if (e->possible_crtcs & (1u << k)) {
                o->crtc_id = res->crtcs[k];
                break;
            }
        drmModeFreeEncoder(e);
    }
    o->crtc_idx = -1;
    for (int k = 0; k < res->count_crtcs; k++)
        if (res->crtcs[k] == o->crtc_id)
            o->crtc_idx = k;
    if (o->crtc_idx < 0) {
        LOGE("drm: no CRTC for connector");
        goto out;
    }

    /* HDMI and AV share a CRTC: take it away from the other one. */
    o->nsteal = 0;
    for (int c = 0; c < res->count_connectors && o->nsteal < 8; c++) {
        uint64_t on = 0;
        const uint32_t id = res->connectors[c];
        const uint32_t prop = prop_id(o->fd, id, DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID", &on);
        if (id != o->conn_id && prop && on == o->crtc_id) {
            o->steal[o->nsteal].conn = id;
            o->steal[o->nsteal++].prop = prop;
        }
    }

    /* Plane: primary with YUV420 preferred, then primary with XRGB8888. */
    drmModePlaneRes *pres = drmModeGetPlaneResources(o->fd);
    uint32_t best = 0;
    int best_score = -1;
    bool best_yuv = false;
    for (uint32_t i = 0; pres && i < pres->count_planes; i++) {
        drmModePlane *pl = drmModeGetPlane(o->fd, pres->planes[i]);
        if (!pl)
            continue;
        if (pl->possible_crtcs & (1u << o->crtc_idx)) {
            uint64_t type = 0;
            prop_id(o->fd, pl->plane_id, DRM_MODE_OBJECT_PLANE, "type", &type);
            bool yuv = plane_has_format(pl, DRM_FORMAT_YUV420);
            bool rgb = plane_has_format(pl, DRM_FORMAT_XRGB8888);
            int score = -1;
            if (type == DRM_PLANE_TYPE_PRIMARY && yuv) score = 3;
            else if (type == DRM_PLANE_TYPE_PRIMARY && rgb) score = 2;
            if (score > best_score) {
                best_score = score;
                best = pl->plane_id;
                best_yuv = score == 3;
            }
        }
        drmModeFreePlane(pl);
    }
    if (pres)
        drmModeFreePlaneResources(pres);
    if (best_score < 0) {
        LOGE("drm: no usable plane");
        goto out;
    }
    o->plane_id = best;
    o->yuv = best_yuv;

    o->p_fb_id = prop_id(o->fd, o->plane_id, DRM_MODE_OBJECT_PLANE, "FB_ID", NULL);
    o->p_crtc_id = prop_id(o->fd, o->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_ID", NULL);
    o->p_src_x = prop_id(o->fd, o->plane_id, DRM_MODE_OBJECT_PLANE, "SRC_X", NULL);
    o->p_src_y = prop_id(o->fd, o->plane_id, DRM_MODE_OBJECT_PLANE, "SRC_Y", NULL);
    o->p_src_w = prop_id(o->fd, o->plane_id, DRM_MODE_OBJECT_PLANE, "SRC_W", NULL);
    o->p_src_h = prop_id(o->fd, o->plane_id, DRM_MODE_OBJECT_PLANE, "SRC_H", NULL);
    o->p_crtc_x = prop_id(o->fd, o->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_X", NULL);
    o->p_crtc_y = prop_id(o->fd, o->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_Y", NULL);
    o->p_crtc_w = prop_id(o->fd, o->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_W", NULL);
    o->p_crtc_h = prop_id(o->fd, o->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_H", NULL);
    o->c_conn_crtc = prop_id(o->fd, o->conn_id, DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID", NULL);
    o->c_mode_id = prop_id(o->fd, o->crtc_id, DRM_MODE_OBJECT_CRTC, "MODE_ID", NULL);
    o->c_active = prop_id(o->fd, o->crtc_id, DRM_MODE_OBJECT_CRTC, "ACTIVE", NULL);

    if (create_buffers(o, W, H)) {
        if (o->yuv) {
            LOGW("drm: YUV420 framebuffer failed, using XRGB8888");
            for (int i = 0; i < NBUF; i++) {
                if (o->buf[i].fb) drmModeRmFB(o->fd, o->buf[i].fb);
                if (o->buf[i].map) munmap(o->buf[i].map, o->buf[i].size);
                if (o->buf[i].handle) {
                    struct drm_mode_destroy_dumb dd = {.handle = o->buf[i].handle};
                    drmIoctl(o->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd);
                }
                memset(&o->buf[i], 0, sizeof(dbuf_t));
            }
            o->yuv = false;
            if (create_buffers(o, W, H))
                goto out;
        } else {
            goto out;
        }
    }

    /* Destination rectangle */
    int MW = o->mode.hdisplay, MH = o->mode.vdisplay;
    /* 720-pixel SD modes are 4:3 as a whole (their pixels are not square). */
    if (o->cfg.stretch || o->cfg.av || (MW == 720 && (MH == 576 || MH == 480))) {
        o->dx = o->dy = 0;
        o->dw = MW;
        o->dh = MH;
    } else if (MW * 3 > MH * 4) {
        o->dh = MH;
        o->dw = (MH * 4 / 3) & ~1;
        o->dx = (MW - o->dw) / 2;
        o->dy = 0;
    } else {
        o->dw = MW;
        o->dh = (MW * 3 / 4) & ~1;
        o->dx = 0;
        o->dy = (MH - o->dh) / 2;
    }

    if (drmModeCreatePropertyBlob(o->fd, &o->mode, sizeof(o->mode), &o->mode_blob))
        goto out;
    int ret = commit(o, 0, DRM_MODE_ATOMIC_ALLOW_MODESET, true);
    if (ret) {
        LOGE("%s: modeset failed: %s", o->name, strerror(-ret));
        goto out;
    }
    o->front = 0;
    o->ready = true;
    o->std_diff = 0;
    LOGI("%s: %dx%d%s@%u, %s plane, picture %dx%d -> %dx%d+%d+%d", o->name, MW, MH,
         o->mode.flags & DRM_MODE_FLAG_INTERLACE ? "i" : "", o->mode.vrefresh, o->yuv ? "YUV420" : "XRGB8888", W, H, o->dw, o->dh, o->dx, o->dy);
    rc = 0;
out:
    drmModeFreeConnector(conn);
    drmModeFreeResources(res);
    if (rc)
        teardown(o);
    return rc;
}

static void fill(out_drm_t *o, int bi, const frame_t *f)
{
    dbuf_t *b = &o->buf[bi];
    const int W = f->w, H = f->h;
    if (o->yuv) {
        for (int y = 0; y < H; y++)
            memcpy(b->map + (size_t)y * b->pitch, f->y + (size_t)y * W, (size_t)W);
        uint8_t *u = b->map + (size_t)b->pitch * H;
        uint8_t *v = u + (size_t)(b->pitch / 2) * (H / 2);
        for (int y = 0; y < H / 2; y++) {
            memcpy(u + (size_t)y * (b->pitch / 2), f->u + (size_t)y * (W / 2), (size_t)W / 2);
            memcpy(v + (size_t)y * (b->pitch / 2), f->v + (size_t)y * (W / 2), (size_t)W / 2);
        }
        return;
    }
    for (int y = 0; y < H; y++) {
        uint32_t *row = (uint32_t *)(b->map + (size_t)y * b->pitch);
        const uint8_t *yy = f->y + (size_t)y * W;
        const uint8_t *uu = f->u + (size_t)(y / 2) * (W / 2);
        const uint8_t *vv = f->v + (size_t)(y / 2) * (W / 2);
        for (int x = 0; x < W; x++) {
            int c = 298 * (yy[x] - 16), d = uu[x / 2] - 128, e = vv[x / 2] - 128;
            int r = (c + 409 * e + 128) >> 8;
            int g = (c - 100 * d - 208 * e + 128) >> 8;
            int bl = (c + 516 * d + 128) >> 8;
            row[x] = (uint32_t)(clampi(r, 0, 255) << 16 | clampi(g, 0, 255) << 8 | clampi(bl, 0, 255));
        }
    }
}

static void flip_handler(int fd, unsigned int seq, unsigned int sec, unsigned int usec, void *data)
{
    out_drm_t *o = data;
    o->flip_pending = false;
    o->front = o->pending_buf;
    o->flips++;
}

static void *drm_thread(void *arg)
{
    out_drm_t *o = arg;
    thread_begin("drm");
    uint64_t seq = 0;
    double next_try = 0;
    drmEventContext ev = {.version = 2, .page_flip_handler = flip_handler};

    while (atomic_load(&o->run)) {
        if (o->ready && o->flip_pending) {
            struct pollfd p = {.fd = o->fd, .events = POLLIN};
            if (poll(&p, 1, 100) > 0)
                drmHandleEvent(o->fd, &ev);
            continue;
        }
        if (!frame_pub_get(o->pub, &o->tmp, &seq, 100)) {
            if (frame_pub_is_eof(o->pub))
                break;
            continue;
        }
        if (o->ready && (o->tmp.w != o->fw || o->tmp.h != o->fh)) {
            /* Standard changed: rebuild, but not on a short misdetection (an
             * analog monitor takes a second to re-sync). */
            if (++o->std_diff < 20)
                continue;
            teardown(o);
        }
        o->std_diff = 0;
        if (!o->ready) {
            if (now_s() < next_try)
                continue;
            if (setup(o, o->tmp.w, o->tmp.h, o->tmp.std)) {
                next_try = now_s() + 2.0;
                continue;
            }
        }
        int bi = (o->front + 1) % NBUF;
        fill(o, bi, &o->tmp);
        int ret = commit(o, bi, DRM_MODE_ATOMIC_NONBLOCK | DRM_MODE_PAGE_FLIP_EVENT, false);
        if (ret == 0) {
            o->flip_pending = true;
            o->pending_buf = bi;
        } else if (ret == -EBUSY) {
            o->busy++;
        } else {
            LOGW("%s: commit failed (%s), re-initialising", o->name, strerror(-ret));
            teardown(o);
            next_try = now_s() + 1.0;
        }
    }
    if (o->fd >= 0)
        teardown(o);
    return NULL;
}

out_drm_t *out_drm_start(const drm_cfg_t *cfg, frame_pub_t *pub)
{
    out_drm_t *o = calloc(1, sizeof(*o));
    if (!o)
        return NULL;
    o->cfg = *cfg;
    o->name = cfg->av ? "av" : "hdmi";
    o->pub = pub;
    o->fd = -1;
    if (frame_alloc(&o->tmp)) {
        free(o);
        return NULL;
    }
    atomic_store(&o->run, true);
    if (pthread_create(&o->th, NULL, drm_thread, o)) {
        frame_release(&o->tmp);
        free(o);
        return NULL;
    }
    LOGI("%s output enabled", o->name);
    return o;
}

void out_drm_stop(out_drm_t *o)
{
    if (!o)
        return;
    atomic_store(&o->run, false);
    pthread_join(o->th, NULL);
    frame_release(&o->tmp);
    free(o);
}
