#pragma once

#include <stddef.h>

#include "frame.h"

/* Y4M file writer (every frame, synchronous). */
typedef struct out_y4m out_y4m_t;
out_y4m_t *out_y4m_open(const char *path, frame_pub_t *pub);
void out_y4m_close(out_y4m_t *o);

/* HDMI through DRM/KMS. */
typedef struct {
    const char *mode;   /* "auto" or "WxH@HZ" */
    bool stretch;       /* fill the screen instead of 4:3 */
} drm_cfg_t;
typedef struct out_drm out_drm_t;
out_drm_t *out_drm_start(const drm_cfg_t *cfg, frame_pub_t *pub);
void out_drm_stop(out_drm_t *o);

/* Web viewer: MJPEG stream + status page. */
typedef size_t (*status_json_fn)(void *ctx, char *buf, size_t n);
typedef struct {
    int port;
    int fps;
    int quality;
    status_json_fn status;
    void *status_ctx;
} http_cfg_t;
typedef struct out_http out_http_t;
out_http_t *out_http_start(const http_cfg_t *cfg, frame_pub_t *pub);
void out_http_stop(out_http_t *o);
