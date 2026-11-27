/* Output frames (YUV 4:2:0, 720 x 576/480) and their distribution. */
#pragma once

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

#include "field.h"

typedef enum { DEINT_BOB = 0, DEINT_WEAVE = 1 } deint_t;

typedef struct {
    int w, h;              /* luma size; chroma is w/2 x h/2 */
    video_std_t std;
    uint64_t seq;
    double t_wall;         /* when the newest field finished decoding */
    bool no_signal;
    uint8_t *y, *u, *v;
} frame_t;

typedef struct {
    int nrx;
    bool locked[2];
    bool fly[2];           /* flywheel: no sync, timing held */
    float snr_db[2];
    float sel_frac[2];     /* share of lines taken from each receiver (last field) */
} osd_info_t;

typedef void (*frame_listener_fn)(void *ctx, const frame_t *f);

#define FRAME_MAX_LISTENERS 4

typedef struct {
    pthread_mutex_t mu;
    pthread_cond_t cv;
    frame_t fr;
    uint64_t seq;
    bool eof;
    deint_t deint;
    bool osd;
    field_t *prev;         /* previous field (weave) */
    bool have_prev;
    frame_listener_fn listeners[FRAME_MAX_LISTENERS];
    void *listener_ctx[FRAME_MAX_LISTENERS];
    int nlisteners;
} frame_pub_t;

int frame_alloc(frame_t *f);   /* allocates for the largest (PAL) size */
void frame_release(frame_t *f);
void frame_copy(frame_t *dst, const frame_t *src);

int frame_pub_init(frame_pub_t *p, deint_t deint, bool osd);
void frame_pub_destroy(frame_pub_t *p);
/* Synchronous listener, called for every frame with the publisher locked
 * (used by lossless sinks such as the Y4M writer). */
void frame_pub_add_listener(frame_pub_t *p, frame_listener_fn fn, void *ctx);
void frame_pub_put_field(frame_pub_t *p, const field_t *f, const osd_info_t *osd);
void frame_pub_put_blank(frame_pub_t *p, video_std_t std, const osd_info_t *osd);
void frame_pub_set_eof(frame_pub_t *p);
/* Wait for a frame newer than *seq and copy it. Returns false on timeout/EOF. */
bool frame_pub_get(frame_pub_t *p, frame_t *dst, uint64_t *seq, int timeout_ms);
bool frame_pub_is_eof(frame_pub_t *p);
