/* Replays a raw capture written by `c5div record`. */
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "common.h"
#include "source.h"

typedef struct {
    FILE *f;
    raw_header_t hdr;
} file_priv_t;

static void *file_thread(void *arg)
{
    source_t *s = arg;
    file_priv_t *p = s->priv;
    const double fs = p->hdr.fs > 0 ? p->hdr.fs : s->cfg.fs;
    const double t0 = now_s();
    uint64_t first = UINT64_MAX;
    const double total = s->cfg.seconds > 0 ? s->cfg.seconds * fs : 0;

    while (s->running && atomic_load(&g_running)) {
        raw_block_header_t bh;
        if (fread(&bh, sizeof(bh), 1, p->f) != 1)
            break;
        if (first == UINT64_MAX)
            first = bh.start;
        if (total > 0 && (double)(bh.start - first) >= total)
            break;
        uint32_t left = bh.n;
        uint64_t pos = bh.start;
        uint32_t flags = bh.flags;
        while (left > 0) {
            ring_block_t *b = ring_acquire(s->ring);
            uint32_t n = left < s->ring->block_samples ? left : s->ring->block_samples;
            if (fread(b->data, sizeof(uint16_t), n, p->f) != n) {
                left = 0;
                break;
            }
            b->start = pos;
            b->n = n;
            b->flags = flags;
            b->gap_est = flags ? bh.gap_est : 0;
            b->t_wall = now_s();
            ring_publish(s->ring, b);
            atomic_fetch_add(&s->samples, n);
            atomic_fetch_add(&s->blocks, 1);
            pos += n;
            left -= n;
            flags = 0;
        }
        if (s->cfg.file_realtime) {
            double due = t0 + (double)(pos - first) / fs;
            double now = now_s();
            if (due > now)
                usleep((useconds_t)((due - now) * 1e6));
        }
    }
    ring_set_eof(s->ring);
    return NULL;
}

static void file_close(source_t *s)
{
    file_priv_t *p = s->priv;
    if (p->f)
        fclose(p->f);
    free(p);
}

int source_file_init(source_t *s)
{
    file_priv_t *p = calloc(1, sizeof(*p));
    if (!p)
        return -1;
    p->f = fopen(s->cfg.path, "rb");
    if (!p->f) {
        LOGE("cannot open %s", s->cfg.path);
        free(p);
        return -1;
    }
    if (fread(&p->hdr, sizeof(p->hdr), 1, p->f) != 1 || memcmp(p->hdr.magic, RAW_MAGIC, 8) != 0) {
        LOGE("%s is not a c5div raw capture", s->cfg.path);
        fclose(p->f);
        free(p);
        return -1;
    }
    s->priv = p;
    s->thread_fn = file_thread;
    s->close_fn = file_close;
    s->realtime = s->cfg.file_realtime;
    s->nlanes = (int)p->hdr.nlanes;
    s->cfg.fs = p->hdr.fs;
    LOGI("file source: %s, %.3f MS/s, %d lane(s)", s->cfg.path, p->hdr.fs / 1e6, s->nlanes);
    return 0;
}
