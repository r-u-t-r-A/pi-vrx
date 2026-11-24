#include <stdlib.h>
#include <string.h>

#include "common.h"
#include "outputs.h"

struct out_y4m {
    FILE *f;
    int w, h;
    bool header;
    uint64_t frames, skipped;
};

static void y4m_listener(void *ctx, const frame_t *fr)
{
    out_y4m_t *o = ctx;
    if (!o->header) {
        const std_params_t *sp = std_params(fr->std);
        o->w = fr->w;
        o->h = fr->h;
        fprintf(o->f, "YUV4MPEG2 W%d H%d F%d:%d Ip A%s C420jpeg\n", fr->w, fr->h, sp->field_rate_num,
                sp->field_rate_den, sp->pal ? "16:15" : "8:9");
        o->header = true;
    }
    if (fr->w != o->w || fr->h != o->h) {
        o->skipped++;
        return;
    }
    fputs("FRAME\n", o->f);
    fwrite(fr->y, 1, (size_t)fr->w * fr->h, o->f);
    fwrite(fr->u, 1, (size_t)(fr->w / 2) * (fr->h / 2), o->f);
    fwrite(fr->v, 1, (size_t)(fr->w / 2) * (fr->h / 2), o->f);
    o->frames++;
}

out_y4m_t *out_y4m_open(const char *path, frame_pub_t *pub)
{
    out_y4m_t *o = calloc(1, sizeof(*o));
    if (!o)
        return NULL;
    o->f = strcmp(path, "-") == 0 ? stdout : fopen(path, "wb");
    if (!o->f) {
        LOGE("cannot write %s", path);
        free(o);
        return NULL;
    }
    frame_pub_add_listener(pub, y4m_listener, o);
    LOGI("y4m output: %s", path);
    return o;
}

void out_y4m_close(out_y4m_t *o)
{
    if (!o)
        return;
    LOGI("y4m: %llu frames written%s", (unsigned long long)o->frames,
         o->skipped ? " (some skipped: frame size changed)" : "");
    if (o->f != stdout)
        fclose(o->f);
    else
        fflush(stdout);
    free(o);
}
