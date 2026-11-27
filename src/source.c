#include "source.h"

#include <stdlib.h>
#include <string.h>

#include "common.h"

void source_cfg_default(source_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->fs = 25e6;
    c->seconds = 0;
    c->block_samples = 1u << 15;   /* 32768 samples = 1.3 ms at 25 MS/s */
    c->dma_chan = 7;
    c->smi_dev = "/dev/smi";
    c->smi_width = 16;
    /* 125 MHz SMI clock assumed: 1+3+1 = 5 cycles = 40 ns = 25 MS/s */
    c->smi_setup = 1;
    c->smi_strobe = 3;
    c->smi_hold = 1;
    c->smi_pace = 0;
    c->synth_std = STD_PAL;
    c->synth_nrx = 2;
    c->synth_noise[0] = 0.6f;
    c->synth_noise[1] = 0.6f;
    c->synth_clicks[0] = 0.0f;
    c->synth_clicks[1] = 0.0f;
    c->synth_sync[0] = 18.0f;
    c->synth_sync[1] = 16.0f;
    c->synth_fade_period = 0;
    c->synth_fade_noise = 8.0f;
    c->synth_gap_every = 0;
    c->synth_gap_us = 30;
    c->synth_skew = 0.05f;
    c->synth_dip_keep = 0.3f;
    c->synth_realtime = true;
    c->file_realtime = true;
}

source_t *source_open(const char *spec, const source_cfg_t *cfg)
{
    source_t *s = calloc(1, sizeof(*s));
    if (!s)
        return NULL;
    s->cfg = *cfg;
    int rc;
    if (strcmp(spec, "smi") == 0 || strncmp(spec, "smi:", 4) == 0) {
        if (spec[3] == ':')
            s->cfg.smi_dev = spec + 4;
        s->kind = "smi";
        rc = source_smidma_init(s);
    } else if (strcmp(spec, "smidev") == 0) {
        s->kind = "smidev";
        rc = source_smi_init(s);
    } else if (strcmp(spec, "synth") == 0 || strncmp(spec, "synth:", 6) == 0) {
        if (spec[5] == ':') {
            const char *o = spec + 6;
            if (strcmp(o, "ntsc") == 0)
                s->cfg.synth_std = STD_NTSC;
            else if (strcmp(o, "pal") == 0)
                s->cfg.synth_std = STD_PAL;
            else {
                LOGE("unknown synth option '%s' (pal|ntsc)", o);
                free(s);
                return NULL;
            }
        }
        s->kind = "synth";
        rc = source_synth_init(s);
    } else if (strncmp(spec, "file:", 5) == 0) {
        s->cfg.path = spec + 5;
        s->kind = "file";
        rc = source_file_init(s);
    } else {
        LOGE("unknown source '%s' (smi | smidev | synth[:pal|ntsc] | file:PATH)", spec);
        free(s);
        return NULL;
    }
    if (rc) {
        free(s);
        return NULL;
    }
    return s;
}

int source_start(source_t *s, ring_t *ring)
{
    s->ring = ring;
    s->running = true;
    if (pthread_create(&s->th, NULL, s->thread_fn, s)) {
        s->running = false;
        return -1;
    }
    return 0;
}

void source_stop(source_t *s)
{
    if (!s->running)
        return;
    s->running = false;
    pthread_join(s->th, NULL);
}

void source_close(source_t *s)
{
    if (!s)
        return;
    source_stop(s);
    if (s->close_fn)
        s->close_fn(s);
    free(s);
}
