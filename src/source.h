/* Capture sources: produce 16-bit SMI-format sample blocks into a ring. */
#pragma once

#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>

#include "ring.h"
#include "video_std.h"

typedef struct {
    /* common */
    double fs;              /* nominal sample rate (Hz) */
    double seconds;         /* stop after this much captured signal (0 = forever) */
    uint32_t block_samples;

    /* smi */
    const char *smi_dev;
    int smi_width;          /* 16 (two receivers) or 8 (receiver A only) */
    int smi_setup, smi_strobe, smi_hold, smi_pace;
    bool smi_swap_pairs;    /* swap the two 16-bit samples of each 32-bit FIFO word (smidev) */
    int dma_chan;           /* smi: DMA channel used by neither Linux nor firmware (default 7) */
    bool dma_force;         /* smi: skip the channel-in-use checks (after a crash we did not record) */

    /* synth */
    video_std_t synth_std;
    int synth_nrx;
    float synth_noise[2];   /* gaussian noise, DAC codes rms */
    float synth_clicks[2];  /* FM clicks per line */
    float synth_sync[2];    /* sync depth in codes (C5 nominal ~18) */
    double synth_fade_period;  /* s; >0: A and B fade alternately */
    float synth_fade_noise;    /* extra noise at the bottom of a fade */
    double synth_gap_every;    /* s; >0: emulate SMI read gaps */
    double synth_gap_us;
    float synth_skew;       /* probability-ish width of bit-2 skew window (0..1 of a code) */
    double synth_fakevs;    /* fake vsync-like pulse runs per second, line-aligned (decoder robustness) */
    double synth_delay_us;  /* receiver B sees the video this much later (C5 pipeline differences) */
    double synth_ppm[2];       /* C5 clock error of each receiver */
    double synth_phase[2];     /* C5 clock phase, codes */
    double synth_delay2_us;    /* with synth_delay_step: the delay after that time (a C5 restarted) */
    double synth_delay_step;   /* s; >0: switch to synth_delay2_us here */
    double synth_off_from[2], synth_off_to[2];  /* s; receiver outputs noise only in [from, to) */
    double synth_dip_every;    /* s; >0: short, independent deep fades (dips) per receiver */
    double synth_dip_len;      /* s; dip duration */
    float synth_dip_noise;     /* noise during a dip */
    float synth_dip_keep;      /* fraction of the video left during a dip (0: noise only, no sync) */
    double synth_jump_at;      /* s; >0: from here on the video of all receivers is synth_jump_us later */
    double synth_jump_us;      /* (a camera/VTX restart: new line and field phase) */
    double synth_std_at;       /* s; >0: from here on the other standard (PAL <-> NTSC: another camera) */
    bool synth_realtime;

    /* file */
    const char *path;
    bool file_realtime;
} source_cfg_t;

typedef struct source source_t;

struct source {
    const char *kind;
    source_cfg_t cfg;
    ring_t *ring;
    pthread_t th;
    bool running;
    bool realtime;      /* drops when consumers lag */
    int nlanes;         /* receivers present in the stream */
    void *priv;
    void *(*thread_fn)(void *);
    void (*close_fn)(source_t *);
    /* stats */
    atomic_uint_fast64_t samples, blocks, gaps, short_reads;
    _Atomic double rate_meas;   /* measured samples/s (wall clock) */
    _Atomic double gap_us_avg;
};

void source_cfg_default(source_cfg_t *c);
/* spec: "smi", "synth", "synth:ntsc", "file:PATH" */
source_t *source_open(const char *spec, const source_cfg_t *cfg);
int source_start(source_t *s, ring_t *ring);
void source_stop(source_t *s);
void source_close(source_t *s);

/* per-kind constructors */
int source_synth_init(source_t *s);
int source_file_init(source_t *s);
int source_smi_init(source_t *s);      /* "smidev": kernel /dev/smi reads (gaps between reads) */
int source_smidma_init(source_t *s);   /* "smi": gapless SMI + DMA ring */

/* Raw recording format (record / file source). */
#define RAW_MAGIC "C5DIVRAW"
typedef struct {
    char magic[8];
    uint32_t version;
    uint32_t sample_bits;
    double fs;
    uint32_t nlanes;
    uint32_t reserved[5];
} raw_header_t;

typedef struct {
    uint64_t start;
    uint64_t gap_est;
    uint32_t n;
    uint32_t flags;
} raw_block_header_t;
