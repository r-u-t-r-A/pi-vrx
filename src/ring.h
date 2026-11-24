/* Single-producer / multi-consumer block ring for raw SMI samples.
 *
 * Each sample is a uint16: bits 0..5 = receiver A (SMI SD0..SD5),
 * bits 8..13 = receiver B (SMI SD8..SD13). Each block carries its virtual
 * sample index so consumers can detect gaps (between SMI reads, overruns).
 *
 * Realtime mode: the producer never blocks; a consumer that falls more than
 * the ring size behind skips ahead and sees a discontinuity.
 * Lossless mode (file / synth sources): the producer waits for the slowest
 * consumer.
 */
#pragma once

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

#define RING_F_DISCONT 1u   /* samples before this block are missing */

#define RING_MAX_CONSUMERS 8

typedef struct {
    uint64_t seq;
    uint64_t start;     /* virtual index of first sample */
    uint64_t gap_est;   /* estimated missing samples before this block (if DISCONT) */
    uint32_t n;
    uint32_t flags;
    double t_wall;      /* capture time (CLOCK_MONOTONIC), end of block */
    uint16_t *data;
} ring_block_t;

typedef struct {
    ring_block_t *blocks;
    uint16_t *mem;
    uint32_t nblocks;
    uint32_t block_samples;
    bool lossless;
    pthread_mutex_t mu;
    pthread_cond_t cv_data;   /* producer -> consumers */
    pthread_cond_t cv_space;  /* consumers -> producer (lossless) */
    uint64_t head;            /* number of published blocks */
    bool eof;
    int nconsumers;
    uint64_t cons_seq[RING_MAX_CONSUMERS];
    bool cons_active[RING_MAX_CONSUMERS];
} ring_t;

typedef enum { RING_OK = 0, RING_TIMEOUT = 1, RING_EOF = 2 } ring_status_t;

int ring_init(ring_t *r, uint32_t nblocks, uint32_t block_samples, bool lossless);
void ring_free(ring_t *r);

/* Producer */
ring_block_t *ring_acquire(ring_t *r);       /* block to fill (may wait in lossless mode) */
void ring_publish(ring_t *r, ring_block_t *b);
void ring_set_eof(ring_t *r);

/* Consumer */
int ring_add_consumer(ring_t *r);
void ring_remove_consumer(ring_t *r, int id);
/* Wait for the next block. On overrun the consumer skips ahead and
 * *overrun is set. The block stays valid until ring_release(). */
ring_status_t ring_next(ring_t *r, int id, const ring_block_t **out, bool *overrun, int timeout_ms);
void ring_release(ring_t *r, int id);
