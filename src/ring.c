#include "ring.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

int ring_init(ring_t *r, uint32_t nblocks, uint32_t block_samples, bool lossless)
{
    memset(r, 0, sizeof(*r));
    r->nblocks = nblocks;
    r->block_samples = block_samples;
    r->lossless = lossless;
    r->blocks = calloc(nblocks, sizeof(ring_block_t));
    r->mem = aligned_alloc(64, (size_t)nblocks * block_samples * sizeof(uint16_t));
    if (!r->blocks || !r->mem)
        return -1;
    for (uint32_t i = 0; i < nblocks; i++)
        r->blocks[i].data = r->mem + (size_t)i * block_samples;
    pthread_mutex_init(&r->mu, NULL);
    pthread_cond_init(&r->cv_data, NULL);
    pthread_cond_init(&r->cv_space, NULL);
    return 0;
}

void ring_free(ring_t *r)
{
    free(r->blocks);
    free(r->mem);
    pthread_mutex_destroy(&r->mu);
    pthread_cond_destroy(&r->cv_data);
    pthread_cond_destroy(&r->cv_space);
}

static uint64_t min_consumer_seq(ring_t *r)
{
    uint64_t m = UINT64_MAX;
    for (int i = 0; i < r->nconsumers; i++)
        if (r->cons_active[i] && r->cons_seq[i] < m)
            m = r->cons_seq[i];
    return m;
}

ring_block_t *ring_acquire(ring_t *r)
{
    pthread_mutex_lock(&r->mu);
    if (r->lossless) {
        /* Keep one block of slack: a consumer may still hold seq cons_seq. */
        while (min_consumer_seq(r) != UINT64_MAX && r->head - min_consumer_seq(r) >= r->nblocks - 1)
            pthread_cond_wait(&r->cv_space, &r->mu);
    }
    ring_block_t *b = &r->blocks[r->head % r->nblocks];
    b->seq = r->head;
    b->flags = 0;
    b->gap_est = 0;
    pthread_mutex_unlock(&r->mu);
    return b;
}

void ring_publish(ring_t *r, ring_block_t *b)
{
    pthread_mutex_lock(&r->mu);
    r->head = b->seq + 1;
    pthread_cond_broadcast(&r->cv_data);
    pthread_mutex_unlock(&r->mu);
}

void ring_set_eof(ring_t *r)
{
    pthread_mutex_lock(&r->mu);
    r->eof = true;
    pthread_cond_broadcast(&r->cv_data);
    pthread_mutex_unlock(&r->mu);
}

int ring_add_consumer(ring_t *r)
{
    pthread_mutex_lock(&r->mu);
    int id = -1;
    if (r->nconsumers < RING_MAX_CONSUMERS) {
        id = r->nconsumers++;
        r->cons_seq[id] = r->head;
        r->cons_active[id] = true;
    }
    pthread_mutex_unlock(&r->mu);
    return id;
}

void ring_remove_consumer(ring_t *r, int id)
{
    pthread_mutex_lock(&r->mu);
    r->cons_active[id] = false;
    pthread_cond_broadcast(&r->cv_space);
    pthread_mutex_unlock(&r->mu);
}

ring_status_t ring_next(ring_t *r, int id, const ring_block_t **out, bool *overrun, int timeout_ms)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout_ms / 1000;
    ts.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000L;
    }
    *overrun = false;
    pthread_mutex_lock(&r->mu);
    while (r->cons_seq[id] >= r->head) {
        if (r->eof) {
            pthread_mutex_unlock(&r->mu);
            return RING_EOF;
        }
        if (pthread_cond_timedwait(&r->cv_data, &r->mu, &ts) == ETIMEDOUT) {
            pthread_mutex_unlock(&r->mu);
            return RING_TIMEOUT;
        }
    }
    /* Realtime: if we are about to be overwritten, jump close to head.
     * Lossless: the producer waits for us (it may get nblocks - 1 ahead,
     * which is not an overrun), so never skip. */
    if (!r->lossless && r->head - r->cons_seq[id] > r->nblocks - 2) {
        r->cons_seq[id] = r->head - 1;
        *overrun = true;
    }
    *out = &r->blocks[r->cons_seq[id] % r->nblocks];
    pthread_mutex_unlock(&r->mu);
    return RING_OK;
}

void ring_release(ring_t *r, int id)
{
    pthread_mutex_lock(&r->mu);
    r->cons_seq[id]++;
    if (r->lossless)
        pthread_cond_broadcast(&r->cv_space);
    pthread_mutex_unlock(&r->mu);
}
