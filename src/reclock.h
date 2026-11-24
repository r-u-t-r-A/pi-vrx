/* C5 sample clock recovery.
 *
 * The C5 holds each DAC code for 50 ns (20 MS/s); the Pi samples at 25 MS/s
 * from its own clock, so one sample in five repeats a code. Decoding the raw
 * samples folds the hold images into the picture (24.43 MHz lands on
 * 0.57 MHz) and includes samples taken while the bits were changing. This
 * stage tracks which of the five sampling phases repeats (it slips by one
 * every ~1.7 ms with the real clocks), and takes each code exactly once, from
 * the sample nearest the middle of its hold time.
 *
 * Output: one sample per C5 code, index k. Code k is centred on capture
 * sample 1.25 k - 0.5 - 0.25 D, D being the tracked repeat phase.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define RECLOCK_RATIO 1.25      /* capture samples per C5 code */

typedef struct reclock reclock_t;

/* Only for captures at 1.25 x the C5 rate (25 MS/s). */
bool reclock_supported(double fs);
reclock_t *reclock_new(int lane);
void reclock_free(reclock_t *rc);
/* Process a capture block. Returns the codes (valid until the next call),
 * their count and the index of the first one. */
const uint8_t *reclock_feed(reclock_t *rc, const uint16_t *s, uint32_t n, uint64_t start, bool discont, uint32_t *n_out,
                            uint64_t *k_start);
/* Tracking state: capture position = 1.25 k - 0.5 - 0.25 (d0 + d1 * (pos - t0)). */
typedef struct {
    bool locked;
    double d0, d1, t0;
    float ppm;          /* C5 clock relative to the capture clock */
    float contrast;     /* repeat-phase contrast, 0..1 */
} reclock_state_t;
void reclock_state(const reclock_t *rc, reclock_state_t *st);
double reclock_to_capture(const reclock_state_t *st, double k);
double reclock_from_capture(const reclock_state_t *st, double pos);
