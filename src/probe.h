#pragma once

#include "decoder.h"
#include "ring.h"
#include "source.h"

/* Capture for `seconds` and print wiring / signal diagnostics per lane. */
int run_probe(source_t *src, ring_t *ring, double seconds, const decoder_cfg_t *dcfg);
/* Capture raw samples to a file replayable with --source file:PATH. */
int run_record(source_t *src, ring_t *ring, double seconds, const char *path);
