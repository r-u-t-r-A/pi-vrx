/* Synthetic composite video generator (test signal source).
 *
 * Produces a standards-shaped PAL or NTSC composite signal (full vertical
 * interval with broad/equalising pulses, colour burst with PAL V-switch,
 * 75% colour bars, grey ramp and a moving box) in luma units:
 * blank = 0, white = 1, sync tip = -1/sync_ratio.
 */
#pragma once

#include "video_std.h"

typedef struct {
    const std_params_t *sp;
    double fsc_w;   /* 2*pi*fsc */
} cvbs_synth_t;

void cvbs_synth_init(cvbs_synth_t *s, video_std_t std);
/* Signal value at absolute time t (seconds). */
double cvbs_synth_eval(const cvbs_synth_t *s, double t);

/* Picture used by the generator (for tests): x,y in [0,1). */
void cvbs_synth_picture(double x, double y, long frame, double *Y, double *U, double *V);
