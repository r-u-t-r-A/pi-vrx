#include "video_std.h"

static const std_params_t k_pal = {
    .std = STD_PAL,
    .name = "PAL",
    .line_us = 64.0,
    .fsc_hz = 4433618.75,
    .lines_per_field = 312.5,
    .sync_ratio = 0.7 / 0.3,
    .burst_nom = 0.15 / 0.7,
    .h_active_us = 132.0 / 13.5,
    .burst_start_us = 5.8, .burst_end_us = 7.6,
    .fp_start_us = -1.4, .fp_end_us = -0.3,
    .bp_start_us = 8.2, .bp_end_us = 10.2,
    .analog_start_us = 10.5, .analog_end_us = 62.35,
    .active_lines = 288,
    .first_active_k = 22,
    .frame_h = 576,
    .field_rate_num = 50, .field_rate_den = 1,
    .pal = 1,
};

static const std_params_t k_ntsc = {
    .std = STD_NTSC,
    .name = "NTSC",
    .line_us = 1e6 / 15734.264,
    .fsc_hz = 3579545.4545,
    .lines_per_field = 262.5,
    .sync_ratio = 100.0 / 40.0,
    .burst_nom = 20.0 / 100.0,
    .h_active_us = 122.0 / 13.5,
    .burst_start_us = 5.5, .burst_end_us = 7.6,
    .fp_start_us = -1.3, .fp_end_us = -0.3,
    .bp_start_us = 8.0, .bp_end_us = 9.1,
    .analog_start_us = 9.4, .analog_end_us = 62.06,
    .active_lines = 240,
    .first_active_k = 17,
    .frame_h = 480,
    .field_rate_num = 60000, .field_rate_den = 1001,
    .pal = 0,
};

const std_params_t *std_params(video_std_t s)
{
    return s == STD_NTSC ? &k_ntsc : &k_pal;
}

const char *std_name(video_std_t s)
{
    switch (s) {
    case STD_PAL: return "PAL";
    case STD_NTSC: return "NTSC";
    default: return "unknown";
    }
}
