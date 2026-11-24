#include "cvbs_synth.h"

#include <math.h>

#include "common.h"

typedef enum { HL_NONE, HL_HSYNC, HL_EQ, HL_BROAD } hl_t;

void cvbs_synth_init(cvbs_synth_t *s, video_std_t std)
{
    s->sp = std_params(std);
    s->fsc_w = 2.0 * M_PI * s->sp->fsc_hz;
}

/* Half-line index hl counts from the start of line 1 (two per line). */
static hl_t half_line_type(const std_params_t *sp, int hl)
{
    if (sp->pal) {
        if ((hl >= 0 && hl <= 4) || (hl >= 625 && hl <= 629))
            return HL_BROAD;
        if ((hl >= 5 && hl <= 9) || (hl >= 620 && hl <= 624) || (hl >= 630 && hl <= 634) ||
            (hl >= 1245 && hl <= 1249))
            return HL_EQ;
    } else {
        if ((hl >= 6 && hl <= 11) || (hl >= 531 && hl <= 536))
            return HL_BROAD;
        if ((hl >= 0 && hl <= 5) || (hl >= 12 && hl <= 17) || (hl >= 525 && hl <= 530) ||
            (hl >= 537 && hl <= 542))
            return HL_EQ;
    }
    return (hl % 2 == 0) ? HL_HSYNC : HL_NONE;
}

void cvbs_synth_picture(double x, double y, long frame, double *Y, double *U, double *V)
{
    static const double bars[8][3] = {
        {.75, .75, .75}, {.75, .75, 0}, {0, .75, .75}, {0, .75, 0},
        {.75, 0, .75},   {.75, 0, 0},   {0, 0, .75},   {0, 0, 0},
    };
    *Y = *U = *V = 0;
    if (x < 0 || x >= 1 || y < 0 || y >= 1)
        return;
    double R, G, B;
    if (y < 0.6) {
        int i = (int)(x * 8);
        R = bars[i][0];
        G = bars[i][1];
        B = bars[i][2];
    } else {
        double g = 0.1 + 0.8 * x;
        R = G = B = g;
        double bx = fmod(frame * 0.0125, 1.0);
        if (y > 0.7 && y < 0.9 && x >= bx && x < bx + 0.08)
            R = G = B = 0.95;
    }
    double Yv = 0.299 * R + 0.587 * G + 0.114 * B;
    *Y = Yv;
    *U = 0.492 * (B - Yv);
    *V = 0.877 * (R - Yv);
}

double cvbs_synth_eval(const cvbs_synth_t *s, double t)
{
    const std_params_t *sp = s->sp;
    const double H = sp->line_us * 1e-6;
    const double Hh = 0.5 * H;
    const int total_hl = (int)lrint(sp->lines_per_field * 4.0);
    const double Tf = total_hl * Hh;

    double tf = fmod(t, Tf);
    if (tf < 0)
        tf += Tf;
    int hl = (int)(tf / Hh);
    if (hl >= total_hl)
        hl = total_hl - 1;
    double pos_h = (tf - hl * Hh) * 1e6;

    /* Sync pulse depth with 0.2 us edges. */
    hl_t ty = half_line_type(sp, hl);
    double w = ty == HL_HSYNC ? 4.7 : ty == HL_EQ ? 2.35 : ty == HL_BROAD ? (Hh * 1e6 - 4.7) : 0.0;
    const double r = 0.2;
    double depth = 0.0;
    if (w > 0) {
        if (pos_h < r)
            depth = pos_h / r;
        else if (pos_h < w)
            depth = 1.0;
        else if (pos_h < w + r)
            depth = 1.0 - (pos_h - w) / r;
    }

    int line = hl / 2 + 1;
    double pos = (tf - (line - 1) * H) * 1e6;
    int field, k;
    if (sp->pal) {
        if (line <= 313) { field = 1; k = line - 1; }
        else { field = 2; k = line - 314; }
    } else {
        if (line >= 4 && line <= 266) { field = 1; k = line - 4; }
        else if (line >= 267) { field = 2; k = line - 267; }
        else { field = 2; k = line + 525 - 267; }
    }
    bool normal = half_line_type(sp, (line - 1) * 2) == HL_HSYNC;
    double v = 0.0;
    double wt = s->fsc_w * t;
    int sw = (line & 1) ? 1 : -1;

    if (normal && k >= 6) {
        double bs = sp->pal ? 5.6 : 5.3, bw = sp->pal ? 2.25 : 2.51;
        if (pos >= bs && pos < bs + bw) {
            double A = sp->burst_nom;
            if (sp->pal)
                v += A * M_SQRT1_2 * (-sin(wt) + sw * cos(wt));
            else
                v += -A * sin(wt);
        }
    }
    if (normal && k >= sp->first_active_k && k < sp->first_active_k + sp->active_lines &&
        pos >= sp->analog_start_us && pos < sp->analog_end_us) {
        int row = 2 * (k - sp->first_active_k) + (field == 1 ? 0 : 1);
        double y = (row + 0.5) / (2.0 * sp->active_lines);
        double x = (pos - sp->h_active_us) / (OUT_W / 13.5);
        long frame = (long)(t / Tf);
        double Y, U, V;
        cvbs_synth_picture(x, y, frame, &Y, &U, &V);
        v += Y + U * sin(wt) + (sp->pal ? sw : 1) * V * cos(wt);
    }
    double sync_level = -1.0 / sp->sync_ratio;
    return depth * sync_level + (1.0 - depth) * v;
}
