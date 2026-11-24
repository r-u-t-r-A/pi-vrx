/* c5div - common helpers */
#pragma once

#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

extern int g_verbose;
extern atomic_bool g_running;

#define LOGI(...) do { fprintf(stderr, "[c5div] " __VA_ARGS__); fputc('\n', stderr); } while (0)
#define LOGW(...) do { fprintf(stderr, "[c5div] warning: " __VA_ARGS__); fputc('\n', stderr); } while (0)
#define LOGE(...) do { fprintf(stderr, "[c5div] error: " __VA_ARGS__); fputc('\n', stderr); } while (0)
#define LOGT(...) do { if (g_verbose >= 2) { fprintf(stderr, "[c5div] " __VA_ARGS__); fputc('\n', stderr); } } while (0)
#define LOGD(...) do { if (g_verbose) { fprintf(stderr, "[c5div] " __VA_ARGS__); fputc('\n', stderr); } } while (0)

static inline double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static inline uint8_t u8f(float v)
{
    int i = (int)lrintf(v);
    return (uint8_t)(i < 0 ? 0 : (i > 255 ? 255 : i));
}

/* Output raster (BT.601 sampling, 13.5 MHz). */
#define OUT_W 720
#define OUT_CW (OUT_W / 2)
#define PX_RATE 13.5e6
#define MAX_FIELD_LINES 288
