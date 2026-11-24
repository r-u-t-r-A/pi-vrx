/* Eight int16 lanes: NEON on the Pi, GCC vector extensions elsewhere, with the
 * same results (PC tests run the same arithmetic). */
#pragma once

#include <stdint.h>
#include <string.h>

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

typedef int16_t v8s __attribute__((vector_size(16)));

static inline v8s v8_load(const int16_t *p)
{
    v8s v;
    memcpy(&v, p, 16);
    return v;
}

static inline void v8_store(int16_t *p, v8s v) { memcpy(p, &v, 16); }

static inline v8s v8_dup(int x)
{
    const int16_t s = (int16_t)x;
    return (v8s){s, s, s, s, s, s, s, s};
}

/* Widen eight bytes. */
static inline v8s v8_load_u8(const uint8_t *p)
{
#ifdef __ARM_NEON
    return (v8s)vreinterpretq_s16_u16(vmovl_u8(vld1_u8(p)));
#else
    return (v8s){p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]};
#endif
}

/* Rounding Q15 product. */
static inline v8s v8_mulr(v8s a, v8s b)
{
#ifdef __ARM_NEON
    return (v8s)vqrdmulhq_s16((int16x8_t)a, (int16x8_t)b);
#else
    v8s r;
    for (int i = 0; i < 8; i++) {
        const int32_t x = ((int32_t)a[i] * b[i] + (1 << 14)) >> 15;
        r[i] = (int16_t)(x > 32767 ? 32767 : x);
    }
    return r;
#endif
}

/* Saturate to bytes. */
static inline void v8_store_u8(uint8_t *p, v8s v)
{
#ifdef __ARM_NEON
    vst1_u8(p, vqmovun_s16((int16x8_t)v));
#else
    for (int i = 0; i < 8; i++)
        p[i] = (uint8_t)(v[i] < 0 ? 0 : v[i] > 255 ? 255 : v[i]);
#endif
}

static inline v8s v8_abs(v8s v)
{
#ifdef __ARM_NEON
    return (v8s)vqabsq_s16((int16x8_t)v);
#else
    const v8s m = v >> 15;
    return (v ^ m) - m;
#endif
}

static inline v8s v8_min(v8s a, v8s b)
{
#ifdef __ARM_NEON
    return (v8s)vminq_s16((int16x8_t)a, (int16x8_t)b);
#else
    const v8s m = a < b;
    return (a & m) | (b & ~m);
#endif
}

/* Sum of the lanes. */
static inline int32_t v8_sum(v8s v)
{
#ifdef __ARM_NEON
    return vaddlvq_s16((int16x8_t)v);
#else
    int32_t s = 0;
    for (int i = 0; i < 8; i++)
        s += v[i];
    return s;
#endif
}
