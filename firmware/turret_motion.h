#ifndef MECANUM_JOG_TURRET_MOTION_H
#define MECANUM_JOG_TURRET_MOTION_H

#include <stdint.h>

/* Quintic ramps occupy 15/46 of the time each, with a constant-speed middle.
 * Peak slope is 23/16; at the same peak speed each ramp takes half the original
 * time, doubling peak acceleration before 20 ms duration rounding.
 * speedDps10 is the peak command speed in 0.1 degrees/second. */
static inline uint32_t turretMotionDurationMs(uint32_t deltaMdeg, uint16_t speedDps10)
{
    uint32_t denominator = (uint32_t)speedDps10 * 32U;
    return ((deltaMdeg * 23U + denominator - 1U) / denominator) * 20U;
}

static inline uint32_t turretMotionAngleMdeg(uint32_t start, uint32_t target,
                                      uint32_t elapsedMs, uint32_t durationMs)
{
    float t, u, s;
    uint32_t delta, offset;
    if (elapsedMs >= durationMs) return target;
    t = (float)elapsedMs / (float)durationMs;
    /* Mirror the quintic ramps so angle, speed and acceleration join smoothly. */
    if (t < 15.0f / 46.0f) {
        u = t * (46.0f / 15.0f);
        s = u * u * u * (20.0f + u * (-15.0f + 3.0f * u)) / 32.0f;
    } else if (t <= 31.0f / 46.0f) {
        s = t * (23.0f / 16.0f) - 7.0f / 32.0f;
    } else {
        u = (1.0f - t) * (46.0f / 15.0f);
        s = 1.0f - u * u * u * (20.0f + u * (-15.0f + 3.0f * u)) / 32.0f;
    }
    delta = target > start ? target - start : start - target;
    offset = (uint32_t)((float)delta * s + 0.5f);
    if (offset > delta) offset = delta;
    return target >= start ? start + offset : start - offset;
}

#endif
