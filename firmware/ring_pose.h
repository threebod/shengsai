#ifndef MECANUM_JOG_RING_POSE_H
#define MECANUM_JOG_RING_POSE_H

#include "mechanism_action.h"

typedef struct {
    int16_t horizontalDmm;
    uint16_t turretDdeg;
} RingPose;

static inline uint8_t ringPosesValid(const RingPose poses[3])
{
    uint8_t i;
    for (i = 0U; i < 3U; ++i)
        if (poses[i].horizontalDmm < MECHANISM_HORIZONTAL_MIN_DMM ||
            poses[i].horizontalDmm > MECHANISM_HORIZONTAL_MAX_DMM ||
            poses[i].turretDdeg > MECHANISM_TURRET_MAX_DDEG) return 0U;
    return 1U;
}

#endif
