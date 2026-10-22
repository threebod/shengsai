#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include "../firmware/turret_motion.h"

static void checkMove(uint32_t start, uint32_t target, uint16_t speed)
{
    uint32_t delta = target > start ? target - start : start - target;
    uint32_t duration = turretMotionDurationMs(delta, speed);
    uint32_t elapsed, previous = start, firstStep = 0U, lastStep = 0U, largestStep = 0U;
    uint32_t maximumStep = ((uint32_t)speed * 2U + 1U);
    assert(duration % 20U == 0U);
    assert(turretMotionAngleMdeg(start, target, 0U, duration) == start);
    for (elapsed = 20U; elapsed <= duration; elapsed += 20U) {
        uint32_t current = turretMotionAngleMdeg(start, target, elapsed, duration);
        uint32_t step = current > previous ? current - previous : previous - current;
        assert(target >= start ? current >= previous && current <= target :
                                 current <= previous && current >= target);
        assert(step <= maximumStep);
        if (elapsed == 20U) firstStep = step;
        lastStep = step;
        if (step > largestStep) largestStep = step;
        previous = current;
    }
    assert(previous == target);
    assert(turretMotionAngleMdeg(start, target, duration + 20U, duration) == target);
    if (duration >= 200U && delta >= 1000U) {
        assert(firstStep < largestStep / 4U);
        assert(lastStep < largestStep / 4U);
    }
}

int main(void)
{
    static const uint32_t positions[] = {0U, 100U, 53000U, 120000U, 260000U, 360000U};
    static const uint16_t speeds[] = {10U, 1200U, 2200U, 2500U};
    unsigned i, j, k;
    assert(turretMotionDurationMs(120000U, 2500U) == 900U);
    for (i = 0U; i < sizeof(positions) / sizeof(positions[0]); ++i)
        for (j = 0U; j < sizeof(positions) / sizeof(positions[0]); ++j)
            for (k = 0U; k < sizeof(speeds) / sizeof(speeds[0]); ++k)
                checkMove(positions[i], positions[j], speeds[k]);
    /* A manual retarget starts at the last issued angle, without a position jump. */
    {
        uint32_t duration = turretMotionDurationMs(207000U, 1200U);
        uint32_t current = turretMotionAngleMdeg(53000U, 260000U, 500U, duration);
        uint32_t nextDuration = turretMotionDurationMs(current - 26000U, 1200U);
        assert(current > 26000U);
        assert(turretMotionAngleMdeg(current, 26000U, 0U, nextDuration) == current);
        checkMove(current, 26000U, 1200U);
    }
    return 0;
}
