#ifndef SHENGSAI_AXIS6_TEST_H
#define SHENGSAI_AXIS6_TEST_H
#include "mechanism_action.h"

static uint8_t axis6TestBuild(const MechanismPose *origin, MechanismAction actions[2])
{
    uint8_t i;
    if (!mechanismPoseValid(origin) ||
        (int32_t)origin->horizontalDmm + 50 > MECHANISM_HORIZONTAL_MAX_DMM) return 0U;
    memset(actions, 0, sizeof(MechanismAction) * 2U);
    for (i = 0U; i < 2U; ++i) {
        actions[i].type = MECHANISM_ACTION_POSE;
        actions[i].pose = *origin;
        actions[i].pose.horizontalRpm = 30U;
        actions[i].pose.horizontalAccel = 50U;
    }
    actions[0].pose.horizontalDmm += 50;
    return 1U;
}
#endif
