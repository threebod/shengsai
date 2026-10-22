#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include "../mission_actions.h"
#include "../raw_pick_action.h"

static void checkTurretRemount(void)
{
    const int observation = rawPickObservePose.turretDdeg;
    uint16_t i;
    assert(observation == 2600);
    assert(rawPickInitial.pose.turretDdeg == observation);
    assert(missionInitial.pose.turretDdeg == observation && missionCoarseInitial.pose.turretDdeg == observation);
    /* Preserve the recorded geometry relative to the observation direction. */
    assert((int)rawPickLowerPose.turretDdeg - observation == -40);
    assert((int)rawPickUpperPoses[0].turretDdeg - observation == 173);
    assert((int)rawPickUpperPoses[1].turretDdeg - observation == -216);
    assert((int)rawPickTravelPose.turretDdeg - observation == -2070);
    assert((int)missionRingPoses[0].turretDdeg - observation == 358);
    assert((int)missionRingPoses[1].turretDdeg - observation == -83);
    assert((int)missionRingPoses[2].turretDdeg - observation == -420);
    assert((int)rawPickActions[5].pose.turretDdeg - observation == -1400);
    assert(rawPickActions[2].pose.turretDdeg == rawPickLowerPose.turretDdeg);
    assert(rawPickActions[11].pose.turretDdeg == observation);
    for (i = 0U; i < MISSION_PLACE_COUNT; ++i)
        if (missionPlaceActions[i].type == MECHANISM_ACTION_POSE) {
            assert(mechanismPoseValid(&missionPlaceActions[i].pose));
            if (i < MISSION_COARSE_PLACE_COUNT)
                assert(missionCoarsePlaceActions[i].pose.turretDdeg == missionPlaceActions[i].pose.turretDdeg);
        }
    /* Other servos and mechanism coordinates must not inherit the turret offset. */
    assert(rawPickInitial.platformDeg[0] == 26U && rawPickInitial.platformDeg[1] == 146U && rawPickInitial.platformDeg[2] == 264U);
    assert(rawPickInitial.gripperOpenDeg == 200U && rawPickInitial.gripperCloseDeg == 155U);
    assert(rawPickObservePose.horizontalDmm == 650 && rawPickLowerPose.liftDmm == 530U);
}

static void checkUnchanged(const MechanismAction *before, const MechanismAction *after, uint16_t count)
{
    uint16_t i;
    for (i = 0U; i < count; ++i) {
        MechanismAction expected = before[i];
        /* Only horizontal/turn may change; ignore unspecified struct padding. */
        assert(expected.type == after[i].type && expected.channel == after[i].channel &&
               expected.value == after[i].value && expected.waitMs == after[i].waitMs);
        assert(expected.pose.liftDmm == after[i].pose.liftDmm &&
               expected.pose.horizontalRpm == after[i].pose.horizontalRpm &&
               expected.pose.horizontalAccel == after[i].pose.horizontalAccel &&
               expected.pose.liftRpm == after[i].pose.liftRpm &&
               expected.pose.liftAccel == after[i].pose.liftAccel &&
               expected.pose.turretDps10 == after[i].pose.turretDps10);
    }
}

int main(void)
{
    static const uint8_t orders[6][3] = {{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
    static const uint8_t land[3] = {8,17,28};
    RingPose original[3], changed[3] = {{-80,3000},{-360,2600},{410,2300}};
    MechanismAction before[MISSION_PICK_BACK_COUNT], after[MISSION_PICK_BACK_COUNT];
    uint8_t order, layer, coarse, item;
    checkTurretRemount();
    assert(missionInitial.gripperOpenDeg == missionCoarseInitial.gripperOpenDeg);
    memcpy(original, missionRingPoses, sizeof(original));
    assert(ringPosesValid(changed));
    for (order = 0U; order < 6U; ++order) {
        for (layer = 1U; layer <= 2U; ++layer)
            for (coarse = 0U; coarse < 2U; ++coarse) {
                uint16_t count = coarse ? MISSION_COARSE_PLACE_COUNT : MISSION_PLACE_COUNT;
                memcpy(missionRingPoses, original, sizeof(original));
                missionBuildTaskPlace(before, orders[order], layer, coarse);
                memcpy(missionRingPoses, changed, sizeof(changed));
                missionBuildTaskPlace(after, orders[order], layer, coarse);
                checkUnchanged(before, after, count);
                for (item = 0U; item < 3U; ++item) {
                    uint8_t ring = orders[order][item] - 1U;
                    assert(after[land[item]].pose.horizontalDmm == changed[ring].horizontalDmm);
                    assert(after[land[item]].pose.turretDdeg == changed[ring].turretDdeg);
                    assert(after[land[item]].pose.liftDmm == (layer == 2U ? 570U : 1250U));
                }
            }
        memcpy(missionRingPoses, original, sizeof(original));
        missionBuildTaskPickBack(before, orders[order], -450);
        memcpy(missionRingPoses, changed, sizeof(changed));
        missionBuildTaskPickBack(after, orders[order], -450);
        checkUnchanged(before, after, MISSION_PICK_BACK_COUNT);
        for (item = 0U; item < 3U; ++item) {
            const MechanismPose *pose = &after[4U + item * 11U].pose;
            assert(pose->horizontalDmm == changed[orders[order][item] - 1U].horizontalDmm);
            assert(pose->turretDdeg == changed[orders[order][item] - 1U].turretDdeg);
            assert(pose->liftDmm == 1250U);
        }
    }
    for (layer = 1U; layer <= 2U; ++layer) {
        memcpy(missionRingPoses, original, sizeof(original));
        missionBuildStoragePlace(before, layer);
        memcpy(missionRingPoses, changed, sizeof(changed));
        missionBuildStoragePlace(after, layer);
        checkUnchanged(before, after, MISSION_PLACE_COUNT);
    }
    return 0;
}
