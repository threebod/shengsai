#include <assert.h>
#include "../mission_actions.h"

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
