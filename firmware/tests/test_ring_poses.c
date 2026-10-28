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
    assert(missionRingPoses[0].horizontalDmm == 130 && missionRingPoses[0].turretDdeg == 3080U);
    assert(missionRingPoses[1].horizontalDmm == -200 && missionRingPoses[1].turretDdeg == 2677U);
    assert(missionRingPoses[2].horizontalDmm == 500 && missionRingPoses[2].turretDdeg == 2335U);
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

static void checkUpperLeftExtension(void)
{
    MechanismAction actions[RAW_PICK_ACTION_COUNT];
    uint8_t slot;
    for (slot = 1U; slot <= 3U; ++slot) {
        assert(rawPickActionsForPosition(actions,slot,2U));
        assert(actions[2].pose.horizontalDmm == 649 && actions[4].pose.horizontalDmm == 649);
        assert(actions[2].pose.turretDdeg == rawPickUpperPoses[0].turretDdeg);
        assert(actions[2].pose.liftDmm == rawPickUpperPoses[0].liftDmm && actions[4].pose.liftDmm == 0U);
        assert(actions[5].pose.horizontalDmm == -500 && actions[5].pose.liftDmm == 0U);
        assert(actions[5].pose.turretDdeg == actions[6].pose.turretDdeg);
        assert(actions[10].pose.liftDmm == 0U && actions[10].pose.turretDdeg == actions[7].pose.turretDdeg);
        assert(actions[10].pose.horizontalDmm == (slot < 3U ? 650 : -500));
        assert(actions[11].pose.horizontalDmm == (slot < 3U ? 650 : 0));
        assert(actions[11].pose.turretDdeg == (slot < 3U ? rawPickObservePose.turretDdeg : rawPickTravelPose.turretDdeg));
        assert(rawPickActionsForPosition(actions,slot,3U));
        assert(actions[2].pose.horizontalDmm == 701 && actions[4].pose.horizontalDmm == 701);
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
    checkUpperLeftExtension();
    assert(missionInitial.gripperOpenDeg == missionCoarseInitial.gripperOpenDeg);
    memcpy(original, missionRingPoses, sizeof(original));
    assert(ringPosesValid(changed));
    for (order = 0U; order < 6U; ++order) {
        for (layer = 1U; layer <= 2U; ++layer)
            for (coarse = 0U; coarse < 2U; ++coarse) {
                uint16_t count;
                memcpy(missionRingPoses, original, sizeof(original));
                count = missionBuildTaskPlace(before, orders[order], layer, coarse);
                assert(count == (coarse ? MISSION_COARSE_PLACE_COUNT - 1U : MISSION_PLACE_COUNT));
                if (coarse) {
                    assert(before[0].type == MECHANISM_ACTION_PLATFORM && before[0].value == 1U);
                    assert(before[1].type == MECHANISM_ACTION_GRIPPER && before[1].value == 1U);
                    assert(before[2].type == MECHANISM_ACTION_POSE && before[2].pose.horizontalDmm == -500);
                    assert(before[2].pose.liftDmm == 0U && before[3].pose.liftDmm == 300U);
                }
                memcpy(missionRingPoses, changed, sizeof(changed));
                missionBuildTaskPlace(after, orders[order], layer, coarse);
                checkUnchanged(before, after, count);
                for (item = 0U; item < 3U; ++item) {
                    uint8_t ring = orders[order][item] - 1U;
                    assert(after[land[item] - coarse].pose.horizontalDmm == changed[ring].horizontalDmm);
                    assert(after[land[item] - coarse].pose.turretDdeg == changed[ring].turretDdeg);
                    assert(after[land[item] - coarse].pose.liftDmm == (layer == 2U ? 570U : 1250U));
                    {
                        unsigned down = land[item] - coarse;
                        const MechanismPose *turn = &after[down - 1U].pose;
                        assert(turn->horizontalDmm == (layer == 2U || ring == 2U ? changed[ring].horizontalDmm :
                                                       after[down - 2U].pose.horizontalDmm));
                        assert(turn->liftDmm == 0U && turn->turretDdeg == after[down].pose.turretDdeg);
                        if (ring == 2U) {
                            /* Raise, then turn/extend together, then lower only. */
                            assert(turn->horizontalDmm == after[down].pose.horizontalDmm);
                            if (item == 2U) {
                                assert(after[down - 2U].pose.horizontalDmm == after[down - 3U].pose.horizontalDmm);
                                assert(after[down - 2U].pose.turretDdeg == after[down - 3U].pose.turretDdeg);
                            }
                        }
                        assert(after[down + 1U].type == MECHANISM_ACTION_GRIPPER && after[down + 1U].value == 1U);
                        if (!coarse || item < 2U) {
                            assert(after[down + 2U].pose.horizontalDmm == changed[ring].horizontalDmm);
                            assert(after[down + 2U].pose.liftDmm == 0U);
                        }
                    }
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
            {
                const MechanismPose *raised = &after[6U + item * 11U].pose;
                const MechanismPose *turn = &after[7U + item * 11U].pose;
                const MechanismPose *tray = &after[8U + item * 11U].pose;
                assert(raised->horizontalDmm == pose->horizontalDmm && raised->liftDmm == 0U);
                assert(raised->turretDdeg == pose->turretDdeg);
                assert(turn->liftDmm == 0U && turn->turretDdeg == 1200U);
                assert(turn->horizontalDmm == (orders[order][item] == 1U ? -450 : pose->horizontalDmm));
                assert(tray->horizontalDmm == -450 && tray->liftDmm == 0U && tray->turretDdeg == 1200U);
            }
        }
    }
    for (layer = 1U; layer <= 2U; ++layer) {
        memcpy(missionRingPoses, original, sizeof(original));
        missionBuildStoragePlace(before, layer);
        memcpy(missionRingPoses, changed, sizeof(changed));
        missionBuildStoragePlace(after, layer);
        checkUnchanged(before, after, MISSION_PLACE_COUNT);
        for (item = 0U; item < 3U; ++item) {
            assert(after[land[item] - 1U].pose.horizontalDmm == (layer == 2U || item == 1U ?
                   after[land[item]].pose.horizontalDmm : after[land[item] - 2U].pose.horizontalDmm));
            assert(after[land[item] - 1U].pose.liftDmm == 0U);
            if (layer == 2U && item == 2U) {
                assert(after[26].pose.turretDdeg == after[25].pose.turretDdeg);
                assert(after[26].pose.horizontalDmm == after[25].pose.horizontalDmm);
            }
            assert(after[land[item] + 2U].pose.horizontalDmm == after[land[item]].pose.horizontalDmm);
            assert(after[land[item] + 2U].pose.liftDmm == 0U);
        }
    }
    return 0;
}
