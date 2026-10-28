#ifndef MECANUM_JOG_MISSION_ACTIONS_H
#define MECANUM_JOG_MISSION_ACTIONS_H

#include "mechanism_action.h"
#include "ring_pose.h"

/* User-specified ring defaults; stored poses take precedence. */
static RingPose missionRingPoses[3] = {{130, 3080}, {-200, 2677}, {500, 2335}};

/* 动作组/放置.json: three tray slots to three rings. Lift coordinates grow
 * downward; the second storage layer keeps its fixed 570 dmm depth. */
static MechanismInitialState missionInitial =
    MECH_INITIAL_STATE(0, 0, 2600, 1000, 200, 2000, 240, 2200, 2700, 1800,
                       1, 1, 200, 155, 26, 146, 264);

/* 动作组/放置_升降600rpm无等待.json is the source; firmware lift moves use 2000 RPM / acc 240. */
static MechanismInitialState missionCoarseInitial =
    MECH_INITIAL_STATE(0, 0, 2600, 1000, 200, 2000, 240, 2200, 2700, 1800,
                       1, 1, 200, 155, 26, 146, 264);

/* Let the grip settle for 200ms after commanded closure before lifting. */
static const MechanismAction missionCoarsePlaceActions[] = {
    MECH_PLATFORM(1, 0),
    MECH_POSE(0, 0, 2600, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(-500, 0, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 300, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_CLOSE(200),
    MECH_POSE(-500, 0, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-200, 0, 2677, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-200, 1250, 2677, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(-200, 0, 2677, 1000, 200, 2000, 240, 2200, 0),
    MECH_PLATFORM(2, 0),
    MECH_POSE(-500, 0, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 300, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_CLOSE(200),
    MECH_POSE(-500, 0, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(500, 0, 2335, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(500, 1250, 2335, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(500, 0, 2335, 1000, 200, 2000, 240, 2200, 0),
    MECH_PLATFORM(3, 0),
    MECH_POSE(-500, 0, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 300, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 300, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_CLOSE(200),
    MECH_POSE(-500, 0, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 0, 3080, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(130, 0, 3080, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(130, 1250, 3080, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0)
};

#define MISSION_COARSE_PLACE_COUNT \
    ((uint16_t)(sizeof(missionCoarsePlaceActions) / sizeof(missionCoarsePlaceActions[0])))

static const MechanismAction missionPlaceActions[] = {
    MECH_PLATFORM(1, 0),
    MECH_POSE(0, 0, 2600, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(-500, 0, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 300, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_CLOSE(200),
    MECH_POSE(-500, 0, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-200, 0, 2677, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-200, 1250, 2677, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(-200, 0, 2677, 1000, 200, 2000, 240, 2200, 0),
    MECH_PLATFORM(2, 0),
    MECH_POSE(-500, 0, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 300, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_CLOSE(200),
    MECH_POSE(-500, 0, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(500, 0, 2335, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(500, 1250, 2335, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(500, 0, 2335, 1000, 200, 2000, 240, 2200, 0),
    MECH_PLATFORM(3, 0),
    MECH_POSE(-500, 0, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 300, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 300, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_CLOSE(200),
    MECH_POSE(-500, 0, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 0, 3080, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(130, 0, 3080, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(130, 1250, 3080, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(130, 0, 3080, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(0, 0, 2600, 1000, 200, 2000, 240, 2200, 0)
};

#define MISSION_PLACE_COUNT \
    ((uint16_t)(sizeof(missionPlaceActions) / sizeof(missionPlaceActions[0])))
#define MISSION_PICK_BACK_COUNT 34U

/* Tray slots preserve task pickup order; rings gives each item's destination. */
static inline uint16_t missionBuildTaskPlace(MechanismAction *actions,
                                 const uint8_t rings[3], uint8_t layer,
                                 uint8_t coarse)
{
    static const uint8_t ringPoses[3][2] = {{7, 8}, {16, 17}, {27, 28}};
    uint8_t item, step, index, ring;
    memcpy(actions, coarse ? missionCoarsePlaceActions : missionPlaceActions,
           coarse ? sizeof(missionCoarsePlaceActions) : sizeof(missionPlaceActions));
    for (item = 0U; item < 3U; ++item) {
        ring = (uint8_t)(rings[item] - 1U);
        actions[item == 0U ? 0U : item == 1U ? 11U : 20U].value = (uint16_t)(item + 1U);
        for (step = 0U; step < 2U; ++step) {
            index = ringPoses[item][step];
            actions[index].pose.turretDdeg = missionRingPoses[ring].turretDdeg;
            actions[index].pose.horizontalDmm = missionRingPoses[ring].horizontalDmm;
        }
        if (layer == 2U)
            actions[ringPoses[item][1]].pose.liftDmm = 570U;
        /* Clear the tray materials before turning back to 120 degrees. */
        if (item < 2U)
            actions[10U + item * 9U].pose = actions[ringPoses[item][0]].pose;
    }
    actions[26].pose.turretDdeg = missionRingPoses[rings[2] - 1U].turretDdeg;
    if (!coarse) actions[30].pose = actions[27].pose;
    /* Ring 3 turns/extends while raised; other first-layer rings keep their approach. */
    if (rings[2] == 3U) actions[26].pose = actions[25].pose;
    if (layer != 2U)
        for (item = 0U; item < 3U; ++item)
            if (rings[item] != 3U)
                actions[ringPoses[item][0]].pose.horizontalDmm =
                    actions[ringPoses[item][0] - 1U].pose.horizontalDmm;
    if (!coarse) return MISSION_PLACE_COUNT;
    /* Go from observation straight to tray pickup, without returning to h=0. */
    memmove(actions + 1, actions + 2, (MISSION_COARSE_PLACE_COUNT - 2U) * sizeof(*actions));
    return MISSION_COARSE_PLACE_COUNT - 1U;
}

/* Raise before the first pickup; lower 15mm for each tray release, then raise. */
static void missionBuildTaskPickBack(MechanismAction *actions,
                                    const uint8_t rings[3],
                                    int16_t trayHorizontalDmm)
{
    uint8_t item, ring;
    uint16_t i = 0U;
    ring = (uint8_t)(rings[2] - 1U);
    actions[i++] = (MechanismAction)MECH_POSE(missionRingPoses[ring].horizontalDmm, 0, missionRingPoses[ring].turretDdeg, 1000, 200, 2000, 240, 2200, 0);
    for (item = 0U; item < 3U; ++item) {
        ring = (uint8_t)(rings[item] - 1U);
        actions[i++] = (MechanismAction)MECH_PLATFORM(item + 1U, 0);
        actions[i++] = (MechanismAction)MECH_GRIPPER_OPEN(0);
        actions[i++] = (MechanismAction)MECH_POSE(missionRingPoses[ring].horizontalDmm, 0, missionRingPoses[ring].turretDdeg, 1000, 200, 2000, 240, 2200, 0);
        actions[i++] = (MechanismAction)MECH_POSE(missionRingPoses[ring].horizontalDmm, 1250, missionRingPoses[ring].turretDdeg, 1000, 200, 2000, 240, 2200, 0);
        actions[i++] = (MechanismAction)MECH_GRIPPER_CLOSE(200);
        actions[i++] = (MechanismAction)MECH_POSE(missionRingPoses[ring].horizontalDmm, 0, missionRingPoses[ring].turretDdeg, 1000, 200, 2000, 240, 2200, 0);
        actions[i++] = (MechanismAction)MECH_POSE(ring == 0U ? trayHorizontalDmm : missionRingPoses[ring].horizontalDmm, 0, 1200, 1000, 200, 2000, 240, 2200, 0);
        actions[i++] = (MechanismAction)MECH_POSE(trayHorizontalDmm, 0, 1200, 1000, 200, 2000, 240, 2200, 0);
        actions[i++] = (MechanismAction)MECH_POSE(trayHorizontalDmm, 150, 1200, 1000, 200, 2000, 240, 2200, 0);
        actions[i++] = (MechanismAction)MECH_GRIPPER_OPEN(0);
        actions[i++] = (MechanismAction)MECH_POSE(trayHorizontalDmm, 0, 1200, 1000, 200, 2000, 240, 2200, 0);
    }
}

static void missionBuildStoragePlace(MechanismAction *actions, uint8_t layer)
{
    static const uint8_t poses[3][3] = {{7, 8, 10}, {16, 17, 19}, {27, 28, 30}};
    uint8_t item, step, ring;
    memcpy(actions, missionPlaceActions, sizeof(missionPlaceActions));
    if (layer == 2U) {
        /* Tray slots 1/2/3 correspond to rings 2/3/1. Stack rings 1/2/3. */
        actions[0].value = 3U;
        actions[11].value = 1U;
        actions[20].value = 2U;
        actions[4].pose = missionPlaceActions[22].pose;
        actions[13].pose = missionPlaceActions[4].pose;
        actions[7].pose = missionPlaceActions[27].pose;
        actions[8].pose = missionPlaceActions[28].pose;
        actions[16].pose = missionPlaceActions[7].pose;
        actions[17].pose = missionPlaceActions[8].pose;
        actions[22].pose = missionPlaceActions[13].pose;
        actions[23].pose = missionPlaceActions[13].pose;
        actions[26].pose.turretDdeg = missionPlaceActions[16].pose.turretDdeg;
        actions[27].pose = missionPlaceActions[16].pose;
        actions[28].pose = missionPlaceActions[17].pose;
        actions[8].pose.liftDmm = 570U;
        actions[17].pose.liftDmm = 570U;
        actions[28].pose.liftDmm = 570U;
        actions[10].pose = actions[7].pose;
        actions[19].pose = actions[16].pose;
        actions[30].pose = actions[27].pose;
    }
    for (item = 0U; item < 3U; ++item) {
        ring = layer == 2U ? item : (uint8_t)((item + 1U) % 3U);
        for (step = 0U; step < 3U; ++step) {
            actions[poses[item][step]].pose.horizontalDmm = missionRingPoses[ring].horizontalDmm;
            actions[poses[item][step]].pose.turretDdeg = missionRingPoses[ring].turretDdeg;
        }
    }
    actions[26].pose.turretDdeg = missionRingPoses[layer == 2U ? 2U : 0U].turretDdeg;
    if (layer == 2U) actions[26].pose = actions[25].pose;
    if (layer != 2U)
        for (item = 0U; item < 3U; ++item)
            if (item != 1U) /* First-layer order is rings 2, 3, 1. */
                actions[poses[item][0]].pose.horizontalDmm =
                    actions[poses[item][0] - 1U].pose.horizontalDmm;
}

/* Ring 1 returns to the tray with simultaneous turn/retract after raising. */
static void missionBuildPickBack(MechanismAction *actions)
{
    static const uint8_t rings[3] = {2U, 3U, 1U};
    missionBuildTaskPickBack(actions, rings, -500);
}

#endif
