#ifndef MECANUM_JOG_RAW_PICK_ACTION_H
#define MECANUM_JOG_RAW_PICK_ACTION_H

#include "mechanism_action.h"
#include "placement_heights.h"
#include "ring_pose.h"

/* Turret remount: recorded angles - 12 degrees (observation 272 -> 260). */

/* Single-item bench sequence from 动作组/夹取.json. The operator places the
 * selected color at this sequence's fixed pickup position. */
static MechanismInitialState rawPickInitial =
    MECH_INITIAL_STATE(0, 0, 2600, 1000, 228, 2000, 248, 2200, 3600, 2200,
                       1, 1, 200, 155, 26, 146, 264);

static const MechanismPose rawPickObservePose =
    { 650, 0, 2600, 1000, 228, 2000, 248, 2200 };

static MechanismPose rawPickLowerPose =
    { -800, 530, 2710, 1000, 228, 2000, 248, 2200 };

static MechanismPose rawPickUpperPoses[2] = {
    { 750, 530, 2903, 1000, 228, 2000, 248, 2200 },
    { 800, 530, 2534, 1000, 228, 2000, 248, 2200 }
};

static uint8_t pickupPosesSaved;

static void applyPickupPoses(const RingPose poses[3])
{
    uint8_t i;
    for (i = 0U; i < 3U; ++i) {
        MechanismPose *pose = i == 0U ? &rawPickLowerPose : &rawPickUpperPoses[i - 1U];
        pose->horizontalDmm = poses[i].horizontalDmm;
        pose->turretDdeg = poses[i].turretDdeg;
    }
}

static const MechanismPose rawPickTravelPose =
    { 0, 0, 530, 1000, 228, 2000, 248, 2200 };

/* Let the grip settle for 200ms after commanded closure before lifting. */
static const MechanismAction rawPickActions[] = {
    MECH_PLATFORM(1, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(-800, 530, 2710, 1000, 228, 2000, 248, 2200, 0),
    MECH_GRIPPER_CLOSE(200),
    MECH_POSE(-800, 0, 2710, 1000, 228, 2000, 248, 2200, 0),
    MECH_POSE(-200, 0, 1200, 1000, 228, 2000, 248, 2200, 0),
    MECH_POSE(-500, 0, 1200, 1000, 228, 2000, 248, 2200, 0),
    MECH_POSE(-500, 170, 1200, 1000, 228, 2000, 248, 2200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_PLATFORM(2, 0),
    MECH_POSE(-500, 0, 1200, 1000, 228, 2000, 248, 2200, 0),
    MECH_POSE(0, 0, 2600, 1000, 228, 2000, 248, 2200, 0)
};

#define RAW_PICK_ACTION_COUNT \
    ((uint16_t)(sizeof(rawPickActions) / sizeof(rawPickActions[0])))

static void rawPickActionsForSlot(MechanismAction *actions, uint8_t slot)
{
    memcpy(actions, rawPickActions, sizeof(rawPickActions));
    actions[2].pose = rawPickLowerPose;
    actions[4].pose = rawPickLowerPose;
    actions[4].pose.liftDmm = 0U;
    actions[7].pose.liftDmm = placementHeights.rawTrayDmm;
    actions[0].value = slot;
    if (slot < 3U) {
        actions[10].pose.horizontalDmm = rawPickObservePose.horizontalDmm;
        actions[11].pose = rawPickObservePose;
    }
    actions[9].value = slot < 3U ? (uint16_t)(slot + 1U) : 3U;
    if (slot == 3U) actions[11].pose = rawPickTravelPose;
}

/* Position order matches vision: bottom, left upper, right upper. */
static uint8_t rawPickActionsForPosition(MechanismAction *actions, uint8_t slot,
                                         uint8_t position)
{
    const MechanismPose *pickup;
    if (slot < 1U || slot > 3U || position < 1U || position > 3U) return 0U;
    pickup = position == 1U ? &rawPickLowerPose : &rawPickUpperPoses[position - 2U];
    rawPickActionsForSlot(actions, slot);
    actions[2].pose = *pickup;
    actions[4].pose = *pickup;
    actions[4].pose.liftDmm = 0U;
    /* Left upper: retract to the tray while turning, without the -20mm stop. */
    if (position == 2U) actions[5].pose = actions[6].pose;
    return 1U;
}

#endif
