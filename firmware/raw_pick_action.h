#ifndef MECANUM_JOG_RAW_PICK_ACTION_H
#define MECANUM_JOG_RAW_PICK_ACTION_H

#include "mechanism_action.h"

/* Turret remount: recorded angles - 12 degrees (observation 272 -> 260). */

/* Single-item bench sequence from 动作组/夹取.json. The operator places the
 * selected color at this sequence's fixed pickup position. */
static MechanismInitialState rawPickInitial =
    MECH_INITIAL_STATE(0, 0, 2600, 1000, 200, 2000, 240, 2200, 3600, 1800,
                       1, 1, 200, 155, 26, 146, 264);

static const MechanismPose rawPickObservePose =
    { 650, 0, 2600, 1000, 200, 2000, 240, 2200 };

static const MechanismPose rawPickLowerPose =
    { -900, 530, 2560, 1000, 200, 2000, 240, 2200 };

static const MechanismPose rawPickUpperPoses[2] = {
    { 649, 530, 2773, 1000, 200, 2000, 240, 2200 },
    { 701, 530, 2384, 1000, 200, 2000, 240, 2200 }
};

static const MechanismPose rawPickTravelPose =
    { 0, 0, 530, 1000, 200, 2000, 240, 2200 };

/* Let the grip settle for 200ms after commanded closure before lifting. */
static const MechanismAction rawPickActions[] = {
    MECH_PLATFORM(1, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_POSE(-900, 530, 2560, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_CLOSE(200),
    MECH_POSE(-900, 0, 2560, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-200, 0, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 0, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(-500, 120, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_GRIPPER_OPEN(0),
    MECH_PLATFORM(2, 0),
    MECH_POSE(-500, 0, 1200, 1000, 200, 2000, 240, 2200, 0),
    MECH_POSE(0, 0, 2600, 1000, 200, 2000, 240, 2200, 0)
};

#define RAW_PICK_ACTION_COUNT \
    ((uint16_t)(sizeof(rawPickActions) / sizeof(rawPickActions[0])))

static void rawPickActionsForSlot(MechanismAction *actions, uint8_t slot)
{
    memcpy(actions, rawPickActions, sizeof(rawPickActions));
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
