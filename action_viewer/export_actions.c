#include <stdio.h>
#include "../firmware/mission_actions.h"
#include "../firmware/raw_pick_action.h"

static unsigned groupCount;

static void printPose(const MechanismPose *pose)
{
    printf("{\"h\":%.1f,\"lift\":%.1f,\"turret\":%.1f,"
           "\"horizontalRpm\":%u,\"horizontalAccel\":%u,\"liftRpm\":%u,"
           "\"liftAccel\":%u,\"turretDps\":%.1f}",
           pose->horizontalDmm / 10.0, pose->liftDmm / 10.0, pose->turretDdeg / 10.0,
           pose->horizontalRpm, pose->horizontalAccel, pose->liftRpm,
           pose->liftAccel, pose->turretDps10 / 10.0);
}

static void printAction(const MechanismAction *action, unsigned index, unsigned approach,
                        MechanismPose *current, const MechanismInitialState *initial,
                        unsigned *holding, unsigned *count)
{
    uint32_t horizontal = 0U, lift = 0U, turret = 0U;
    static const char *types[] = {"pose", "gripper", "platform", "servo", "wait"};
    if ((*count)++) putchar(',');
    printf("{\"sourceStep\":%u,\"approach\":%s,\"type\":\"%s\","
           "\"channel\":%u,\"value\":%u,\"waitMs\":%u,\"pose\":",
           index, approach ? "true" : "false", types[action->type],
           action->channel, action->value, action->waitMs);
    printPose(&action->pose);
    if (action->type == MECHANISM_ACTION_POSE) {
        int16_t dh = action->pose.horizontalDmm - current->horizontalDmm;
        int16_t dl = action->pose.liftDmm - current->liftDmm;
        uint32_t dt = mechanismAbsoluteDelta((int16_t)(action->pose.turretDdeg - current->turretDdeg));
        horizontal = mechanismMotorDurationMs(mechanismHorizontalPulses(dh), action->pose.horizontalRpm,
                                              action->pose.horizontalAccel) + (dh ? MECHANISM_HORIZONTAL_MARGIN_MS : 0U);
        lift = mechanismMotorDurationMs(mechanismLiftPulses(dl), action->pose.liftRpm, action->pose.liftAccel);
        turret = turretMotionDurationMs(dt * 100U, *holding ? action->pose.turretDps10 : 2500U);
        *current = action->pose;
    } else if (action->type == MECHANISM_ACTION_GRIPPER) {
        *holding = !action->value;
    }
    printf(",\"motionMs\":{\"h\":%lu,\"lift\":%lu,\"turret\":%lu}}",
           (unsigned long)horizontal, (unsigned long)lift, (unsigned long)turret);
    (void)initial;
}

static void printGroup(const char *id, const char *family, unsigned layer, unsigned position,
                       unsigned slot, const uint8_t order[3], const MechanismInitialState *initial,
                       const MechanismAction *actions, unsigned length)
{
    MechanismPose current = initial->pose;
    unsigned i, emitted = 0U, holding = !initial->gripperOpen;
    if (groupCount++) putchar(',');
    printf("{\"id\":\"%s\",\"family\":\"%s\",\"layer\":%u,\"position\":%u,"
           "\"slot\":%u,\"order\":[%u,%u,%u],\"initial\":{\"pose\":",
           id, family, layer, position, slot, order[0], order[1], order[2]);
    printPose(&initial->pose);
    printf(",\"gripper\":%u,\"platform\":%u,\"slot\":%u},"
           "\"servo\":{\"open\":%u,\"close\":%u,\"gripperDps\":%.1f,"
           "\"platformDps\":%.1f,\"platformAngles\":[%u,%u,%u]},\"actions\":[",
           initial->gripperOpen ? initial->gripperOpenDeg : initial->gripperCloseDeg,
           initial->platformDeg[initial->platform - 1U], initial->platform,
           initial->gripperOpenDeg, initial->gripperCloseDeg,
           initial->gripperDps10 / 10.0, initial->platformDps10 / 10.0,
           initial->platformDeg[0], initial->platformDeg[1], initial->platformDeg[2]);
    for (i = 0U; i < length; ++i) {
        const MechanismAction *action = &actions[i];
        if (initial == &rawPickInitial && action->type == MECHANISM_ACTION_POSE &&
            action->pose.horizontalDmm == rawPickLowerPose.horizontalDmm &&
            action->pose.liftDmm == rawPickLowerPose.liftDmm &&
            action->pose.turretDdeg == rawPickLowerPose.turretDdeg &&
            current.horizontalDmm != action->pose.horizontalDmm && current.liftDmm < action->pose.liftDmm) {
            MechanismAction approach = *action;
            approach.pose.liftDmm = current.liftDmm;
            approach.waitMs = 0U;
            printAction(&approach, i, 1U, &current, initial, &holding, &emitted);
        }
        printAction(action, i, 0U, &current, initial, &holding, &emitted);
    }
    fputs("]}", stdout);
}

int main(void)
{
    static const uint8_t orders[6][3] = {{2,3,1},{1,2,3},{1,3,2},{2,1,3},{3,1,2},{3,2,1}};
    MechanismAction actions[MISSION_PICK_BACK_COUNT];
    unsigned order, position, slot;
    uint16_t firstLayer, secondLayer, rawTray, returnTray, count;
    char id[40];
    missionBuildTaskPlace(actions, orders[0], 1U, 0U);
    firstLayer = actions[8].pose.liftDmm;
    missionBuildTaskPlace(actions, orders[0], 2U, 0U);
    secondLayer = actions[8].pose.liftDmm;
    rawPickActionsForSlot(actions, 1U);
    rawTray = actions[7].pose.liftDmm;
    missionBuildTaskPickBack(actions, orders[0], -450);
    returnTray = actions[9].pose.liftDmm;
    printf("{\"version\":1,\"heights\":[%.1f,%.1f,%.1f,%.1f],\"rings\":[",
           firstLayer / 10.0, secondLayer / 10.0, rawTray / 10.0, returnTray / 10.0);
    for (order = 0U; order < 3U; ++order) {
        if (order) putchar(',');
        printf("{\"h\":%.1f,\"turret\":%.1f}", missionRingPoses[order].horizontalDmm / 10.0,
               missionRingPoses[order].turretDdeg / 10.0);
    }
    fputs("],\"groups\":[", stdout);
    for (position = 1U; position <= 3U; ++position)
        for (slot = 1U; slot <= 3U; ++slot) {
            rawPickActionsForPosition(actions, slot, position);
            snprintf(id, sizeof(id), "raw-%u-%u", position, slot);
            printGroup(id, "raw", 0U, position, slot, orders[0], &rawPickInitial, actions, RAW_PICK_ACTION_COUNT);
        }
    for (order = 0U; order < 6U; ++order) {
        const uint8_t *rings = orders[order];
        snprintf(id, sizeof(id), "coarse-%u%u%u", rings[0], rings[1], rings[2]);
        count = missionBuildTaskPlace(actions, rings, 1U, 1U);
        printGroup(id, "coarse", 1U, 0U, 0U, rings, &missionCoarseInitial, actions, count);
        snprintf(id, sizeof(id), "storage1-%u%u%u", rings[0], rings[1], rings[2]);
        missionBuildTaskPlace(actions, rings, 1U, 0U);
        printGroup(id, "storage1", 1U, 0U, 0U, rings, &missionInitial, actions, MISSION_PLACE_COUNT);
        snprintf(id, sizeof(id), "storage2-%u%u%u", rings[0], rings[1], rings[2]);
        missionBuildTaskPlace(actions, rings, 2U, 0U);
        printGroup(id, "storage2", 2U, 0U, 0U, rings, &missionInitial, actions, MISSION_PLACE_COUNT);
        snprintf(id, sizeof(id), "return-%u%u%u", rings[0], rings[1], rings[2]);
        missionBuildTaskPickBack(actions, rings, -450);
        printGroup(id, "return", 1U, 0U, 0U, rings, &missionCoarseInitial, actions, MISSION_PICK_BACK_COUNT);
        snprintf(id, sizeof(id), "return2-%u%u%u", rings[0], rings[1], rings[2]);
        missionBuildTaskPickBack(actions, rings, -470);
        printGroup(id, "return2", 1U, 0U, 0U, rings, &missionCoarseInitial, actions, MISSION_PICK_BACK_COUNT);
    }
    fputs("]}\n", stdout);
    return ferror(stdout) ? 1 : 0;
}
