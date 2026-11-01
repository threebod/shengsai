#ifndef SHENGSAI_MISSION_OBSERVATION_H
#define SHENGSAI_MISSION_OBSERVATION_H

static const MechanismPose *missionObservationPose(uint8_t taskIndex)
{
    if (taskIndex == 3U || taskIndex == 10U) return &rawPickObservePose;
    if (taskIndex == 6U || taskIndex == 8U || taskIndex == 13U || taskIndex == 15U)
        return &missionRingObservePose;
    return (const MechanismPose *)0;
}

static void cancelMissionObservation(void)
{
    if (missionObservationTask) stopServoMotion();
    missionObservationTask = 0U;
}

static uint32_t missionObservationLeadMs(const MechanismPose *target)
{
    MechanismPose axes = *target;
    uint32_t duration, turretMs;
    uint32_t delta = mechanismAbsoluteDelta((int16_t)(target->turretDdeg -
        mechanismState.current.turretDdeg)) * 100U;
    axes.turretDdeg = mechanismState.current.turretDdeg;
    duration = mechanismMoveDurationMs(&mechanismState.current, &axes);
    turretMs = turretMotionDurationMs(delta, gripperMayHoldMaterial ?
        target->turretDps10 : SERVO_EMPTY_TURRET_SPEED_DPS10);
    if (turretMs > duration) duration = turretMs;
    /* Feedback scheduling allowance only; arrival still requires real feedback. */
    return duration + 100U;
}

static float missionObservationLeadDistance(uint32_t leadMs, uint16_t rpm, float scale)
{
    float seconds = leadMs / 1000.0f;
    float speed = rpm * (ROUTE_MM_PER_REV / 60.0f) * scale;
    float brake = ROUTE_BRAKE_RPM_S * (ROUTE_MM_PER_REV / 60.0f) * scale;
    float brakeSeconds = speed / brake;
    /* Fastest remaining cruise/braking profile; acceleration only adds time. */
    if (seconds <= brakeSeconds) return 0.5f * brake * seconds * seconds;
    return speed * (seconds - 0.5f * brakeSeconds);
}

static void serviceMissionObservation(const RouteLeg *leg, float remaining,
                                      uint16_t cruiseRpm, uint8_t lateral)
{
    const MechanismPose *target;
    uint8_t task, coarseCentre;
    float scale;
    if (!missionActive || !presetRouteActive || !routeActive || routeWaiting ||
        routeRotating || missionArcActive || motionMode || !leg->event ||
        presetRoute.index >= presetRoute.count || presetRoute.legs[presetRoute.index].arc)
        return;
    task = presetRoute.legs[presetRoute.index].taskIndex;
    coarseCentre = (uint8_t)((task == 6U || task == 13U) &&
                             leg->upMm < 0 && !leg->rightMm && leg->heading == -1);
    /* RAW return may unfold on its last straight before the station heading turn. */
    target = missionObservationPose(task);
    if (!target || missionObservationTask ||
        (leg->departureHeading != 4 && leg->departureHeading != routeHeading) ||
        (!coarseCentre && task != 10U && leg->heading != 4 && leg->heading != routeHeading)) return;
    if (!imuValid || clockMs - imuStamp > 250U || motionInterrupted() || hostEStop) {
        missionAbort("observation_fault");
        return;
    }
    if (!mechanismState.valid) { missionAbort("observation_pose_invalid"); return; }
    if (mechanismState.running || mechanismActionActive || visionSessionActive() ||
        servoChannelMoving[0] || servoChannelMoving[1] || servoChannelMoving[2]) return;
    if (mechanismState.current.liftDmm != 0U) return;
    if (mechanismState.current.horizontalDmm != 0 ||
        (mechanismState.current.turretDdeg != 530U &&
         mechanismState.current.turretDdeg != 2600U)) {
        /* Coarse recovery's existing raised tray pose is also a legal departure. */
        if (!((task == 8U || task == 15U) &&
              mechanismState.current.horizontalDmm == (task == 15U ? -470 : -450) &&
              mechanismState.current.turretDdeg == 1200U)) return;
    }
    scale = lateral ? routeLateralScale : routeForwardScale;
    if (remaining > missionObservationLeadDistance(missionObservationLeadMs(target),
                                                   cruiseRpm, scale)) return;
    missionObservationTask = task;
    startMechanismPose(target);
    if (!mechanismState.running) { missionAbort("observation_start_failed"); return; }
}

/* 0: normal station preparation, 1: still moving, 2: prepared, -1: aborted. */
static int8_t finishMissionObservation(uint8_t nextIndex)
{
    const MechanismPose *target;
    if (!missionObservationTask) return 0;
    target = missionObservationPose(missionObservationTask);
    if (missionObservationTask + 1U != nextIndex || !mechanismState.valid || !target) {
        missionAbort("observation_failed"); return -1;
    }
    if (!routeWaiting || mechanismState.running || servoChannelMoving[2]) return 1;
    if (mechanismState.current.horizontalDmm != target->horizontalDmm ||
        mechanismState.current.liftDmm != target->liftDmm ||
        mechanismState.current.turretDdeg != target->turretDdeg) {
        missionAbort("observation_failed"); return -1;
    }
    missionObservationTask = 0U;
    return 2;
}

#endif
