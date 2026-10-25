#ifndef SHENGSAI_OBSTACLE_TURRET_H
#define SHENGSAI_OBSTACLE_TURRET_H

/* Both validated obstacle-selection inputs prepare all three servos.
 * Axis validity must never be established here. */
static uint8_t prepareObstacleTurret(void)
{
    if (emergencyStop || hostEStop || jogCanFault || hostStartPending || obstacleTurretPending ||
        motionMode || routeActive || mechanismState.running || mechanismActionActive ||
        visionSessionActive() || servoChannelMoving[0] || servoChannelMoving[1] ||
        servoChannelMoving[2]) return 0U;
    obstacleTurretReady = 0U;
    obstacleTurretPending = 1U;
    servoSetTarget(2U, rawPickInitial.gripperCloseDeg, rawPickInitial.gripperDps10);
    servoSetTarget(3U, 26U, rawPickInitial.platformDps10);
    servoSetTarget(4U, 53U, 1200U);
    return 1U;
}

static void serviceObstacleTurret(void)
{
    if (!obstacleTurretPending) return;
    if (emergencyStop || hostEStop || jogCanFault) {
        hostOperationFailed = 1U;
        stopServoMotion();
        invalidateMechanism("obstacle_turret_fault");
        return;
    }
    if (servoChannelMoving[0] || servoChannelMoving[1] || servoChannelMoving[2]) return;
    if (currentAngleMdeg[0] != (uint32_t)rawPickInitial.gripperCloseDeg * 1000U ||
        currentAngleMdeg[1] != 26000U || currentAngleMdeg[2] != 53000U) {
        hostOperationFailed = 1U;
        stopServoMotion();
        invalidateMechanism("obstacle_turret_interrupted");
        return;
    }
    mechanismState.current.turretDdeg = 530U;
    obstacleTurretPending = 0U;
    obstacleTurretReady = 1U;
}

static void serviceObstacleHostStart(void)
{
    if (hostStartPending && (hostEStop || emergencyStop || jogCanFault || hostCancelled ||
                            hostOperationFailed || !hostConfirmed ||
                            (hostHeartbeatActive && clockMs - hostHeartbeatStamp > HOST_HEARTBEAT_TIMEOUT_MS))) {
        hostStartPending = 0U;
        hostOperationFailed = 1U;
        stopServoMotion();
    } else if (hostStartPending && !obstacleTurretPending) {
        uint8_t startCommand = hostStartPending;
        char command[32];
        hostStartPending = 0U;
        if (!obstacleTurretReady) hostOperationFailed = 1U;
        else if (startCommand == SS_MISSION) {
            missionButtonRpm = hostStartRpm;
            missionButtonReady = 1U;
            startButtonMission(hostStartZone);
            if (!missionActive) hostOperationFailed = 1U;
            else missionOffline = 1U;
        } else {
            sprintf(command, "route auto %u %u", hostStartZone, hostStartRpm);
            armed = 1U;
            hostExecuting = 1U;
            processCommand(command);
            hostExecuting = 0U;
            if (hostError || jogCanFault) hostOperationFailed = 1U;
        }
    }
}

#endif
