#ifndef SHENGSAI_STRAIGHT_TEST_H
#define SHENGSAI_STRAIGHT_TEST_H

/* Start from the current pose: forward only, no turn or workstation actions. */
static uint8_t startStraightTest(void)
{
    static const char start[] = "STRAIGHT_TEST mm=1680 rpm=240";
    if (!imuValid || clockMs-imuStamp > 250U || motionInterrupted()) return 0U;
    invalidateNavigation("straight_test");
    missionButtonReady = missionOffline = 0U;
    presetRouteActive = missionActive = fullRouteRunning = arcRoutePhase = 0U;
    missionArcActive = missionArcExit = rawPickRouteActive = 0U;
    routeWaiting = routeRotating = routeOnlyTurn = routeTurnInBand = routeStationAlignment = 0U;
    routeHeading = routeNextHeading = 0;
    routeBaseYaw = routeYaw = imuYaw;
    routeX = routeY = 0.0f;
    routeForward = routeRight = 0;
    routeDriveRpm = routeTurnRpm = routeCorrection = 0.0f;
    routeHeadingPidReset(&routeHeadingPid);
    routeSentValid = routeIndex = presetExecutionStep = 0U;
    routeRpm = SS_STRAIGHT_TEST_RPM;
    navPathCount = 1U;
    navPath[0].x = 0; navPath[0].y = SS_STRAIGHT_TEST_MM;
    navPath[0].arrivalHeading = NAV_HEADING_KEEP;
    routeAuto = 1U;
    routeTick = routeLegStart = routeSettle = clockMs;
    setAllMotorsEnabled(true);
    if (motionInterrupted()) { stopAllMotors(); return 0U; }
    taskLogReset(&taskLog);
    taskLogAppend(&taskLog, (const uint8_t *)start, sizeof(start)-1U);
    routeActive = navRunning = straightTestActive = 1U;
    hostHeartbeatActive = 1U; hostHeartbeatStamp = clockMs;
    return 1U;
}

static void finishStraightTest(uint8_t aborted)
{
    static const char complete[] = "STRAIGHT_TEST result=estimated_complete";
    static const char failed[] = "STRAIGHT_TEST result=aborted";
    if (!straightTestActive) return;
    taskLogAppend(&taskLog, (const uint8_t *)(aborted ? failed : complete),
                  aborted ? sizeof(failed)-1U : sizeof(complete)-1U);
    straightTestActive = 0U;
}
#endif
