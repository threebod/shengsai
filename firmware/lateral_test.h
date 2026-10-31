#ifndef SHENGSAI_LATERAL_TEST_H
#define SHENGSAI_LATERAL_TEST_H

/* A single local navigation leg reuses the formal route's speed profile,
 * lateral scale and heading controller, without workstation actions. */
static uint8_t startLateralTest(int16_t distanceMm)
{
    if (!imuValid || clockMs-imuStamp > 250U || motionInterrupted()) return 0U;
    invalidateNavigation("lateral_test");
    missionButtonReady = 0U;
    presetRouteActive = missionActive = fullRouteRunning = arcRoutePhase = 0U;
    missionArcActive = missionArcExit = rawPickRouteActive = 0U;
    routeWaiting = routeRotating = routeTurnInBand = routeStationAlignment = 0U;
    routeHeading = routeNextHeading = 0;
    routeBaseYaw = routeYaw = imuYaw;
    routeX = routeY = 0.0f;
    routeForward = routeRight = 0;
    routeDriveRpm = routeTurnRpm = routeCorrection = 0.0f;
    routeHeadingPidReset(&routeHeadingPid);
    routeSentValid = routeIndex = 0U;
    navPathCount = 1U;
    navPath[0].x = distanceMm; navPath[0].y = 0;
    navPath[0].arrivalHeading = NAV_HEADING_KEEP;
    routeAuto = 1U;
    routeTick = routeLegStart = routeSettle = clockMs;
    setAllMotorsEnabled(true);
    if (motionInterrupted()) { stopAllMotors(); return 0U; }
    routeActive = navRunning = lateralTestActive = 1U;
    hostHeartbeatActive = 1U; hostHeartbeatStamp = clockMs;
    return 1U;
}
#endif
