#ifndef SHENGSAI_PLACEMENT_TEST_RUNTIME_H
#define SHENGSAI_PLACEMENT_TEST_RUNTIME_H

static uint8_t placementTestHeadingReady(void)
{
    float yaw;
    uint32_t sample;
    int8_t ready;
    __disable_irq(); yaw = imuYaw; sample = imuStamp; __enable_irq();
    ready = placementHeadingStep(&placementHeadingGate,routeYaw,yaw,sample,clockMs,yawSign);
    if (ready < 0) { missionAbort("test_heading_timeout"); return 0U; }
    sendRouteSpeeds(0,0,routeRound(placementHeadingGate.rpm));
    recordRouteHeading(routeYaw,yaw,sample,'T');
    return (uint8_t)(ready == 1);
}

static void servicePlacementTestSafety(void)
{
    if (!placementTestActive) return;
    if (!imuValid || clockMs-imuStamp > 250U || motionInterrupted() || hostEStop) {
        missionAbort("test_imu_or_can"); return;
    }
    if (mechanismActionActive &&
        (missionPhase == MISSION_COARSE_PLACE || missionPhase == MISSION_COARSE_PICK ||
         missionPhase == MISSION_TEMP_PLACE) && routeAbs(headingError(routeYaw,imuYaw)) > 0.5f)
        missionAbort("test_heading_during_action");
}

static uint8_t startPlacementTest(void)
{
    uint8_t i;
    placementTestActive = 1U;
    memset(&placementHeadingGate,0,sizeof(placementHeadingGate));
    placementTestRetries = 0U;
    armed = 1U;
    processRouteCommand("route mission 2 100");
    armed = 0U;
    if (!missionActive || !routeActive) { placementTestActive = 0U; return 0U; }
    memset(&missionTask,0,sizeof(missionTask));
    for (i = 0U; i < 3U; ++i) {
        missionTask.colors[0][i] = (uint8_t)(i+1U);
        missionTask.rings[0][i] = (uint8_t)(i+1U);
    }
    missionTaskValid = 1U;
    missionOffline = 0U;
    hostHeartbeatActive = 1U; hostHeartbeatStamp = clockMs;
    serialSendString("ALIGN_TEST zone=2 layer=1 rpm=100 order=123 yaw_tolerance_cd=50 stable_ms=200\r\n");
    return 1U;
}
#endif
