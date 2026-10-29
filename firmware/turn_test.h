#ifndef SHENGSAI_TURN_TEST_H
#define SHENGSAI_TURN_TEST_H

static uint8_t startTurnTest(int8_t direction)
{
    if ((direction!=1 && direction!=-1) || !imuValid ||
        clockMs-imuStamp>250U || motionInterrupted()) return 0U;
    invalidateNavigation("turn_test");
    missionButtonReady=missionOffline=0U;
    presetRouteActive=missionActive=fullRouteRunning=arcRoutePhase=0U;
    missionArcActive=missionArcExit=rawPickRouteActive=0U;
    routeWaiting=routeTurnInBand=routeStationAlignment=0U;
    routeHeading=0;routeNextHeading=direction;
    routeBaseYaw=imuYaw;routeYaw=routeBaseYaw+90.0f*direction*yawSign;
    routeX=routeY=0.0f;routeForward=routeRight=0;
    routeDriveRpm=routeTurnRpm=routeCorrection=0.0f;
    routeHeadingPidReset(&routeHeadingPid);routeTurnPidReset(&routeTurnPid);
    routeSentValid=routeIndex=presetExecutionStep=0U;
    routeTurnRpmLimit=(uint16_t)ROUTE_TURN_RPM;
    setAllMotorsEnabled(true);
    if (motionInterrupted()) {stopAllMotors();return 0U;}
    taskLogReset(&taskLog);
    turnTestLogBegin(&taskLog,&turnTestLog,clockMs,direction,yawSign,routeBaseYaw,routeYaw,motorTrim+1);
    routeTick=routeLegStart=routeTurnStart=clockMs;
    routeOnlyTurn=routeRotating=routeActive=turnTestActive=1U;
    hostHeartbeatActive=1U;hostHeartbeatStamp=clockMs;
    return 1U;
}

static void finishTurnTest(uint8_t aborted)
{
    float yaw;uint32_t stamp;
    if (!turnTestActive) return;
    __disable_irq();yaw=imuYaw;stamp=imuStamp;__enable_irq();
    turnTestLogEnd(&taskLog,&turnTestLog,clockMs,stamp,routeYaw,yaw,
                   (uint8_t)(aborted || motionInterrupted()));
    turnTestActive=0U;
}
#endif
