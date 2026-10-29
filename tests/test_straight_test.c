#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include "../firmware/route_plan.h"
#include "../firmware/map_navigation.h"
#include "../firmware/route_heading_log.h"
#include "../firmware/route_pid_log.h"

static uint8_t imuValid, fault, failEnable, missionButtonReady, missionOffline;
static uint8_t presetRouteActive, missionActive, fullRouteRunning, arcRoutePhase;
static uint8_t missionArcActive, missionArcExit, rawPickRouteActive, presetExecutionStep;
static uint8_t routeWaiting, routeRotating, routeOnlyTurn, routeTurnInBand, routeStationAlignment;
static uint8_t routeSentValid, routeIndex, navPathCount, routeAuto, routeActive, navRunning;
static uint8_t straightTestActive, hostHeartbeatActive;
static int8_t routeHeading, routeNextHeading;
static int16_t routeForward, routeRight;
static uint16_t routeRpm;
static float imuYaw, routeBaseYaw, routeYaw, routeX, routeY, routeDriveRpm, routeTurnRpm, routeCorrection;
static uint32_t clockMs, imuStamp, routeTick, routeLegStart, routeSettle, hostHeartbeatStamp;
static NavPoint navPath[NAV_PATH_CAPACITY];
static RouteHeadingPid routeHeadingPid;
static TaskLog taskLog;
static unsigned enables, stops, invalidations;
static uint8_t motionInterrupted(void) { return fault; }
static void invalidateNavigation(const char *reason) {
    assert(!strcmp(reason,"straight_test")); ++invalidations; navRunning=navPathCount=0;
}
static void setAllMotorsEnabled(bool enabled) {
    assert(enabled); ++enables; if(failEnable) fault=1;
}
static void finishStraightTest(uint8_t aborted);
static void stopAllMotors(void) { ++stops; routeActive=navRunning=0; finishStraightTest(1); }
#include "../firmware/straight_test.h"

static void reset(void) {
    clockMs=imuStamp=500;imuValid=1;imuYaw=37.5f;
    fault=failEnable=routeActive=navRunning=straightTestActive=hostHeartbeatActive=0;
    enables=stops=invalidations=0;
    missionButtonReady=missionOffline=presetRouteActive=missionActive=fullRouteRunning=1;
    arcRoutePhase=missionArcActive=missionArcExit=rawPickRouteActive=1;
    presetExecutionStep=2;routeRpm=100;
    taskLogReset(&taskLog);
    taskLogAppend(&taskLog,(const uint8_t *)"OLD",3);
}
static void contains(const char *expected) {
    char text[2048];
    assert(taskLog.end<sizeof(text));
    memcpy(text,taskLog.bytes,taskLog.end);text[taskLog.end]=0;
    assert(strstr(text,expected));
    assert(!strstr(text,"OLD"));
}
int main(void) {
    RouteHeadingLog yawLog={0};RoutePidLog pidLog={0};
    RoutePidSample sample={0};RouteHeadingPidGains gains={2.0f,0.25f,0.12f};
    uint32_t task;
    reset();task=taskLog.task;assert(startStraightTest());
    assert(enables==1 && !stops && invalidations==1 && taskLog.task==task+1);
    assert(routeActive && navRunning && straightTestActive && routeAuto);
    assert(!missionButtonReady && !missionOffline && !presetRouteActive && !missionActive && !fullRouteRunning);
    assert(!arcRoutePhase && !missionArcActive && !missionArcExit && !rawPickRouteActive);
    assert(navPathCount==1 && !routeIndex && !navPath[0].x && navPath[0].y==1680);
    assert(navPath[0].arrivalHeading==NAV_HEADING_KEEP && !routeHeading && !routeNextHeading);
    assert(routeBaseYaw==imuYaw && routeYaw==imuYaw && !routeX && !routeY);
    assert(!routeForward && !routeRight && !routeDriveRpm && !routeCorrection && !presetExecutionStep);
    assert(routeRpm==240 && hostHeartbeatActive && hostHeartbeatStamp==clockMs && routeLegStart==clockMs);
    contains("STRAIGHT_TEST mm=1680 rpm=240");
    routeHeadingLogRecord(&taskLog,&yawLog,500,500,0,0,0,'D',imuYaw,imuYaw);
    sample.now=sample.sample=500;sample.dt=20;sample.yawSign=1;
    sample.target=sample.actual=imuYaw;sample.scale=1;
    routePidLogRecord(&taskLog,&pidLog,&sample,&gains,&routeHeadingPid);
    contains("PIDCFG span=");contains("PID 500,500,");contains("tpl=0 phase=D");
    routeHeadingLogRecord(&taskLog,&yawLog,2700,2700,0,0,0,'A',imuYaw,imuYaw-0.4f);
    finishStraightTest(0);assert(!straightTestActive);contains("phase=A");
    contains("STRAIGHT_TEST result=estimated_complete");
    task=taskLog.end;finishStraightTest(1);assert(taskLog.end==task);
    reset();assert(startStraightTest());stopAllMotors();assert(!straightTestActive);
    contains("STRAIGHT_TEST result=aborted");
    reset();task=taskLog.task;imuValid=0;assert(!startStraightTest() && !enables && taskLog.task==task);
    reset();imuStamp=249;assert(!startStraightTest() && !enables);
    reset();fault=1;assert(!startStraightTest() && !enables);
    reset();task=taskLog.task;failEnable=1;assert(!startStraightTest());
    assert(enables==1 && stops==1 && !routeActive && !navRunning && !straightTestActive && taskLog.task==task);
    reset();imuYaw=-179.9f;assert(startStraightTest() && routeYaw==imuYaw);finishStraightTest(1);
    puts("PASS straight test: forward 1680mm/240RPM, current yaw, route PID logs, completion/abort and IMU/CAN rejection");
    return 0;
}
