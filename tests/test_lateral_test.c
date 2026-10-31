#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include "../firmware/route_plan.h"
#include "../firmware/map_navigation.h"

static uint8_t imuValid, fault, failEnable, missionButtonReady, presetRouteActive, missionActive;
static uint8_t fullRouteRunning, arcRoutePhase, missionArcActive, missionArcExit, rawPickRouteActive;
static uint8_t routeWaiting, routeRotating, routeTurnInBand, routeStationAlignment;
static uint8_t routeSentValid, routeIndex, navPathCount, routeAuto, routeActive, navRunning, navInitialized;
static uint8_t lateralTestActive, hostHeartbeatActive;
static int8_t routeHeading, routeNextHeading;
static int16_t routeForward, routeRight;
static float imuYaw, routeBaseYaw, routeYaw, routeX, routeY, routeDriveRpm, routeTurnRpm, routeCorrection;
static uint32_t clockMs, imuStamp, routeTick, routeLegStart, routeSettle, hostHeartbeatStamp;
static NavPoint navPath[NAV_PATH_CAPACITY];
static RouteHeadingPid routeHeadingPid;
static unsigned enables, stops, invalidations;
static uint8_t motionInterrupted(void) { return fault; }
static void invalidateNavigation(const char *reason) {
    (void)reason; ++invalidations; navInitialized=navRunning=navPathCount=0;
}
static void setAllMotorsEnabled(bool enabled) {
    assert(enabled); ++enables; if(failEnable) fault=1;
}
static void stopAllMotors(void) { ++stops; routeActive=navRunning=lateralTestActive=0; }
#include "../firmware/lateral_test.h"

static void reset(void) {
    clockMs=imuStamp=500;imuValid=1;imuYaw=37.5f;
    fault=failEnable=routeActive=navRunning=lateralTestActive=hostHeartbeatActive=0;
    enables=stops=invalidations=0;
    missionButtonReady=navInitialized=1;
}
int main(void) {
    int direction;
    for(direction=-1;direction<=1;direction+=2) {
        reset();assert(startLateralTest((int16_t)(direction*1000)));
        assert(enables==1 && !stops && invalidations==1);
        assert(routeActive && navRunning && lateralTestActive && routeAuto && !missionButtonReady);
        assert(!navInitialized && !presetRouteActive && !missionActive && !fullRouteRunning);
        assert(navPathCount==1 && !routeIndex && navPath[0].x==direction*1000 && !navPath[0].y);
        assert(navPath[0].arrivalHeading==NAV_HEADING_KEEP && !routeHeading && !routeNextHeading);
        assert(routeBaseYaw==imuYaw && routeYaw==imuYaw && !routeX && !routeY);
        assert(!routeForward && !routeRight && !routeDriveRpm && !routeCorrection);
        assert(hostHeartbeatActive && hostHeartbeatStamp==clockMs && routeLegStart==clockMs);
    }
    reset();imuValid=0;assert(!startLateralTest(1000) && !enables);
    reset();imuStamp=249;assert(!startLateralTest(1000) && !enables);
    reset();fault=1;assert(!startLateralTest(1000) && !enables);
    reset();failEnable=1;assert(!startLateralTest(1000));
    assert(enables==1 && stops==1 && !routeActive && !navRunning && !lateralTestActive);
    puts("PASS lateral test: signed local leg, no station/turn, formal navigation and heartbeat, IMU/CAN rejection");
    return 0;
}
