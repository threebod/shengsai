#ifndef SHENGSAI_PLACEMENT_TEST_H
#define SHENGSAI_PLACEMENT_TEST_H
#include "preset_route.h"
#include "straight_control.h"

/* Zone 2 -> lower approach -> COARSE, followed by the original COARSE->TEMP
 * template. No QR, pickup, second layer or return-home leg. */
static uint8_t placementTestBuild(PresetRoute *route, uint16_t regions)
{
    static const PresetPose points[4] = {
        {2250,150,0}, {2100,150,0}, {2100,400,0}, {1200,400,-1}
    };
    unsigned i;
    memset(route,0,sizeof(*route));
    if (!ss_obstacle_regions_valid(regions)) return 0U;
    route->regions = regions;
    for (i = 0U; i < 3U; ++i) {
        PresetLeg *leg = &route->legs[i];
        leg->from = points[i]; leg->to = points[i+1U];
        leg->drive.rightMm = (int16_t)(leg->to.x-leg->from.x);
        leg->drive.upMm = (int16_t)(leg->to.y-leg->from.y);
        leg->drive.departureHeading = 0;
        leg->drive.heading = leg->to.heading;
        leg->drive.event = i == 2U ? "COARSE" : (const char *)0;
        leg->taskIndex = 6U;
        if (!presetSegmentClear(leg->from,leg->to,regions)) goto failed;
    }
    route->count = 3U;
    if (presetGroupConnect(route,PRESET_GROUP_CT,8U,"TEMP")) return 1U;
failed:
    route->count = route->index = 0U;
    return 0U;
}

typedef struct {
    RouteTurnPid pid;
    uint32_t started, tick, stable;
    float rpm;
    uint8_t active, inBand, corrected;
} PlacementHeadingGate;

/* -1 timeout/stale data, 0 turning or settling, 1 stable on fresh IMU data. */
static int8_t placementHeadingStep(PlacementHeadingGate *gate, float target,
                                  float yaw, uint32_t sample, uint32_t now,
                                  int8_t yawSign)
{
    float error = headingError(target,yaw);
    uint32_t dt;
    if (!gate->active) {
        memset(gate,0,sizeof(*gate));
        gate->active = 1U; gate->started = gate->tick = now;
        routeTurnPidReset(&gate->pid);
    }
    if (now-sample > 250U || now-gate->started > 10000U) return -1;
    dt = now-gate->tick;
    if (dt > 150U) return -1;
    if (dt < 20U) return 0;
    gate->tick = now;
    if (routeAbs(error) > 0.5f) {
        gate->corrected = 1U; gate->inBand = 0U;
        gate->rpm = routeTurnRamp(gate->rpm,
            routeTurnPidStep(&gate->pid,error,sample,ROUTE_TURN_RPM,yawSign),dt);
        return 0;
    }
    gate->rpm = 0.0f;
    if (!gate->inBand) { gate->inBand = 1U; gate->stable = now; }
    if (now-gate->stable >= 200U && (int32_t)(sample-gate->stable) >= 200) {
        gate->active = 0U;
        return 1;
    }
    return 0;
}
#endif
