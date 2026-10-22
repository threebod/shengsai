#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include "../shared/protocol.h"
#include "../firmware/preset_route.h"
#include "../firmware/tjc_obstacles.h"

static PresetRoute route;
static uint16_t mask(unsigned first, unsigned second)
{ return (uint16_t)((1U << (first-1U)) | (1U << (second-1U))); }

static void selection(void)
{
    unsigned value, valid = 0U, a, b;
    for (value = 0U; value <= 65535U; ++value) valid += ss_obstacle_regions_valid((uint16_t)value);
    assert(valid == 105U);
    for (a = 1U; a <= 15U; ++a) for (b = a+1U; b <= 15U; ++b) {
        TjcObstacleParser parser = {{0},0};
        const uint8_t bytes[6] = {0xA5,0x5A,0x4F,(uint8_t)a,(uint8_t)b,(uint8_t)(0x4F+a+b)};
        unsigned i;
        for (i = 0U; i < 5U; ++i) assert(!tjc_obstacles_feed(&parser,bytes[i]));
        assert(tjc_obstacles_feed(&parser,bytes[5]) == mask(a,b));
    }
    assert(ssObstacleRegions[0].y0 == 1850 && ssObstacleRegions[12].y1 == 550);
    assert(ssObstacleRegions[1].x0 == 550 && ssObstacleRegions[2].x0 == 1400);
    assert(ssObstacleRegions[7].x0 == 1000 && ssObstacleRegions[7].y0 == 1000);
    {
        uint8_t request[5] = {2,0x10,0x08,240,0};
        assert(ss_route_request_valid(request,5U));
        assert(!ss_route_request_valid(request,4U));
        assert(!ss_route_request_valid(request,6U));
        request[0] = 0; assert(!ss_route_request_valid(request,5U)); request[0] = 1;
        request[1] = 1; request[2] = 0; assert(!ss_route_request_valid(request,5U));
        request[1] = 3; request[3] = 9; assert(!ss_route_request_valid(request,5U));
        request[3] = 251; assert(!ss_route_request_valid(request,5U));
    }
}

static void geometry(void)
{
    PresetPose a = {1200,1200,0}, b = {1200,2080,0}, corner = {400,400,0};
    PresetLeg before = {{0}, {2100,1200,0}, {2100,2080,0}, 3, 0};
    PresetLeg after = {{0}, {2100,2080,1}, {1200,2080,1}, 3, 0};
    /* Region 5 blocks an edge; region 2 only blocks its whole-body turn at RAW. */
    assert(!presetSegmentClear(a,b,mask(5,12)));
    assert(presetSegmentClear(a,b,mask(2,12)));
    assert(!presetTurnClear(b,mask(2,12)));
    assert(presetTurnClear(corner,mask(5,12)));
    corner.x = 2250; assert(!presetTurnClear(corner,0U));
    assert(presetArcClear(&before,&after,mask(5,12)));
    assert(!presetArcClear(&before,&after,mask(3,12)));
    /* Centre clear, rotated body crossing a fixed obstacle: must reject. */
    assert(!presetBodyClear(473.0f,473.0f,0.785398163f,0U));
    assert(presetCostLess((PresetCost){0,5000,9000},(PresetCost){1,0,1}));
    assert(presetCostLess((PresetCost){0,0,9000},(PresetCost){0,1,1}));
}

static void checkRoute(uint8_t zone)
{
    static const uint8_t events[8] = {1,3,6,8,10,13,15,19};
    static const uint8_t waits[7] = {2,4,7,9,11,14,16};
    unsigned i, event = 0U, wait = 0U;
    uint8_t logical = 0U;
    PresetPose previous = {2250,2250,0};
    if (zone == 2U) previous.y = 150;
    for (i = 0U; i < route.count; ++i) {
        PresetLeg *leg = &route.legs[i];
        RouteLeg active;
        route.index = (uint16_t)i; active = presetActiveLeg(&route);
        assert(presetSegmentClear(leg->from,leg->to,route.regions));
        assert(previous.x == leg->from.x && previous.y == leg->from.y);
        if (previous.heading != leg->from.heading && !(i && route.legs[i-1U].arc))
            assert(presetTurnClear(leg->from,route.regions));
        assert((active.upMm == 0) != (active.rightMm == 0));
        if (leg->drive.event) {
            RouteLeg calibrated;
            assert(event < 8U && leg->taskIndex == events[event]);
            calibrated = routeDistanceLeg(events[event],zone);
            assert(leg->drive.upMm == calibrated.upMm && leg->drive.rightMm == calibrated.rightMm);
            assert((calibrated.heading == 4 || leg->drive.heading == calibrated.heading) && !strcmp(leg->drive.event,calibrated.event));
            ++event;
        }
        if (leg->arc) {
            assert(!leg->drive.event && i+1U < route.count);
            assert(presetArcClear(leg,&route.legs[i+1U],route.regions));
            assert(presetDirection(leg->from,leg->to) == leg->from.heading);
            assert(presetDirection(route.legs[i+1U].from,route.legs[i+1U].to) == route.legs[i+1U].from.heading);
        }
        previous = leg->to;
    }
    assert(event == 8U && previous.x == 2250 && previous.y == (zone == 1U ? 2250 : 150));
    route.index = 0U;
    while (route.index < route.count) {
        uint8_t next = presetAdvance(&route,&logical);
        if (next == 2U) { assert(wait < 7U && logical == waits[wait]); ++wait; }
        else if (!next) assert(route.index == route.count);
        else assert(logical == route.legs[route.index].taskIndex);
    }
    assert(wait == 7U);
}

int main(void)
{
    unsigned zone, a, b, feasible[2] = {0,0}, rejected[2] = {0,0}, arcs = 0U, detours = 0U;
    selection(); geometry();
    assert(!presetBuild(&route,0,mask(5,12)) && !route.count);
    assert(!presetBuild(&route,1,0U) && !route.count);
    assert(!presetBuild(&route,1,0x8001U) && !route.count);
    for (zone = 1U; zone <= 2U; ++zone) for (a = 1U; a <= 15U; ++a) for (b = a+1U; b <= 15U; ++b) {
        if (presetBuild(&route,(uint8_t)zone,mask(a,b))) {
            unsigned i;
            ++feasible[zone-1U]; checkRoute((uint8_t)zone);
            for (i = 0U; i < route.count; ++i) arcs += route.legs[i].arc;
            if (route.count > 20U) ++detours;
        } else { ++rejected[zone-1U]; assert(!route.count && !route.index); }
    }
    assert(feasible[0] && feasible[1] && rejected[0] && rejected[1] && arcs && detours);
    assert(presetBuild(&route,1U,mask(5,12)));
    assert(!presetBuild(&route,2U,mask(5,12)) && !route.count); /* blocked start-zone 2 approach */
    printf("PASS 105 pairs x 2 zones: feasible=%u/%u rejected=%u/%u arcs=%u detours=%u; task order and calibrated approaches\n",
           feasible[0],feasible[1],rejected[0],rejected[1],arcs,detours);
    return 0;
}
