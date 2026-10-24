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

/* Original 1mm swept-body calculation, retained independently to verify the
 * cached quarter-turn/reflection samples against every heading and map anchor. */
static uint8_t referenceArcClear(const PresetLeg *before, const PresetLeg *after, uint16_t regions)
{
    int8_t heading = before->from.heading;
    float angle;
    if ((before->from.x != before->to.x || before->from.y != before->to.y) &&
        abs(presetHeadingChange(heading,presetDirection(before->from,before->to))) == 2)
        heading = presetDirection(before->from,before->to);
    angle = heading*1.570796327f;
    float x = before->to.x+sinf(angle)*PRESET_ARC_TRIM;
    float y = before->to.y-cosf(angle)*PRESET_ARC_TRIM;
    float distance = 0.0f, degrees, curvature, step;
    int8_t change = presetHeadingChange(before->from.heading,after->from.heading);
    if (change != 1 && change != -1) return 0U;
    while (distance < ARC_BLEND_LENGTH_MM) {
        if (!presetBodyClear(x,y,angle,regions)) return 0U;
        step = ARC_BLEND_LENGTH_MM-distance < 1.0f ? ARC_BLEND_LENGTH_MM-distance : 1.0f;
        arcBlendShape(distance+step*0.5f,&degrees,&curvature);
        angle = heading*1.570796327f+change*degrees*0.01745329252f;
        x -= sinf(angle)*step; y += cosf(angle)*step; distance += step;
    }
    angle = (heading+change)*1.570796327f;
    for (distance = 0; distance <= 30.0f; distance += 1.0f)
        if (!presetBodyClear(x-sinf(angle)*distance,y+cosf(angle)*distance,angle,regions)) return 0U;
    return 1U;
}

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
    before.from = (PresetPose){1200,300,-1}; before.to = (PresetPose){300,300,-1};
    after.from = (PresetPose){300,300,2}; after.to = (PresetPose){300,1100,2};
    assert(presetArcClear(&before,&after,mask(7,9)));
    assert(referenceArcClear(&before,&after,mask(7,9)));
    after.from.heading = after.to.heading = 0;
    assert(!presetArcClear(&before,&after,mask(7,9))); /* Never blend a reverse/forward cusp. */
    /* Centre clear, rotated body crossing a fixed obstacle: must reject. */
    assert(!presetBodyClear(473.0f,473.0f,0.785398163f,0U));
    assert(presetCostLess((PresetCost){100,0,1300},(PresetCost){0,0,5000}));
    assert(presetCostLess((PresetCost){0,100,125},(PresetCost){0,0,2000}));
    assert(presetCostLess((PresetCost){0,10,100},(PresetCost){1,0,100}));
    {
        static const uint16_t maps[] = {0U,320U,68U,2064U};
        unsigned n, heading, turn, map;
        for (map = 0U; map < sizeof(maps)/sizeof(maps[0]); ++map)
            for (n = 0U; n < PRESET_NODE_COUNT; ++n) for (heading = 0U; heading < 4U; ++heading)
                for (turn = 1U; turn < 4U; turn += 2U) {
                    before.from = before.to = presetNode(n,heading);
                    after.from = after.to = presetNode(n,(heading+turn)%4U);
                    assert(presetArcClear(&before,&after,maps[map]) == referenceArcClear(&before,&after,maps[map]));
                }
    }
}

static void recenterEnvelope(void)
{
    PresetLeg *arrived;
    unsigned i;
    assert(presetBuild(&route,1U,mask(7,9)));
    assert(!presetRecenterClear(&route,80.0f,70.0f));
    for (i = 0U; i < route.count; ++i) if (route.legs[i].taskIndex == 3U && route.legs[i].drive.event) break;
    assert(i < route.count); route.index = (uint16_t)(i+1U);
    assert(presetRecenterClear(&route,80.0f,70.0f));
    assert(presetRecenterClear(&route,-80.0f,-70.0f));
    assert(!presetRecenterClear(&route,0.0f,300.0f)); /* Full body beyond top boundary. */
    assert(!presetRecenterClear(&route,NAN,0.0f));
    route.count = route.index = 1U; arrived = &route.legs[0];
    memset(arrived,0,sizeof(*arrived));
    arrived->from = (PresetPose){400,1100,2}; arrived->to = (PresetPose){400,1200,2};
    arrived->drive.upMm = 150; arrived->drive.heading = 2; arrived->drive.event = "TEMP";
    assert(presetRecenterClear(&route,0.0f,10.0f));
    assert(!presetRecenterClear(&route,0.0f,-1.0f)); /* Region 7 begins at x=550. */
    arrived->drive.event = 0;
    assert(!presetRecenterClear(&route,0.0f,10.0f));
}

static void checkRoute(uint8_t zone)
{
    static const uint8_t events[8] = {1,3,6,8,10,13,15,19};
    static const uint8_t waits[7] = {2,4,7,9,11,14,16};
    static const PresetPose stations[8] = {
        {2100,1200,0},{1200,2080,1},{1200,400,-1},{400,1200,2},
        {1200,2080,1},{1200,400,-1},{400,1200,2},{2250,2250,-1}
    };
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
            PresetPose station, endpoint;
            int nominal, commanded, expected;
            unsigned tuned, span;
            assert(event < 8U && leg->taskIndex == events[event]);
            calibrated = routeDistanceLeg(events[event],zone);
            station = stations[event];
            if (event == 7U && zone == 2U) station.y = 150;
            assert(leg->to.x == station.x && leg->to.y == station.y);
            assert(leg->to.heading == station.heading);
            assert(leg->drive.departureHeading == leg->from.heading && leg->drive.heading == station.heading);
            if (leg->from.heading != station.heading) assert(presetTurnClear(station,route.regions));
            assert(!strcmp(leg->drive.event,calibrated.event));
            nominal = leg->drive.upMm ? leg->to.y-leg->from.y : leg->to.x-leg->from.x;
            commanded = leg->drive.upMm ? leg->drive.upMm : leg->drive.rightMm;
            if (!event) assert(commanded == calibrated.upMm); /* QR keeps its original ingress. */
            else if ((leg->drive.upMm != 0) != (calibrated.upMm != 0)) assert(commanded == nominal);
            else {
                tuned = (unsigned)abs(calibrated.upMm ? calibrated.upMm : calibrated.rightMm);
                span = event == 1U ? 900U : event == 7U ? 150U : 800U;
                expected = (int)(((unsigned)abs(nominal)*tuned+span/2U)/span);
                if (leg->calibrationSpan) {
                    int correction = (int)((leg->calibrationSpan*tuned+span/2U)/span) - leg->calibrationSpan;
                    expected = abs(nominal)+correction;
                }
                assert(commanded == (nominal < 0 ? -expected : expected));
            }
            assert(nominal*commanded > 0);
            endpoint = leg->from; endpoint.x += leg->drive.rightMm; endpoint.y += leg->drive.upMm;
            assert(presetSegmentClear(leg->from,endpoint,route.regions));
            ++event;
        }
        if (leg->arc) {
            assert(!leg->drive.event && i+1U < route.count);
            assert(presetArcClear(leg,&route.legs[i+1U],route.regions));
            assert(referenceArcClear(leg,&route.legs[i+1U],route.regions));
            assert(presetHeadingChange(leg->from.heading,presetDirection(leg->from,leg->to)) == 0 ||
                   abs(presetHeadingChange(leg->from.heading,presetDirection(leg->from,leg->to))) == 2);
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

static void focusedRoutes(void)
{
    unsigned pair;
    for (pair = 0U; pair < 2U; ++pair) {
        unsigned i, nominal = 0U, lateral = 0U, reverse = 0U, arcs = 0U;
        unsigned lowerLeft = 0U, upperLeft = 0U;
        int tempUp[2] = {0,0}, tempRight[2] = {0,0}, secondRawUp = 0, secondRawRight = 0;
        uint16_t regions = pair ? mask(7,9) : mask(3,7);
        float commanded = 0.0f;
        assert(presetBuild(&route,1U,regions)); checkRoute(1U);
        for (i = 0U; i < route.count; ++i) {
            PresetLeg *leg = &route.legs[i];
            unsigned distance = (unsigned)(abs(leg->to.x-leg->from.x)+abs(leg->to.y-leg->from.y));
            int8_t direction = presetDirection(leg->from,leg->to);
            RouteLeg active;
            route.index = (uint16_t)i; active = presetActiveLeg(&route);
            nominal += distance; commanded += (float)(abs(active.upMm)+abs(active.rightMm));
            if (direction != leg->from.heading) {
                if (abs(presetHeadingChange(leg->from.heading,direction)) == 2) reverse += distance;
                else lateral += distance;
            }
            if (leg->arc) {
                ++arcs; commanded += ARC_BLEND_LENGTH_MM;
                if (leg->to.x < 550 && leg->to.y < 550) ++lowerLeft;
                if (leg->to.x < 550 && leg->to.y > 1850) ++upperLeft;
            }
            if (pair) {
                if (leg->taskIndex == 8U || leg->taskIndex == 15U) {
                    unsigned round = leg->taskIndex == 15U;
                    tempUp[round] += leg->drive.upMm; tempRight[round] += leg->drive.rightMm;
                }
                if (leg->taskIndex == 10U) {
                    secondRawUp += leg->drive.upMm; secondRawRight += leg->drive.rightMm;
                }
                /* Both COARSE approaches use the free centre lane instead of
                 * being forced around the bottom-left fixed entry. */
                if (leg->taskIndex == 6U || leg->taskIndex == 13U) {
                    assert(leg->from.x == 1200 && leg->to.x == 1200);
                    assert(leg->drive.event && leg->to.y < leg->from.y);
                }
                /* After either TEMP stop, never retreat south merely to turn. */
                if (leg->taskIndex == 10U || leg->taskIndex == 19U)
                    assert(leg->to.y >= leg->from.y);
            } else if (leg->taskIndex == 3U && leg->drive.event) {
                assert(leg->to.x == 1200 && leg->to.y == 2080);
                assert(leg->from.heading == 1 && leg->drive.heading == 1);
            }
        }
        assert(arcs && route.count < 40U);
        if (pair) assert(lowerLeft == 2U && upperLeft == 2U);
        if (pair) {
            assert(tempUp[0] == 850 && tempUp[1] == 850 && tempRight[0] == -800 && tempRight[1] == -800);
            assert(secondRawUp == 880 && secondRawRight == 850);
        }
        printf("FOCUS zone=1 regions=%s legs=%u nominal_mm=%u commanded_mm=%.1f lateral_mm=%u reverse_mm=%u arcs=%u\n",
               pair ? "7/9" : "3/7",route.count,nominal,commanded,lateral,reverse,arcs);
    }
    {
        /* Same-axis full approaches retain their existing calibration; a
         * short approach from the opposite side cannot have its sign flipped. */
        PresetPose from = {400,400,-1}, to = {1200,400,-1};
        RouteLeg leg = presetStationLeg(routeDistanceLeg(6U,1U),from,to,800U);
        assert(leg.rightMm == 910);
        from.x = 1300;
        leg = presetStationLeg(routeDistanceLeg(6U,1U),from,to,800U);
        assert(leg.rightMm == -114);
    }
}

int main(void)
{
    unsigned zone, a, b, feasible[2] = {0,0}, rejected[2] = {0,0}, arcs = 0U, detours = 0U;
    selection(); geometry(); recenterEnvelope();
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
    focusedRoutes();
    printf("PASS 105 pairs x 2 zones: feasible=%u/%u rejected=%u/%u arcs=%u detours=%u; task order and calibrated approaches\n",
           feasible[0],feasible[1],rejected[0],rejected[1],arcs,detours);
    return 0;
}
