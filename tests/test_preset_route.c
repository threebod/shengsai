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
        if (!presetBodyClearScope(x,y,sinf(angle),cosf(angle),regions,
                                  (uint8_t)(before->arc != PRESET_ARC_FOUR_POINT))) return 0U;
        step = ARC_BLEND_LENGTH_MM-distance < 1.0f ? ARC_BLEND_LENGTH_MM-distance : 1.0f;
        arcBlendShape(distance+step*0.5f,&degrees,&curvature);
        angle = heading*1.570796327f+change*degrees*0.01745329252f;
        x -= sinf(angle)*step; y += cosf(angle)*step; distance += step;
    }
    angle = (heading+change)*1.570796327f;
    for (distance = 0; distance <= 30.0f; distance += 1.0f)
        if (!presetBodyClearScope(x-sinf(angle)*distance,y+cosf(angle)*distance,sinf(angle),cosf(angle),regions,
                                  (uint8_t)(before->arc != PRESET_ARC_FOUR_POINT))) return 0U;
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
        PresetPose from = {400,400,-1}, to = {1200,400,-1};
        RouteLeg leg = presetStationLeg(routeDistanceLeg(6U,1U),from,to,800U);
        assert(leg.rightMm == 910);
        from.x = 1300;
        leg = presetStationLeg(routeDistanceLeg(6U,1U),from,to,800U);
        assert(leg.rightMm == -114); /* A short opposite approach cannot reverse its sign. */
    }
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
            int nominal, commanded;
            assert(event < 8U && leg->taskIndex == events[event]);
            calibrated = routeDistanceLeg(events[event],zone);
            station = stations[event];
            if (event == 7U) {
                if (zone == 2U) station.y = 150;
                else station.heading = 1;
            }
            assert(leg->to.x == station.x && leg->to.y == station.y);
            assert(leg->to.heading == station.heading);
            assert(leg->drive.departureHeading == leg->from.heading && leg->drive.heading == station.heading);
            if (leg->from.heading != station.heading) assert(presetTurnClear(station,route.regions));
            assert(!strcmp(leg->drive.event,calibrated.event));
            nominal = leg->drive.upMm ? leg->to.y-leg->from.y : leg->to.x-leg->from.x;
            commanded = leg->drive.upMm ? leg->drive.upMm : leg->drive.rightMm;
            assert(nominal*commanded > 0);
            endpoint = leg->from; endpoint.x += leg->drive.rightMm; endpoint.y += leg->drive.upMm;
            assert(presetSegmentClear(leg->from,endpoint,route.regions));
            ++event;
        }
        if (leg->arc) {
            if (leg->arc == PRESET_ARC_FOUR_POINT)
                assert(((leg->taskIndex == 8U || leg->taskIndex == 15U) && leg->drive.rightMm == -860) ||
                       ((leg->taskIndex == 10U || leg->taskIndex == 19U) && leg->drive.upMm == 880) ||
                       ((leg->taskIndex == 6U || leg->taskIndex == 13U) &&
                        (leg->drive.rightMm == -830 || leg->drive.upMm == -1740)));
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

static void coarseArcPreserved(void)
{
    unsigned zone, round;
    for (zone = 1U; zone <= 2U; ++zone) {
        assert(presetBuild(&route,(uint8_t)zone,mask(7,9)));
        for (round = 0U; round < 2U; ++round) {
            unsigned i, arcs = 0U, legs = 0U;
            uint8_t task = round ? 15U : 8U;
            int up = 0, right = 0;
            for (i = 0U; i < route.count; ++i) if (route.legs[i].taskIndex == task) {
                const PresetLeg *leg = &route.legs[i];
                ++legs;
                assert(leg->drive.upMm >= 0 && leg->drive.rightMm <= 0);
                up += leg->drive.upMm; right += leg->drive.rightMm;
                if (leg->arc) {
                    RouteLeg active;
                    PresetLeg modern = *leg;
                    ++arcs;
                    assert(leg->arc == PRESET_ARC_FOUR_POINT);
                    assert(leg->to.x == 400 && leg->to.y == 400);
                    assert(leg->drive.rightMm == -860);
                    route.index = (uint16_t)i; active = presetActiveLeg(&route);
                    assert(active.rightMm == -(860-PRESET_ARC_TRIM));
                    assert(referenceArcClear(leg,&route.legs[i+1U],route.regions));
                    modern.arc = 1U;
                    assert(!presetArcClear(&modern,&route.legs[i+1U],route.regions));
                } else {
                    RouteLeg active;
                    route.index = (uint16_t)i; active = presetActiveLeg(&route);
                    assert(active.upMm == 601 && !active.rightMm);
                }
            }
            assert(legs == 2U && arcs == 1U && up == 850 && right == -860);
        }
    }
}

static void fourPointObstacleSelection(void)
{
    static const unsigned points[] = {5,7,9,11}, blocked[] = {10,13,14};
    const PresetPose start = {1200,400,-1};
    unsigned a, b, round;
    for (a = 0U; a < 4U; ++a) for (b = a+1U; b < 4U; ++b) for (round = 0U; round < 2U; ++round) {
        memset(&route,0,sizeof(route)); route.regions = mask(points[a],points[b]);
        assert(presetTryCandidate(&route,start,&presetCoarseToTemp[0],round ? 15U : 8U,"TEMP",2));
        assert(route.count == 2U && route.legs[0].arc == PRESET_ARC_FOUR_POINT);
        route.index = 0U; assert(presetActiveLeg(&route).rightMm == -611);
        route.index = 1U; assert(presetActiveLeg(&route).upMm == 601);
    }
    for (a = 0U; a < 3U; ++a) {
        memset(&route,0,sizeof(route)); route.regions = mask(blocked[a],5U);
        assert(!presetTryCandidate(&route,start,&presetCoarseToTemp[0],8U,"TEMP",2));
        assert(!route.count && !route.index);
    }
}

static void tempReturnArcPreserved(void)
{
    const PresetPose start = {400,1200,2};
    static const unsigned points[] = {5,7,9,11}, blocked[] = {1,2,4};
    unsigned zone, i, a, b;
    for (zone = 1U; zone <= 2U; ++zone) {
        unsigned legs = 0U;
        assert(presetBuild(&route,(uint8_t)zone,mask(7,9)));
        for (i = 0U; i < route.count; ++i) if (route.legs[i].taskIndex == 10U) {
            const PresetLeg *leg = &route.legs[i];
            RouteLeg active;
            route.index = (uint16_t)i; active = presetActiveLeg(&route);
            assert(leg->drive.upMm >= 0 && leg->drive.rightMm >= 0);
            if (!legs) {
                assert(leg->from.x == 400 && leg->from.y == 1200);
                assert(leg->to.x == 400 && leg->to.y == 2080);
                assert(leg->drive.upMm == 880 && leg->arc == PRESET_ARC_FOUR_POINT);
                assert(active.upMm == 631 && !active.rightMm);
            } else {
                assert(!leg->arc && !active.upMm && active.rightMm == 601);
                assert(leg->drive.rightMm == 850 && !strcmp(leg->drive.event,"RAW_2"));
            }
            ++legs;
        }
        assert(legs == 2U);
    }
    for (a = 0U; a < 4U; ++a) for (b = a+1U; b < 4U; ++b) {
        memset(&route,0,sizeof(route)); route.regions = mask(points[a],points[b]);
        assert(presetTryCandidate(&route,start,&presetTempToRaw[0],10U,"RAW_2",1));
        assert(route.count == 2U && route.legs[0].arc == PRESET_ARC_FOUR_POINT);
    }
    for (a = 0U; a < 3U; ++a) {
        memset(&route,0,sizeof(route)); route.regions = mask(blocked[a],11U);
        assert(!presetTryCandidate(&route,start,&presetTempToRaw[0],10U,"RAW_2",1));
        assert(!route.count && !route.index);
    }
}

static void remainingArcsPreserved(void)
{
    const PresetPose raw = {1200,2080,1}, coarse = {1200,400,-1};
    const PresetPose temp = {400,1200,2}, qr = {2100,1200,0};
    struct ArcCase {
        const PresetCandidate *candidates;
        PresetPose start;
        uint16_t regions;
        uint8_t task, arcs;
        int8_t heading;
    } cases[] = {
        {presetRawToCoarse,raw,mask(8,9),6U,2U,-1},
        {presetRawToCoarse,raw,mask(4,5),6U,2U,-1},
        {presetCoarseToTemp,coarse,mask(13,7),8U,2U,2},
        {presetTempToRaw,temp,mask(7,9),10U,1U,1},
        {presetTempToRaw,temp,mask(4,5),10U,2U,1},
        {presetQr1ToRaw,qr,mask(6,5),3U,1U,1},
        {presetQr2ToRaw,qr,mask(6,5),3U,1U,1}
    };
    unsigned c, i;
    for (c = 0U; c < sizeof(cases)/sizeof(cases[0]); ++c) {
        unsigned arcs = 0U;
        memset(&route,0,sizeof(route)); route.regions = cases[c].regions;
        assert(presetFixedConnect(&route,cases[c].start,cases[c].candidates,cases[c].task,"TEST",cases[c].heading));
        for (i = 0U; i < route.count; ++i) if (route.legs[i].arc) {
            ++arcs;
            assert(referenceArcClear(&route.legs[i],&route.legs[i+1U],route.regions));
        }
        if (arcs != cases[c].arcs) fprintf(stderr,"missing arcs: case=%u task=%u actual=%u expected=%u\n",c,cases[c].task,arcs,cases[c].arcs);
        assert(arcs == cases[c].arcs);
    }
    {
        /* Full missions: preserve the old arc count at each workstation,
         * including both repeated rounds and both start-zone tables. */
        static const unsigned focus[][5] = {
            {7,9,0,1,1}, {8,9,2,1,1}, {4,5,2,1,2}, {7,13,0,2,1}
        };
        unsigned zone;
        for (zone = 1U; zone <= 2U; ++zone) for (c = 0U; c < 4U; ++c) {
            unsigned counts[ROUTE_COUNT] = {0};
            assert(presetBuild(&route,(uint8_t)zone,mask(focus[c][0],focus[c][1])));
            for (i = 0U; i < route.count; ++i)
                if (route.legs[i].arc) ++counts[route.legs[i].taskIndex];
            assert(counts[6] == focus[c][2] && counts[13] == focus[c][2]);
            assert(counts[8] == focus[c][3] && counts[15] == focus[c][3]);
            assert(counts[10] == focus[c][4] && counts[3] == 1U);
        }
        /* Region 3 blocks a rotation at RAW. The QR outside arrival must
         * retain the working heading rather than requiring a final pivot. */
        assert(presetBuild(&route,1U,mask(3,12)));
        assert(presetBuild(&route,2U,mask(3,6)));
    }
}

static void candidateCase(const PresetCandidate candidates[3], PresetPose start,
                          uint16_t regions, uint8_t task, int8_t heading,
                          unsigned count, int firstX, int firstY)
{
    unsigned i;
    uint8_t built;
    memset(&route,0,sizeof(route)); route.regions = regions;
    built = presetFixedConnect(&route,start,candidates,task,"TEST",heading);
    if (!built) fprintf(stderr,"candidate failed: task=%u regions=%u expected=%d/%d\n",task,regions,firstX,firstY);
    assert(built);
    assert(route.count == count);
    assert(route.legs[0].to.x == firstX && route.legs[0].to.y == firstY);
    assert(route.legs[count-1U].drive.event && route.legs[count-1U].to.heading == heading);
    for (i = 0U; i < count; ++i) {
        const PresetLeg *leg = &route.legs[i];
        if (leg->arc)
            assert(referenceArcClear(leg,&route.legs[i+1U],regions));
        if (i && route.legs[i-1U].to.heading != leg->from.heading && !route.legs[i-1U].arc)
            assert(presetTurnClear(leg->from,regions));
    }
}

static void fixedCandidatePriorities(void)
{
    const PresetPose raw = {1200,2080,1}, coarse = {1200,400,-1};
    const PresetPose temp = {400,1200,2}, qr = {2100,1200,0};
    unsigned zone, variant;
    /* Region 8 blocks the centre; adding 4 also blocks the left detour. */
    candidateCase(presetRawToCoarse,raw,mask(7,9),6U,-1,1,1200,400);
    candidateCase(presetRawToCoarse,raw,mask(8,9),6U,-1,3,400,2080);
    candidateCase(presetRawToCoarse,raw,mask(4,8),6U,-1,4,2100,2080);
    candidateCase(presetCoarseToTemp,coarse,mask(7,9),8U,2,2,400,400);
    candidateCase(presetCoarseToTemp,coarse,mask(10,9),8U,2,2,1200,1200);
    candidateCase(presetCoarseToTemp,coarse,mask(13,7),8U,2,3,1200,2080);
    candidateCase(presetTempToRaw,temp,mask(7,9),10U,1,2,400,2080);
    candidateCase(presetTempToRaw,temp,mask(4,11),10U,1,2,1200,1200);
    candidateCase(presetTempToRaw,temp,mask(4,5),10U,1,6,400,1150);
    /* A clear line cannot authorize a station turn swept into region 2. */
    candidateCase(presetTempToRaw,temp,mask(2,11),10U,1,6,400,1150);
    for (zone = 1U; zone <= 2U; ++zone) {
        const PresetCandidate *startRoutes = zone == 1U ? presetStart1ToQr : presetStart2ToQr;
        const PresetCandidate *qrRoutes = zone == 1U ? presetQr1ToRaw : presetQr2ToRaw;
        PresetPose start = {2250,2250,0};
        start.y = zone == 1U ? 2250 : 150;
        for (variant = 0U; variant < 3U; ++variant) {
            uint16_t regions = variant == 0U ? mask(7,9) : zone == 1U ?
                (variant == 1U ? mask(6,11) : mask(6,5)) :
                (variant == 1U ? mask(12,5) : mask(12,11));
            if (variant && (zone == 2U || variant == 1U))
                candidateCase(startRoutes,start,regions,1U,0,4U,2250,zone == 1U ? 2000 : 300);
            else
                candidateCase(startRoutes,start,regions,1U,0,variant ? 5U : 2U,2100,start.y);
            assert(route.legs[0].taskIndex == 0U && route.legs[1].taskIndex == 1U);
            if (variant) assert(route.legs[2].to.x == (variant == 1U ? 1200 : 300));
        }
        candidateCase(qrRoutes,qr,mask(7,11),3U,1,2,2100,2080);
        assert(route.legs[0].drive.upMm == (zone == 1U ? 860 : 855));
        assert(route.legs[1].drive.rightMm == -870);
        candidateCase(qrRoutes,qr,mask(6,11),3U,1,2,1200,1200);
        candidateCase(qrRoutes,qr,mask(6,5),3U,1,6,2100,1150);
        /* Both detours still cross 9: never silently add a fourth route. */
        memset(&route,0,sizeof(route)); route.regions = mask(6,9);
        assert(!presetFixedConnect(&route,qr,qrRoutes,3U,"RAW_1",1) && !route.count);
        {
            PresetLeg prefix;
            PresetPose gate = {2100,start.y,0};
            RouteLeg exit = routeDistanceLeg(0U,(uint8_t)zone);
            assert(presetAppend(&route,start,gate,0U,&exit)); prefix = route.legs[0];
            assert(!presetFixedConnect(&route,qr,qrRoutes,3U,"RAW_1",1));
            assert(route.count == 1U && !memcmp(&prefix,&route.legs[0],sizeof(prefix)));
        }
        assert(!presetBuild(&route,(uint8_t)zone,zone == 1U ? mask(6,9) : mask(12,9)));
        assert(!route.count && !route.index);
    }
    candidateCase(presetCoarseToTemp,coarse,mask(7,9),8U,2,2,400,400);
    assert(route.legs[0].drive.rightMm == -860 && route.legs[1].drive.upMm == 850);
    assert(route.legs[0].arc == PRESET_ARC_FOUR_POINT && !route.legs[1].arc);
    candidateCase(presetTempToRaw,temp,mask(7,9),10U,1,2,400,2080);
    assert(route.legs[0].drive.upMm == 880 && route.legs[0].arc == PRESET_ARC_FOUR_POINT);
    assert(route.legs[1].drive.rightMm == 850 && !route.legs[1].arc);
}

static unsigned taskBegin(const PresetRoute *value, uint8_t task)
{
    unsigned i;
    for (i = 0U; i < value->count && value->legs[i].taskIndex != task; ++i) { }
    assert(i < value->count); return i;
}
static void sameTask(const PresetRoute *a, uint8_t taskA, const PresetRoute *b, uint8_t taskB)
{
    unsigned i = taskBegin(a,taskA), j = taskBegin(b,taskB);
    while (i < a->count && a->legs[i].taskIndex == taskA) {
        const PresetLeg *x = &a->legs[i], *y = &b->legs[j];
        assert(j < b->count && y->taskIndex == taskB);
        assert(x->from.x == y->from.x && x->from.y == y->from.y && x->from.heading == y->from.heading);
        assert(x->to.x == y->to.x && x->to.y == y->to.y && x->to.heading == y->to.heading);
        assert(x->drive.upMm == y->drive.upMm && x->drive.rightMm == y->drive.rightMm);
        assert(x->drive.heading == y->drive.heading && x->drive.departureHeading == y->drive.departureHeading);
        assert(x->arc == y->arc && !!x->drive.event == !!y->drive.event);
        ++i; ++j;
    }
    assert(j == b->count || b->legs[j].taskIndex != taskB);
}

static void rawLeftFourPointRoute(void)
{
    static const int up[3] = {0,-1242,0}, right[3] = {-581,0,661};
    static const unsigned blocked[] = {1,2,4,10,13,14,15};
    unsigned zone, round, i;
    for (zone = 1U; zone <= 2U; ++zone) {
        assert(presetBuild(&route,(uint8_t)zone,mask(8,9)));
        for (round = 0U; round < 2U; ++round) {
            unsigned first = taskBegin(&route,round ? 13U : 6U);
            assert(route.legs[first].to.x == 400 && route.legs[first].to.y == 2080);
            assert(route.legs[first+1U].to.x == 400 && route.legs[first+1U].to.y == 400);
            assert(route.legs[first].drive.rightMm == routeDistanceLeg(4U,(uint8_t)zone).rightMm);
            assert(route.legs[first+1U].drive.upMm == routeDistanceLeg(5U,(uint8_t)zone).upMm);
            assert(route.legs[first+2U].drive.rightMm == routeDistanceLeg(6U,(uint8_t)zone).rightMm);
            for (i = 0U; i < 3U; ++i) {
                RouteLeg active;
                assert(route.legs[first+i].arc == (i < 2U ? PRESET_ARC_FOUR_POINT : 0U));
                route.index = (uint16_t)(first+i); active = presetActiveLeg(&route);
                assert(active.upMm == up[i] && active.rightMm == right[i]);
            }
            assert(route.legs[first+2U].drive.event && route.legs[first+2U].to.x == 1200);
        }
    }
    for (i = 0U; i < sizeof(blocked)/sizeof(blocked[0]); ++i) {
        memset(&route,0,sizeof(route)); route.regions = mask(blocked[i],9U);
        assert(!presetTryCandidate(&route,(PresetPose){1200,2080,1},&presetRawToCoarse[1],6U,"COARSE",-1));
        assert(!route.count);
    }
}

static void upperArrivalUsesRawBend(void)
{
    unsigned zone, round, i;
    for (zone = 1U; zone <= 2U; ++zone) {
        assert(presetBuild(&route,(uint8_t)zone,mask(7,13)));
        for (round = 0U; round < 2U; ++round) {
            uint8_t task = round ? 15U : 8U;
            unsigned first = taskBegin(&route,task), legs = 0U;
            const PresetLeg *bend = &route.legs[first+1U];
            for (i = first; i < route.count && route.legs[i].taskIndex == task; ++i) {
                assert(route.legs[i].drive.rightMm <= 0);
                ++legs;
            }
            assert(legs == 3U && route.legs[first].arc == 1U);
            assert(bend->arc == PRESET_ARC_FOUR_POINT && bend->drive.rightMm == -860);
            /* Same anchor, headings and arc rule as RAW->COARSE's upper-left bend. */
            assert(bend->to.x == presetRawToCoarse[1].steps[0].x);
            assert(bend->to.y == presetRawToCoarse[1].steps[0].y);
            assert(bend->from.heading == presetRawToCoarse[1].steps[0].heading);
            assert(route.legs[first+2U].from.heading == presetRawToCoarse[1].steps[1].heading);
            assert(presetArcClear(bend,&route.legs[first+2U],route.regions));
            assert(referenceArcClear(bend,&route.legs[first+2U],route.regions));
            route.index = (uint16_t)(first+1U); assert(presetActiveLeg(&route).rightMm == -362);
            route.index = (uint16_t)(first+2U); assert(presetActiveLeg(&route).upMm == -637);
            assert(route.legs[first+2U].to.x == 400 && route.legs[first+2U].to.y == 1200);
            assert(route.legs[first+2U].drive.event && !route.legs[first+2U].drive.rightMm);
        }
    }
    memset(&route,0,sizeof(route)); route.regions = mask(7,9);
    assert(presetTryCandidate(&route,(PresetPose){1200,400,-1},&presetCoarseToTemp[2],8U,"TEMP",2));
    assert(route.count == 3U && route.legs[1].drive.rightMm == -860);
}

static void startSharedDetourDistance(void)
{
    static const unsigned otherRegions[] = {1,4,7,10,11,12,13};
    static const int up[] = {-250,0,-800,0}, right[] = {0,-1000,0,900};
    static const int x[] = {2250,1200,1200,2100}, y[] = {2000,2000,1200,1200};
    unsigned zone, i;
    for (zone = 1U; zone <= 2U; ++zone) {
        assert(presetBuild(&route,(uint8_t)zone,mask(7,9)));
        assert(route.legs[0].taskIndex == 0U && !route.legs[0].arc);
        assert(route.legs[0].drive.upMm == 0);
        assert(route.legs[0].drive.rightMm == (zone == 1U ? -200 : -150));
        assert(route.legs[0].from.x == 2250 && route.legs[0].to.x == 2100);
        assert(route.legs[0].from.y == (zone == 1U ? 2250 : 150));
    }
    for (i = 0U; i < sizeof(otherRegions)/sizeof(otherRegions[0]); ++i) {
        unsigned legIndex;
        assert(presetBuild(&route,1U,mask(6,otherRegions[i])));
        for (legIndex = 0U; legIndex < 4U; ++legIndex) {
            const PresetLeg *leg = &route.legs[legIndex];
            RouteLeg active;
            route.index = (uint16_t)legIndex; active = presetActiveLeg(&route);
            assert(active.upMm == up[legIndex] && active.rightMm == right[legIndex]);
            assert(leg->to.x == x[legIndex] && leg->to.y == y[legIndex]);
            assert(leg->taskIndex == (legIndex ? 1U : 0U) && !leg->arc);
            assert(leg->from.heading == 0 && leg->to.heading == 0);
            assert(!!leg->drive.event == (legIndex == 3U));
        }
        assert(!strcmp(route.legs[3].drive.event,"QR"));
        assert(presetBuild(&route,2U,mask(6,otherRegions[i])));
        if (otherRegions[i] == 12U)
            assert(route.legs[0].drive.upMm == 150 && !route.legs[0].drive.rightMm);
        else
            assert(route.legs[0].drive.rightMm == -150 && !route.legs[0].drive.upMm);
    }
}

static void fixedStartDetours(void)
{
    unsigned i;
    /* Region 6 blocks the direct zone-1 ingress. Region 11 leaves its
     * second start and QR->RAW candidates available. */
    assert(presetBuild(&route,1U,mask(6,11)));
    for (i = 0U; i < route.count && !route.legs[i].drive.event; ++i) { }
    assert(i < route.count && !strcmp(route.legs[i].drive.event,"QR"));
    assert(route.legs[i].from.x == 1200 && route.legs[i].to.x == 2100);
}

static void sharedQrDistance(void)
{
    unsigned zone, a, b, secondCandidates = 0U;
    for (zone = 1U; zone <= 2U; ++zone) for (a = 1U; a <= 15U; ++a)
        for (b = a+1U; b <= 15U; ++b) {
            const PresetCandidate *candidates = zone == 1U ? presetQr1ToRaw : presetQr2ToRaw;
            memset(&route,0,sizeof(route)); route.regions = mask(a,b);
            if (!presetFixedConnect(&route,(PresetPose){2100,1200,0},candidates,3U,"RAW_1",1)) continue;
            if (route.legs[0].to.x != 1200 || route.legs[0].to.y != 1200) continue;
            ++secondCandidates;
            assert(route.count == 2U && route.legs[0].drive.rightMm == -900);
            assert(!route.legs[0].arc && !route.legs[1].arc);
            route.index = 1U;
            assert(presetActiveLeg(&route).upMm == 800);
            assert(route.legs[1].to.x == 1200 && route.legs[1].to.y == 2080);
            assert(route.legs[1].drive.event && route.legs[1].to.heading == 1);
        }
    assert(secondCandidates);
}

static void start2ExitAndHome1(void)
{
    unsigned a, b, i, homeRoutes = 0U, exits = 0U;
    for (a = 1U; a <= 15U; ++a) for (b = a+1U; b <= 15U; ++b) {
        if (presetBuild(&route,2U,mask(a,b)) && (a == 12U || b == 12U)) {
            ++exits;
            assert(route.legs[0].drive.upMm == 150 && !route.legs[0].drive.rightMm);
            assert(route.legs[0].from.x == 2250 && route.legs[0].to.x == 2250);
            assert(route.legs[0].to.y == 300 && !route.legs[0].arc);
            assert(route.legs[1].drive.rightMm < 0 && !route.legs[1].drive.upMm);
        }
        if (!presetBuild(&route,1U,mask(a,b))) continue;
        ++homeRoutes;
        i = taskBegin(&route,19U);
        assert(route.legs[i].from.heading == 2); /* Leave TEMP without a pivot. */
        assert(route.legs[route.count-2U].drive.rightMm > 0);
        assert(route.legs[route.count-2U].to.x == 2250 && route.legs[route.count-2U].to.y == 2080);
        assert(route.legs[route.count-1U].drive.upMm == 170);
        assert(route.legs[route.count-1U].from.x == 2250 && route.legs[route.count-1U].to.x == 2250);
        assert(route.legs[route.count-1U].to.y == 2250 && route.legs[route.count-1U].to.heading == 1);
        assert(!strcmp(route.legs[route.count-1U].drive.event,"HOME"));
        for (; i < route.count; ++i) {
            const PresetLeg *leg = &route.legs[i];
            assert(abs(presetHeadingChange(leg->from.heading,leg->to.heading)) != 2);
            if (i) assert(abs(presetHeadingChange(route.legs[i-1U].to.heading,leg->from.heading)) != 2);
            if (leg->arc == PRESET_ARC_FOUR_POINT) {
                assert(leg->drive.upMm == presetTempToRaw[0].steps[0].upMm);
                assert(leg->to.x == presetTempToRaw[0].steps[0].x);
                assert(leg->to.y == presetTempToRaw[0].steps[0].y);
            }
        }
    }
    assert(exits && homeRoutes);
    /* A shared COARSE->TEMP corridor cannot bypass failures at other stations. */
    memset(&route,0,sizeof(route)); route.regions = mask(7,14);
    assert(!presetTurnClear((PresetPose){1200,400,-1},route.regions));
    assert(presetFixedConnect(&route,(PresetPose){1200,400,0},presetCoarseToTemp,8U,"TEMP",2));
    assert(route.count == 3U && route.legs[1].arc == PRESET_ARC_FOUR_POINT);
    memset(&route,0,sizeof(route)); route.regions = mask(2,7);
    assert(!presetTurnClear((PresetPose){1200,2080,1},route.regions));
    assert(!presetFixedConnect(&route,(PresetPose){400,1200,2},presetTempToRaw,10U,"RAW_2",1));
    for (i = 1U; i <= 2U; ++i) {
        assert(!presetBuild(&route,(uint8_t)i,mask(7,14)) && !route.count);
        assert(!presetBuild(&route,(uint8_t)i,mask(2,7)) && !route.count);
    }
}

int main(void)
{
    unsigned zone, a, b, feasible[2] = {0,0}, rejected[2] = {0,0}, arcs = 0U, detours = 0U;
    static PresetRoute zone1;
    unsigned shared = 0U;
    selection(); geometry(); coarseArcPreserved(); fourPointObstacleSelection(); tempReturnArcPreserved(); remainingArcsPreserved(); fixedCandidatePriorities(); fixedStartDetours(); startSharedDetourDistance(); sharedQrDistance(); rawLeftFourPointRoute(); upperArrivalUsesRawBend(); start2ExitAndHome1(); recenterEnvelope();
    assert(!presetBuild(&route,0,mask(5,12)) && !route.count);
    assert(!presetBuild(&route,1,0U) && !route.count);
    assert(!presetBuild(&route,1,0x8001U) && !route.count);
    for (zone = 1U; zone <= 2U; ++zone) for (a = 1U; a <= 15U; ++a) for (b = a+1U; b <= 15U; ++b) {
        if (presetBuild(&route,(uint8_t)zone,mask(a,b))) {
            unsigned i;
            ++feasible[zone-1U]; checkRoute((uint8_t)zone);
            sameTask(&route,6U,&route,13U); sameTask(&route,8U,&route,15U);
            for (i = 0U; i < route.count; ++i) arcs += !!route.legs[i].arc;
            if (route.count > 20U) ++detours;
        } else { ++rejected[zone-1U]; assert(!route.count && !route.index); }
    }
    assert(feasible[0] && feasible[1] && rejected[0] && rejected[1] && arcs && detours);
    assert(presetBuild(&route,1U,mask(6,5))); /* Long start and QR->RAW detours. */
    assert(presetBuild(&route,2U,mask(12,11))); /* Mirror long start detour. */
    for (a = 1U; a <= 15U; ++a) for (b = a+1U; b <= 15U; ++b) {
        if (!presetBuild(&zone1,1U,mask(a,b)) || !presetBuild(&route,2U,mask(a,b))) continue;
        sameTask(&zone1,6U,&route,6U); sameTask(&zone1,8U,&route,8U);
        sameTask(&zone1,10U,&route,10U); sameTask(&zone1,13U,&route,13U);
        sameTask(&zone1,15U,&route,15U); ++shared;
    }
    assert(shared);
    printf("PASS shared station routes: %u pairs across start zones\n",shared);
    printf("PASS 105 pairs x 2 zones: feasible=%u/%u rejected=%u/%u arcs=%u detours=%u; task order and calibrated approaches\n",
           feasible[0],feasible[1],rejected[0],rejected[1],arcs,detours);
    return 0;
}
