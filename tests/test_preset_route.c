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
    PresetPose a = {1200,1200,0}, b = {1200,2080,0};
    PresetLeg before = {{0}, {2100,1200,0}, {2100,2080,0}, 3, 0, 0};
    PresetLeg after = {{0}, {2100,2080,1}, {1200,2080,1}, 3, 0, 0};
    assert(!presetSegmentClear(a,b,mask(5,12)));
    assert(presetSegmentClear(a,b,mask(2,12)));
    assert(presetArcClear(&before,&after,mask(5,12)));
    assert(!presetArcClear(&before,&after,mask(3,12)));
    before.from = (PresetPose){1200,300,-1}; before.to = (PresetPose){300,300,-1};
    after.from = (PresetPose){300,300,2}; after.to = (PresetPose){300,1100,2};
    assert(presetArcClear(&before,&after,mask(7,9)));
    assert(referenceArcClear(&before,&after,mask(7,9)));
    after.from.heading = after.to.heading = 0;
    assert(!presetArcClear(&before,&after,mask(7,9)));
    assert(!presetBodyClear(473.0f,473.0f,0.785398163f,0U));
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
        assert((active.upMm == 0) != (active.rightMm == 0));
        if (leg->drive.event) {
            RouteLeg calibrated;
            PresetPose station;
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
            assert(!strcmp(leg->drive.event,calibrated.event));
            nominal = leg->drive.upMm ? leg->to.y-leg->from.y : leg->to.x-leg->from.x;
            commanded = leg->drive.upMm ? leg->drive.upMm : leg->drive.rightMm;
            assert(nominal*commanded > 0);
            ++event;
        }
        if (leg->arc) {
            if (leg->arc == PRESET_ARC_FOUR_POINT)
                assert(((leg->taskIndex == 8U || leg->taskIndex == 15U) &&
                        leg->drive.rightMm == (leg->to.y == 2080 ? -910 : -860)) ||
                       ((leg->taskIndex == 10U || leg->taskIndex == 19U) && leg->drive.upMm == 880) ||
                       ((leg->taskIndex == 10U || leg->taskIndex == 19U) &&
                        (leg->drive.upMm == -850 || leg->drive.rightMm == 860)) ||
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

static unsigned templateId(const char *code)
{
    unsigned i;
    for (i = 1U; i <= PRESET_TEMPLATE_COUNT; ++i)
        if (presetTemplates[i].code && !strcmp(presetTemplates[i].code,code)) return i;
    assert(!"unknown template code"); return 0U;
}

static void fixedDistances(void)
{
    struct Case {
        unsigned a, b;
        uint8_t task, count;
        const char *code;
        int16_t up[3], right[3];
    } cases[] = {
        {7,9,6,1,"RC-1",{-1680},{0}},
        {7,14,6,1,"RC-1",{-1680},{0}},
        {8,9,6,3,"RC-2",{0,-1242,0},{-581,0,661}},
        {4,8,6,3,"RC-3",{0,-1431,0},{651,0,-900}},
        {8,14,6,3,"RC-3",{0,-1431,0},{651,0,-900}},
        {7,9,8,2,"CT-1",{0,601},{-611,0}},
        {7,13,8,3,"CT-3",{1431,0,-637},{0,-412,0}},
        {7,9,10,2,"TR-1",{631,0},{0,601}},
        {1,5,10,3,"TR-3",{0,382,0},{1451,0,-651}},
        {1,7,10,3,"TR-4",{-601,0,1431},{0,362,0}},
        {4,7,10,3,"TR-4",{-601,0,1431},{0,362,0}}
    };
    unsigned zone, c, i;
    static PresetRoute original;
    for (zone = 1U; zone <= 2U; ++zone) for (c = 0U; c < sizeof(cases)/sizeof(cases[0]); ++c) {
        const struct Case *test = &cases[c];
        unsigned first;
        assert(presetBuild(&route,(uint8_t)zone,mask(test->a,test->b)));
        first = taskBegin(&route,test->task);
        for (i = 0U; i < test->count; ++i) {
            RouteLeg active;
            route.index = (uint16_t)(first+i); active = presetActiveLeg(&route);
            assert(route.legs[first+i].templateId == templateId(test->code));
            assert(active.upMm == test->up[i] && active.rightMm == test->right[i]);
            assert(!!active.event == (i+1U == test->count));
        }
    }
    /* The whole shared departure, not just its initial distance, is identical. */
    assert(presetBuild(&original,1U,mask(6,7)));
    assert(presetBuild(&route,1U,mask(6,12)));
    sameTask(&original,0U,&route,0U); sameTask(&original,1U,&route,1U);
    assert(route.legs[0].drive.upMm == -250);
    assert(route.legs[1].drive.rightMm == -1000);
    assert(route.legs[2].drive.upMm == -800);
    assert(route.legs[3].drive.rightMm == 900);
    for (zone = 1U; zone <= 2U; ++zone) {
        unsigned first;
        assert(presetBuild(&route,(uint8_t)zone,mask(6,7)));
        first = taskBegin(&route,3U);
        assert(route.legs[first].drive.rightMm == -900);
        assert(route.legs[first+1U].drive.upMm == 800);
    }
    /* Reused upper-left bend has the same anchor/headings/scope. */
    {
        const PresetStep *raw = presetTemplates[templateId("RC-2")].steps;
        const PresetStep *coarse = presetTemplates[templateId("CT-3")].steps;
        assert(raw[0].x == coarse[1].x && raw[0].y == coarse[1].y);
        assert(raw[0].departureHeading == coarse[1].departureHeading);
        assert(raw[0].commandHeading == coarse[1].commandHeading);
        assert(raw[0].arc == coarse[1].arc && raw[0].arc == PRESET_ARC_FOUR_POINT);
    }
}

static void sameMotion(const PresetLeg *x, const PresetLeg *y)
{
    assert(x->templateId == y->templateId);
    assert(x->from.x == y->from.x && x->from.y == y->from.y && x->from.heading == y->from.heading);
    assert(x->to.x == y->to.x && x->to.y == y->to.y && x->to.heading == y->to.heading);
    assert(x->drive.upMm == y->drive.upMm && x->drive.rightMm == y->drive.rightMm);
    assert(x->drive.departureHeading == y->drive.departureHeading && x->drive.heading == y->drive.heading);
    assert(x->arc == y->arc && !!x->drive.event == !!y->drive.event);
}

/* Compare complete realized segments for each ID across all obstacle pairs,
 * zones and rounds. This catches any new hidden per-mask adjustment. */
static void frozenTemplates(void)
{
    static PresetLeg observed[PRESET_TEMPLATE_COUNT+1U][8];
    unsigned lengths[PRESET_TEMPLATE_COUNT+1U] = {0};
    unsigned uses[PRESET_TEMPLATE_COUNT+1U] = {0};
    unsigned zone, a, b, id, group, templates = 0U;
    for (zone = 1U; zone <= 2U; ++zone) for (a = 1U; a <= 15U; ++a) for (b = a+1U; b <= 15U; ++b) {
        unsigned first = 0U;
        if (!presetBuild(&route,(uint8_t)zone,mask(a,b))) continue;
        while (first < route.count) {
            unsigned last = first, i;
            id = route.legs[first].templateId;
            assert(id && id <= PRESET_TEMPLATE_COUNT);
            while (!route.legs[last].drive.event) { ++last; assert(last < route.count); }
            assert(last-first+1U <= 8U);
            if (!uses[id]) {
                lengths[id] = last-first+1U;
                memcpy(observed[id],&route.legs[first],lengths[id]*sizeof(PresetLeg));
            }
            assert(lengths[id] == last-first+1U);
            for (i = 0U; i < lengths[id]; ++i) {
                const PresetLeg *x = &observed[id][i], *y = &route.legs[first+i];
                sameMotion(x,y);
            }
            ++uses[id]; first = last+1U;
        }
    }
    /* Pruning QR alternatives can make another valid segment unavailable in a
     * full mission. Check each retained candidate independently as well. */
    for (group = 0U; group < PRESET_GROUP_COUNT; ++group)
        for (a = 1U; a <= 15U; ++a) for (b = a+1U; b <= 15U; ++b) {
            unsigned candidate;
            for (candidate = 0U; candidate < presetGroups[group].count; ++candidate) {
                unsigned i;
                id = presetGroups[group].templates[candidate];
                memset(&route,0,sizeof(route)); route.regions = mask(a,b);
                if (!presetTryTemplate(&route,&presetGroups[group],(uint16_t)id,3U,"TEST")) continue;
                if (!uses[id]) {
                    lengths[id] = route.count;
                    memcpy(observed[id],route.legs,route.count*sizeof(PresetLeg));
                }
                assert(lengths[id] == route.count);
                for (i = 0U; i < route.count; ++i) sameMotion(&observed[id][i],&route.legs[i]);
                ++uses[id];
            }
        }
    for (id = 1U; id <= PRESET_TEMPLATE_COUNT; ++id) {
        unsigned other;
        if (!presetTemplates[id].count) { assert(!presetTemplates[id].code); continue; }
        ++templates;
        assert(uses[id]);
        for (other = id+1U; other <= PRESET_TEMPLATE_COUNT; ++other)
            if (presetTemplates[other].count) assert(strcmp(presetTemplates[id].code,presetTemplates[other].code));
    }
    printf("PASS %u fixed templates: identical full motions across all selections\n",templates);
}

static void wholeTemplatePriority(void)
{
    static PresetRoute prefix, expected;
    const char *event = "TEST";
    unsigned group, a, b;
    for (group = 0U; group < PRESET_GROUP_COUNT; ++group) for (a = 1U; a <= 15U; ++a) for (b = a+1U; b <= 15U; ++b) {
        unsigned i, chosen = 0U;
        memset(&prefix,0,sizeof(prefix)); prefix.regions = mask(a,b);
        prefix.count = 1U; prefix.legs[0].drive.event = "PREFIX";
        prefix.legs[0].drive.upMm = 123;
        for (i = 0U; i < presetGroups[group].count; ++i) {
            expected = prefix;
            if (presetTryTemplate(&expected,&presetGroups[group],presetGroups[group].templates[i],3U,event)) {
                chosen = presetGroups[group].templates[i]; break;
            }
            assert(expected.count == prefix.count && !memcmp(&expected.legs[0],&prefix.legs[0],sizeof(PresetLeg)));
        }
        route = prefix;
        assert(!!presetGroupConnect(&route,group,3U,event) == !!chosen);
        assert(!memcmp(&route.legs[0],&prefix.legs[0],sizeof(PresetLeg)));
        if (chosen) {
            assert(route.legs[1].templateId == chosen);
            assert(route.count == expected.count);
            assert(!memcmp(route.legs,expected.legs,route.count*sizeof(PresetLeg)));
        } else assert(route.count == 1U);
    }
    /* A blocked corridor rejects its whole template without editing its arcs. */
    memset(&route,0,sizeof(route)); route.regions = mask(5,7);
    assert(!presetTryTemplate(&route,&presetGroups[PRESET_GROUP_TR],(uint16_t)templateId("TR-3"),10U,"RAW_2"));
    assert(!route.count);
    assert(presetGroupConnect(&route,PRESET_GROUP_TR,10U,"RAW_2"));
    assert(route.legs[0].templateId != templateId("TR-3"));
}

static void homeAndStartExits(void)
{
    unsigned a, b, i;
    for (a = 1U; a <= 15U; ++a) for (b = a+1U; b <= 15U; ++b) {
        if (presetBuild(&route,2U,mask(a,b)) && (a == 12U || b == 12U)) {
            assert(route.legs[0].drive.upMm == 150 && !route.legs[0].drive.rightMm);
            assert(route.legs[0].from.x == 2250 && route.legs[0].to.x == 2250);
            assert(route.legs[1].drive.rightMm < 0 && !route.legs[1].drive.upMm);
        }
        if (!presetBuild(&route,1U,mask(a,b))) continue;
        i = taskBegin(&route,19U);
        assert(abs(presetHeadingChange(2,route.legs[i].from.heading)) != 2);
        for (; i < route.count; ++i) {
            const PresetLeg *leg = &route.legs[i];
            assert(abs(presetHeadingChange(leg->from.heading,leg->to.heading)) != 2);
            if (i) assert(abs(presetHeadingChange(route.legs[i-1U].to.heading,leg->from.heading)) != 2);
        }
        assert(route.legs[route.count-2U].drive.rightMm > 0);
        assert(route.legs[route.count-2U].to.x == 2250 && route.legs[route.count-2U].to.y == 2080);
        assert(route.legs[route.count-1U].drive.upMm == 170);
        assert(route.legs[route.count-1U].from.x == 2250 && route.legs[route.count-1U].to.x == 2250);
        assert(route.legs[route.count-1U].to.y == 2250 && route.legs[route.count-1U].to.heading == 1);
    }
}

static void rawRightDirectArrival(void)
{
    static PresetRoute reference;
    static const unsigned regions[][2] = {{4,8},{8,14}};
    static const int up[] = {0,-1431,0}, right[] = {651,0,-900};
    unsigned zone, map, round, i;
    for (zone = 1U; zone <= 2U; ++zone) for (map = 0U; map < 2U; ++map) {
        assert(presetBuild(&route,(uint8_t)zone,mask(regions[map][0],regions[map][1])));
        for (round = 0U; round < 2U; ++round) {
            uint8_t task = round ? 13U : 6U;
            unsigned first = taskBegin(&route,task);
            for (i = 0U; i < 3U; ++i) {
                RouteLeg active;
                const PresetLeg *leg = &route.legs[first+i];
                assert(leg->taskIndex == task && leg->drive.upMm <= 0);
                assert(leg->arc == (i == 0U));
                route.index = (uint16_t)(first+i); active = presetActiveLeg(&route);
                assert(active.upMm == up[i] && active.rightMm == right[i]);
            }
            assert(route.legs[first+1U].to.x == 2100 && route.legs[first+1U].to.y == 400);
            assert(route.legs[first+1U].drive.upMm == -1680);
            assert(route.legs[first+2U].from.y == 400 && route.legs[first+2U].to.y == 400);
            assert(route.legs[first+2U].to.x == 1200 && route.legs[first+2U].drive.event);
            assert(route.legs[first+2U].from.heading == -1 && route.legs[first+2U].to.heading == -1);
            if (map) sameTask(&reference,task,&route,task);
        }
        if (!map) reference = route;
    }
}

static void start1LoopDownBeforeLeft(void)
{
    static const unsigned pairs[][2] = {{5,6},{6,8},{6,9}};
    unsigned map, i;
    for (map = 0U; map < sizeof(pairs)/sizeof(pairs[0]); ++map) {
        unsigned qr, arcs = 0U;
        memset(&route,0,sizeof(route)); route.regions = mask(pairs[map][0],pairs[map][1]);
        assert(presetGroupConnect(&route,PRESET_GROUP_S1,1U,"QR"));
        assert(route.legs[0].taskIndex == 0U && !route.legs[0].arc);
        assert(route.legs[0].from.x == 2250 && route.legs[0].from.y == 2250);
        assert(route.legs[0].to.x == 2250 && route.legs[0].to.y == 2100);
        assert(route.legs[0].drive.upMm == -150 && !route.legs[0].drive.rightMm);
        assert(route.legs[1].taskIndex == 1U && !route.legs[1].arc);
        assert(route.legs[1].from.y == 2100 && route.legs[1].to.y == 2100);
        assert(route.legs[1].drive.rightMm < 0 && !route.legs[1].drive.upMm);
        if (!map) {
            assert(route.legs[1].drive.rightMm == -2000);
            assert(route.legs[1].to.x == 300);
        } else {
            assert(route.legs[1].drive.rightMm == -150 && route.legs[1].to.x == 2100);
        }
        for (qr = 0U; qr < route.count && !route.legs[qr].drive.event; ++qr) { }
        assert(qr < route.count && !strcmp(route.legs[qr].drive.event,"QR"));
        for (i = 0U; i <= qr; ++i) arcs += !!route.legs[i].arc;
        if (map == 1U) assert(arcs == 3U); /* Keep the pictured outer-loop bends. */
    }
}

static void start1CentreLowerDetour(void)
{
    static const int16_t up[] = {-150,0,0,-1720,0,594};
    static const int16_t right[] = {0,-150,-900,0,651,0};
    static const int16_t x[] = {2250,2100,1200,1200,2100,2100};
    static const int16_t y[] = {2100,2100,2100,380,380,1200};
    unsigned i, first;
    memset(&route,0,sizeof(route)); route.regions = mask(6,9);
    assert(presetGroupConnect(&route,PRESET_GROUP_S1,1U,"QR"));
    for (i = 0U; i < 6U; ++i) {
        RouteLeg active;
        const PresetLeg *leg = &route.legs[i];
        assert(leg->templateId == templateId("S1-B2"));
        assert(leg->to.x == x[i] && leg->to.y == y[i]);
        assert(leg->arc == (i == 4U));
        assert(leg->taskIndex == (i ? 1U : 0U));
        route.index = (uint16_t)i; active = presetActiveLeg(&route);
        assert(active.upMm == up[i] && active.rightMm == right[i]);
        assert(!!active.event == (i == 5U));
        if (leg->arc) assert(referenceArcClear(leg,&route.legs[i+1U],route.regions));
    }
    assert(!strcmp(route.legs[5].drive.event,"QR"));
    assert(route.legs[3].from.x == 1200 && route.legs[3].drive.upMm == -1720);
    assert(route.legs[5].drive.upMm == 843);
    /* Upper and centre-bottom are parking pivots; only the right-bottom blends. */
    assert(route.legs[2].to.heading == 1 && route.legs[3].from.heading == 2);
    assert(route.legs[3].to.heading == 2 && route.legs[4].from.heading == -1);
    /* Region 8 blocks this entire fixed template: retain the outside loop. */
    memset(&route,0,sizeof(route)); route.regions = mask(6,8);
    assert(!presetTryTemplate(&route,&presetGroups[PRESET_GROUP_S1],
                              (uint16_t)templateId("S1-B2"),1U,"QR"));
    assert(!route.count);
    assert(presetGroupConnect(&route,PRESET_GROUP_S1,1U,"QR"));
    first = taskBegin(&route,1U);
    assert(route.legs[first].templateId == templateId("S1-B1"));
    assert(presetOrderS1[2] == templateId("S1-B2"));
}

static void rawThreeRoutes(void)
{
    static const unsigned pairs[][2] = {{5,15},{8,15},{11,15}};
    const PresetGroup *group = &presetGroups[PRESET_GROUP_RC];
    unsigned zone, pair, round;
    assert(group->count == 3U);
    assert(group->templates[0] == templateId("RC-1") &&
           group->templates[1] == templateId("RC-2") &&
           group->templates[2] == templateId("RC-3"));
    for (zone = 1U; zone <= 2U; ++zone)
        for (pair = 0U; pair < sizeof(pairs)/sizeof(pairs[0]); ++pair) {
            assert(presetBuild(&route,(uint8_t)zone,mask(pairs[pair][0],pairs[pair][1])));
            for (round = 0U; round < 2U; ++round) {
                unsigned first = taskBegin(&route,round ? 13U : 6U);
                const PresetLeg *last = &route.legs[first+2U];
                PresetPose projected = last->from;
                assert(route.legs[first].templateId == templateId("RC-2"));
                assert(route.legs[first].drive.rightMm == -830);
                assert(route.legs[first+1U].drive.upMm == -1740);
                assert(last->drive.rightMm == 910 && last->drive.event);
                assert(last->to.x == 1200 && last->to.y == 400);
                assert(presetSegmentClear(last->from,last->to,route.regions));
                /* The 910mm command is tuned for this corridor. Its naive
                 * x=1310 projection must not replace the x=1200 map endpoint. */
                projected.x += last->drive.rightMm;
                assert(projected.x == 1310 && !presetSegmentClear(last->from,projected,route.regions));
                route.index = (uint16_t)(first+2U);
                assert(presetActiveLeg(&route).rightMm == 661);
            }
        }
}

static void noPivotClearance(void)
{
    static const uint8_t pairs[][2] = {
        {1,3},{2,3},{2,6},{3,4},{3,7},{3,8},{3,9},{3,10},{6,8},{6,9}
    };
    unsigned zone, i, first;
    for (zone = 1U; zone <= 2U; ++zone)
        for (i = 0U; i < sizeof(pairs)/sizeof(pairs[0]); ++i)
            assert(presetBuild(&route,(uint8_t)zone,mask(pairs[i][0],pairs[i][1])));
    /* Keep the original centre arrival and working heading, without moving
     * its station pivot or adding a clearance offset. */
    assert(presetBuild(&route,1U,mask(7,14)));
    first = taskBegin(&route,6U);
    assert(route.legs[first].templateId == templateId("RC-1"));
    assert(route.legs[first].drive.upMm == -1680 && !route.legs[first].drive.rightMm);
    assert(route.legs[first].from.heading == 2 && route.legs[first].to.heading == -1);
    assert(route.legs[first].to.x == 1200 && route.legs[first].to.y == 400);
    /* At QR, region 6 used to reject the departure pivot of the outside
     * route. Its first translation remains the unmodified -900mm step. */
    assert(presetBuild(&route,1U,mask(6,8)));
    first = taskBegin(&route,3U);
    assert(route.legs[first].templateId == templateId("Q-B3"));
    assert(route.legs[first].from.x == 2100 && route.legs[first].from.y == 1200);
    assert(route.legs[first].drive.upMm == -900 && route.legs[first].from.heading == 2);
    /* Region 4 used to reject TEMP's parking turn. Keep the direct right
     * route, with no 50mm clearance moves added around the centre lane. */
    assert(presetBuild(&route,1U,mask(4,5)));
    first = taskBegin(&route,10U);
    assert(route.legs[first].templateId == templateId("TR-3"));
    assert(route.legs[first].drive.rightMm == 1700 && !route.legs[first].drive.upMm);
    assert(route.legs[first].from.y == 1200 && route.legs[first].to.y == 1200);
    assert(route.legs[first].arc == 1U && route.legs[first+1U].arc == 1U);
}

static void qrSimplification(void)
{
    unsigned group, i;
    assert(!presetTemplates[12].count && !presetTemplates[13].count && !presetTemplates[15].count);
    for (group = PRESET_GROUP_Q1; group <= PRESET_GROUP_Q2; ++group) {
        const PresetGroup *g = &presetGroups[group];
        assert(g->count == 4U && g->templates[1] == templateId("Q-2") &&
               g->templates[2] == templateId("Q-3") && g->templates[3] == templateId("Q-B3"));
        memset(&route,0,sizeof(route)); route.regions = mask(5,8);
        assert(presetTryTemplate(&route,g,(uint16_t)templateId("Q-B3"),3U,"RAW_1"));
        assert(route.count == 4U);
        for (i = 0U; i < 3U; ++i) {
            assert(route.legs[i].arc == 1U);
            assert(referenceArcClear(&route.legs[i],&route.legs[i+1U],route.regions));
        }
        assert(route.legs[3].from.x == 300 && route.legs[3].to.x == 1200);
        assert(route.legs[3].drive.rightMm == 897 && !route.legs[3].drive.upMm);
        assert(route.legs[3].drive.departureHeading == -1 && route.legs[3].drive.heading == 1);
        assert(!route.legs[3].arc && !strcmp(route.legs[3].drive.event,"RAW_1"));
        route.index = 3U; assert(presetActiveLeg(&route).rightMm == 648);
        memset(&route,0,sizeof(route)); route.regions = mask(3,8);
        assert(presetGroupConnect(&route,group,3U,"RAW_1"));
        assert(route.count == 4U && route.legs[0].templateId == templateId("Q-B3"));
        assert(route.legs[3].drive.rightMm == 897 && route.legs[3].drive.heading == 1);
    }
}

int main(void)
{
    unsigned zone, a, b, feasible[2] = {0,0}, rejected[2] = {0,0}, arcs = 0U, detours = 0U;
    static PresetRoute zone1;
    unsigned shared = 0U;
    selection(); geometry(); coarseArcPreserved(); fixedDistances(); rawThreeRoutes(); noPivotClearance(); qrSimplification(); frozenTemplates();
    wholeTemplatePriority(); homeAndStartExits(); rawRightDirectArrival(); start1LoopDownBeforeLeft(); start1CentreLowerDetour(); recenterEnvelope();
    assert(!presetBuild(&route,0,mask(5,12)) && !route.count);
    assert(!presetBuild(&route,1,0U) && !route.count);
    assert(!presetBuild(&route,1,0x8001U) && !route.count);
    for (zone = 1U; zone <= 2U; ++zone) for (a = 1U; a <= 15U; ++a) for (b = a+1U; b <= 15U; ++b) {
        if (presetBuild(&route,(uint8_t)zone,mask(a,b))) {
            unsigned i;
            ++feasible[zone-1U]; checkRoute((uint8_t)zone);
            sameTask(&route,6U,&route,13U); sameTask(&route,8U,&route,15U);
            for (i = 0U; i < route.count; ++i) {
                const PresetLeg *leg = &route.legs[i];
                if (leg->taskIndex == 6U || leg->taskIndex == 13U)
                    assert(leg->templateId == templateId("RC-1") ||
                           leg->templateId == templateId("RC-2") ||
                           leg->templateId == templateId("RC-3"));
                arcs += !!leg->arc;
            }
            if (route.count > 20U) ++detours;
        } else {
            ++rejected[zone-1U]; assert(!route.count && !route.index);
            assert((zone == 1U && a == 3U && b == 6U) ||
                   (zone == 2U && a == 12U && b == 15U));
        }
    }
    assert(feasible[0] == 104U && feasible[1] == 104U);
    assert(rejected[0] == 1U && rejected[1] == 1U && arcs && detours);
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
