#ifndef SHENGSAI_PRESET_ROUTE_H
#define SHENGSAI_PRESET_ROUTE_H
#include <stdlib.h>
#include <string.h>
#include "../shared/obstacle_regions.h"
#include "route_plan.h"
#include "arc_route_blend.h"

#define PRESET_NODE_COUNT 45U
#define PRESET_STATE_COUNT (PRESET_NODE_COUNT * 4U)
#define PRESET_LEG_CAPACITY 128U
#define PRESET_ARC_TRIM 249
typedef struct { int16_t x, y; int8_t heading; } PresetPose;
typedef struct {
    RouteLeg drive;
    PresetPose from, to;
    uint8_t taskIndex, arc;
} PresetLeg;
typedef struct {
    PresetLeg legs[PRESET_LEG_CAPACITY];
    uint16_t count, index, regions;
} PresetRoute;
typedef struct { uint32_t lateral, reverse, distance; } PresetCost;
static const int16_t presetGridX[5] = {300,400,1200,2100,2250};
static const int16_t presetGridY[9] = {150,300,400,1150,1200,1250,2080,2100,2250};
static const SsObstacleRegion presetFixed[] = {
    {550,550,1000,1000}, {1400,550,1850,1000},
    {550,1400,1000,1850}, {1400,1400,1850,1850},
    {0,910,150,1490}, {910,0,1490,150}
};
static int8_t presetHeading(unsigned heading) { return heading == 3U ? -1 : (int8_t)heading; }
static unsigned presetHeadingIndex(int8_t heading) { return heading == -1 ? 3U : (unsigned)heading; }
static PresetPose presetNode(unsigned node, unsigned heading)
{
    PresetPose p;
    p.x = presetGridX[node % 5U]; p.y = presetGridY[node / 5U];
    p.heading = presetHeading(heading); return p;
}
static int presetNodeAt(PresetPose pose)
{
    unsigned i;
    for (i = 0; i < PRESET_NODE_COUNT; ++i) {
        PresetPose p = presetNode(i, 0U);
        if (p.x == pose.x && p.y == pose.y) return (int)i;
    }
    return -1;
}
static uint8_t presetOverlap(float x0, float y0, float x1, float y1, const SsObstacleRegion *r)
{ return (uint8_t)(x0 < r->x1 && x1 > r->x0 && y0 < r->y1 && y1 > r->y0); }
static uint8_t presetSegmentClear(PresetPose from, PresetPose to, uint16_t regions)
{
    int x0 = from.x < to.x ? from.x : to.x, x1 = from.x > to.x ? from.x : to.x;
    int y0 = from.y < to.y ? from.y : to.y, y1 = from.y > to.y ? from.y : to.y;
    unsigned i;
    if ((from.x != to.x && from.y != to.y) || x0 < 150 || x1 > 2250 || y0 < 150 || y1 > 2250) return 0U;
    for (i = 0; i < sizeof(presetFixed)/sizeof(presetFixed[0]); ++i)
        if (presetOverlap(x0-150,y0-150,x1+150,y1+150,&presetFixed[i])) return 0U;
    for (i = 0; i < SS_OBSTACLE_REGION_COUNT; ++i)
        if ((regions & (1U << i)) && presetOverlap(x0-150,y0-150,x1+150,y1+150,&ssObstacleRegions[i])) return 0U;
    return 1U;
}
static uint8_t presetCircleHits(PresetPose p, const SsObstacleRegion *r)
{
    int32_t dx = p.x < r->x0 ? r->x0-p.x : p.x > r->x1 ? p.x-r->x1 : 0;
    int32_t dy = p.y < r->y0 ? r->y0-p.y : p.y > r->y1 ? p.y-r->y1 : 0;
    return (uint8_t)(dx*dx + dy*dy < 45000L);
}
static uint8_t presetTurnClear(PresetPose p, uint16_t regions)
{
    unsigned i;
    if (p.x < 213 || p.x > 2187 || p.y < 213 || p.y > 2187) return 0U;
    for (i = 0; i < sizeof(presetFixed)/sizeof(presetFixed[0]); ++i)
        if (presetCircleHits(p, &presetFixed[i])) return 0U;
    for (i = 0; i < SS_OBSTACLE_REGION_COUNT; ++i)
        if ((regions & (1U << i)) && presetCircleHits(p, &ssObstacleRegions[i])) return 0U;
    return 1U;
}
/* Separating-axis test for the complete rotated square, rather than its centre. */
static uint8_t presetBodyHits(float x, float y, float s, float c, float half, const SsObstacleRegion *r)
{
    float dx = (r->x0+r->x1)*0.5f-x, dy = (r->y0+r->y1)*0.5f-y;
    float hx = (r->x1-r->x0)*0.5f, hy = (r->y1-r->y0)*0.5f;
    float as = fabsf(s), ac = fabsf(c);
    if (fabsf(dx) >= hx+half*(as+ac) || fabsf(dy) >= hy+half*(as+ac)) return 0U;
    if (fabsf(-dx*s+dy*c) >= half+hx*as+hy*ac ||
        fabsf(dx*c+dy*s) >= half+hx*ac+hy*as) return 0U;
    return 1U;
}
static uint8_t presetBodyClear(float x, float y, float angle, uint16_t regions)
{
    float s = sinf(angle), c = cosf(angle), extent = 152.0f*(fabsf(s)+fabsf(c));
    unsigned i;
    /* 1mm arc samples, 2mm dilation bounds travel and corner rotation between samples. */
    if (x-extent < 0.0f || x+extent > 2400.0f || y-extent < 0.0f || y+extent > 2400.0f) return 0U;
    for (i = 0; i < sizeof(presetFixed)/sizeof(presetFixed[0]); ++i)
        if (presetBodyHits(x,y,s,c,152.0f,&presetFixed[i])) return 0U;
    for (i = 0; i < SS_OBSTACLE_REGION_COUNT; ++i)
        if ((regions & (1U << i)) && presetBodyHits(x,y,s,c,152.0f,&ssObstacleRegions[i])) return 0U;
    return 1U;
}
static int8_t presetDirection(PresetPose from, PresetPose to)
{ return to.x > from.x ? -1 : to.x < from.x ? 1 : to.y > from.y ? 0 : 2; }
static int8_t presetHeadingChange(int8_t from, int8_t to)
{
    int8_t change = to-from;
    if (change > 2) change -= 4;
    if (change < -2) change += 4;
    return change;
}
static uint8_t presetArcClear(const PresetLeg *before, const PresetLeg *after, uint16_t regions)
{
    float angle = before->from.heading*1.570796327f;
    float x = before->to.x + sinf(angle)*PRESET_ARC_TRIM;
    float y = before->to.y - cosf(angle)*PRESET_ARC_TRIM;
    float distance = 0.0f, degrees, curvature, step;
    int8_t change = presetHeadingChange(before->from.heading, after->from.heading);
    if (change != 1 && change != -1) return 0U;
    while (distance < ARC_BLEND_LENGTH_MM) {
        if (!presetBodyClear(x,y,angle,regions)) return 0U;
        step = ARC_BLEND_LENGTH_MM-distance < 1.0f ? ARC_BLEND_LENGTH_MM-distance : 1.0f;
        arcBlendShape(distance+step*0.5f,&degrees,&curvature);
        angle = before->from.heading*1.570796327f + change*degrees*0.01745329252f;
        x -= sinf(angle)*step; y += cosf(angle)*step;
        distance += step;
    }
    angle = after->from.heading*1.570796327f;
    /* Include the controller's existing extra 30mm exit allowance. */
    for (distance = 0; distance <= 30.0f; distance += 1.0f)
        if (!presetBodyClear(x-sinf(angle)*distance,y+cosf(angle)*distance,angle,regions)) return 0U;
    return 1U;
}
static uint8_t presetCostLess(PresetCost a, PresetCost b)
{
    if (a.lateral != b.lateral) return a.lateral < b.lateral;
    if (a.reverse != b.reverse) return a.reverse < b.reverse;
    return a.distance < b.distance;
}
static uint8_t presetAppend(PresetRoute *route, PresetPose from, PresetPose to, uint8_t taskIndex,
                             const RouteLeg *calibrated)
{
    PresetLeg *leg;
    if (from.x == to.x && from.y == to.y) return 1U;
    if (!presetSegmentClear(from,to,route->regions) || route->count >= PRESET_LEG_CAPACITY) return 0U;
    if (!calibrated && route->count) {
        leg = &route->legs[route->count-1U];
        if (!leg->drive.event && leg->taskIndex == taskIndex && leg->from.heading == from.heading &&
            leg->to.x == from.x && leg->to.y == from.y &&
            presetDirection(leg->from,leg->to) == presetDirection(from,to)) {
            leg->drive.upMm += to.y-from.y; leg->drive.rightMm += to.x-from.x;
            leg->to = to; return 1U;
        }
    }
    leg = &route->legs[route->count++];
    memset(leg,0,sizeof(*leg)); leg->from = from; leg->to = to; leg->taskIndex = taskIndex;
    if (calibrated) leg->drive = *calibrated;
    else { leg->drive.upMm = to.y-from.y; leg->drive.rightMm = to.x-from.x; }
    leg->drive.departureHeading = from.heading; leg->drive.heading = to.heading;
    return 1U;
}
static uint8_t presetConnect(PresetRoute *route, PresetPose start, PresetPose target, uint8_t taskIndex)
{
    static PresetCost costs[PRESET_STATE_COUNT];
    static int16_t previous[PRESET_STATE_COUNT];
    static uint8_t visited[PRESET_STATE_COUNT];
    static uint16_t reverse[PRESET_STATE_COUNT];
    int startNode = presetNodeAt(start), targetNode = presetNodeAt(target), finish = -1;
    unsigned i, count = 0U;
    const PresetCost infinity = {0xFFFFFFFFUL,0xFFFFFFFFUL,0xFFFFFFFFUL};
    if (startNode < 0 || targetNode < 0 || !presetSegmentClear(start,start,route->regions) ||
        !presetSegmentClear(target,target,route->regions)) return 0U;
    for (i = 0; i < PRESET_STATE_COUNT; ++i) { costs[i] = infinity; previous[i] = -1; visited[i] = 0U; }
    costs[startNode*4+presetHeadingIndex(start.heading)].lateral = 0U;
    costs[startNode*4+presetHeadingIndex(start.heading)].reverse = 0U;
    costs[startNode*4+presetHeadingIndex(start.heading)].distance = 0U;
    for (;;) {
        int best = -1;
        PresetCost cost = infinity;
        unsigned node, heading, neighbor;
        PresetPose p;
        for (i = 0; i < PRESET_STATE_COUNT; ++i)
            if (!visited[i] && presetCostLess(costs[i],cost)) { best = (int)i; cost = costs[i]; }
        if (best < 0) break;
        node = (unsigned)best/4U; heading = (unsigned)best%4U; p = presetNode(node,heading);
        if ((int)node == targetNode && p.heading == target.heading) { finish = best; break; }
        visited[best] = 1U;
        if (presetTurnClear(p,route->regions)) for (neighbor = 0; neighbor < 4U; ++neighbor) {
            unsigned state = node*4U+neighbor;
            PresetCost next = cost;
            if (neighbor == heading) continue;
            next.distance += neighbor == (heading+2U)%4U ? 240U : 120U;
            if (presetCostLess(next,costs[state])) { costs[state] = next; previous[state] = (int16_t)best; }
        }
        for (neighbor = 0; neighbor < PRESET_NODE_COUNT; ++neighbor) {
            unsigned state = neighbor*4U+heading, distance;
            PresetPose q = presetNode(neighbor,heading);
            PresetCost next = cost;
            int8_t direction;
            if (!((neighbor/5U == node/5U && (neighbor+1U == node || node+1U == neighbor)) ||
                  (neighbor%5U == node%5U && (neighbor+5U == node || node+5U == neighbor)))) continue;
            /* The boundary anchors serve only the start-zone entrances. */
            if ((p.x == 2250 || q.x == 2250) && !(p.y == q.y && (p.y == 150 || p.y == 2250))) continue;
            if ((p.y == 150 || p.y == 2250) && p.x < 2100) continue;
            if ((q.y == 150 || q.y == 2250) && q.x < 2100) continue;
            if (!presetSegmentClear(p,q,route->regions)) continue;
            distance = (unsigned)(abs(q.x-p.x)+abs(q.y-p.y)); direction = presetDirection(p,q);
            if (direction != p.heading && abs(presetHeadingChange(p.heading,direction)) != 2) next.lateral += distance;
            else if (direction != p.heading) next.reverse += distance;
            next.distance += distance;
            if (presetCostLess(next,costs[state])) { costs[state] = next; previous[state] = (int16_t)best; }
        }
    }
    if (finish < 0) return 0U;
    do {
        if (count >= PRESET_STATE_COUNT) return 0U;
        reverse[count++] = (uint16_t)finish; finish = previous[finish];
    } while (finish >= 0);
    for (i = count-1U; i > 0U; --i) {
        PresetPose from = presetNode(reverse[i]/4U,reverse[i]%4U);
        PresetPose to = presetNode(reverse[i-1U]/4U,reverse[i-1U]%4U);
        if (!presetAppend(route,from,to,taskIndex,0)) return 0U;
    }
    return 1U;
}
static void presetSelectArcs(PresetRoute *route)
{
    unsigned i;
    for (i = 0; i+1U < route->count; ++i) {
        PresetLeg *before = &route->legs[i], *after = &route->legs[i+1U];
        int available = abs(before->drive.upMm)+abs(before->drive.rightMm);
        int following = abs(after->drive.upMm)+abs(after->drive.rightMm);
        int nominal = abs(before->to.x-before->from.x)+abs(before->to.y-before->from.y);
        if (i && route->legs[i-1U].arc) { available -= PRESET_ARC_TRIM; nominal -= PRESET_ARC_TRIM; }
        if (before->drive.event || before->to.x != after->from.x || before->to.y != after->from.y ||
            presetDirection(before->from,before->to) != before->from.heading ||
            presetDirection(after->from,after->to) != after->from.heading ||
            available <= PRESET_ARC_TRIM+2 || nominal <= PRESET_ARC_TRIM+2 ||
            following <= PRESET_ARC_TRIM+32 ||
            abs(after->to.x-after->from.x)+abs(after->to.y-after->from.y) <= PRESET_ARC_TRIM+32) continue;
        if (presetArcClear(before,after,route->regions)) {
            before->arc = 1U; before->drive.heading = after->from.heading;
        }
    }
}
static uint8_t presetBuild(PresetRoute *route, uint8_t zone, uint16_t regions)
{
    /* Preserve each event leg's calibrated distance, direction and working heading.
     * Only the transit between those fixed approach segments is replanned. */
    static const uint8_t events[8] = {1,3,6,8,10,13,15,19};
    static const PresetPose approaches[7] = {
        {2100,2080,1}, {400,400,-1}, {400,400,2}, {400,2080,1},
        {400,400,-1}, {400,400,2}, {2100,2250,-1}
    };
    static const PresetPose stations[7] = {
        {1200,2080,1}, {1200,400,-1}, {400,1200,2}, {1200,2080,1},
        {1200,400,-1}, {400,1200,2}, {2250,2250,-1}
    };
    PresetPose current = {2250,2250,0}, gate = {2100,2250,0}, qr = {2100,1200,0};
    RouteLeg calibrated;
    unsigned stage;
    memset(route,0,sizeof(*route));
    if ((zone != 1U && zone != 2U) || !ss_obstacle_regions_valid(regions)) return 0U;
    route->regions = regions;
    if (zone == 2U) current.y = gate.y = 150;
    calibrated = routeDistanceLeg(0U,zone);
    if (!presetAppend(route,current,gate,0U,&calibrated)) goto fail;
    calibrated = routeDistanceLeg(events[0],zone);
    if (!presetAppend(route,gate,qr,events[0],&calibrated)) goto fail;
    current = qr;
    for (stage = 0; stage < 7U; ++stage) {
        PresetPose approach = approaches[stage], station = stations[stage];
        if (stage == 6U && zone == 2U) approach.y = station.y = 150;
        if (!presetConnect(route,current,approach,events[stage+1U])) goto fail;
        /* A path can end with a heading-only transition. The fixed approach
         * executes that departure turn; the search already checked its sweep. */
        calibrated = routeDistanceLeg(events[stage+1U],zone);
        if (!presetAppend(route,approach,station,events[stage+1U],&calibrated)) goto fail;
        current = station;
    }
    presetSelectArcs(route);
    return 1U;
fail:
    route->count = route->index = 0U; return 0U;
}
static RouteLeg presetActiveLeg(const PresetRoute *route)
{
    RouteLeg leg = route->legs[route->index].drive;
    int trim = (route->legs[route->index].arc ? PRESET_ARC_TRIM : 0) +
               (route->index && route->legs[route->index-1U].arc ? PRESET_ARC_TRIM : 0);
    if (leg.upMm) leg.upMm += leg.upMm > 0 ? -trim : trim;
    else leg.rightMm += leg.rightMm > 0 ? -trim : trim;
    return leg;
}
/* 0 finished, 1 transit, 2 station. Task indices never count detour nodes. */
static uint8_t presetAdvance(PresetRoute *route, uint8_t *taskIndex)
{
    const PresetLeg *leg = &route->legs[route->index++];
    if (route->index >= route->count) return 0U;
    if (leg->drive.event) { *taskIndex = (uint8_t)(leg->taskIndex+1U); return 2U; }
    *taskIndex = route->legs[route->index].taskIndex; return 1U;
}
#endif
