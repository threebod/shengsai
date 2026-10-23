#ifndef SHENGSAI_PRESET_ROUTE_H
#define SHENGSAI_PRESET_ROUTE_H
#include <stdlib.h>
#include <string.h>
#include "../shared/obstacle_regions.h"
#include "route_plan.h"
#include "arc_route_blend.h"

#define PRESET_GRID_COLUMNS 7U
#define PRESET_GRID_ROWS 11U
#define PRESET_NODE_COUNT (PRESET_GRID_COLUMNS * PRESET_GRID_ROWS)
/* Keep forward arrivals distinct: a cheaper reverse arrival cannot supply the
 * incoming tangent for a continuous corner. */
#define PRESET_NODE_STATES 8U
#define PRESET_STATE_COUNT (PRESET_NODE_COUNT * PRESET_NODE_STATES)
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
static const int16_t presetGridX[PRESET_GRID_COLUMNS] = {300,400,1100,1200,1300,2100,2250};
static const int16_t presetGridY[PRESET_GRID_ROWS] = {150,300,400,1100,1150,1200,1250,1300,2080,2100,2250};
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
    p.x = presetGridX[node % PRESET_GRID_COLUMNS]; p.y = presetGridY[node / PRESET_GRID_COLUMNS];
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
static uint8_t presetBodyClearComponents(float x, float y, float s, float c, uint16_t regions)
{
    float extent = 152.0f*(fabsf(s)+fabsf(c));
    unsigned i;
    /* 1mm arc samples, 2mm dilation bounds travel and corner rotation between samples. */
    if (x-extent < 0.0f || x+extent > 2400.0f || y-extent < 0.0f || y+extent > 2400.0f) return 0U;
    for (i = 0; i < sizeof(presetFixed)/sizeof(presetFixed[0]); ++i)
        if (presetBodyHits(x,y,s,c,152.0f,&presetFixed[i])) return 0U;
    for (i = 0; i < SS_OBSTACLE_REGION_COUNT; ++i)
        if ((regions & (1U << i)) && presetBodyHits(x,y,s,c,152.0f,&ssObstacleRegions[i])) return 0U;
    return 1U;
}
static uint8_t presetBodyClear(float x, float y, float angle, uint16_t regions)
{ return presetBodyClearComponents(x,y,sinf(angle),cosf(angle),regions); }
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
    typedef struct { float x, y, s, c; } ArcSample;
    /* One local left bend, including its final position. Quarter-turns and
     * reflection reuse exactly the same 1mm samples without per-map trig. */
    static ArcSample samples[402];
    static unsigned count;
    unsigned i;
    int8_t change = presetHeadingChange(before->from.heading, after->from.heading);
    if (change != 1 && change != -1) return 0U;
    if (!count) {
        float distance = 0.0f, degrees, curvature, step;
        ArcSample p = {0.0f,-PRESET_ARC_TRIM,0.0f,1.0f};
        while (distance < ARC_BLEND_LENGTH_MM && count+1U < sizeof(samples)/sizeof(samples[0])) {
            samples[count++] = p;
            step = ARC_BLEND_LENGTH_MM-distance < 1.0f ? ARC_BLEND_LENGTH_MM-distance : 1.0f;
            arcBlendShape(distance+step*0.5f,&degrees,&curvature);
            p.s = sinf(degrees*0.01745329252f); p.c = cosf(degrees*0.01745329252f);
            p.x -= p.s*step; p.y += p.c*step; distance += step;
        }
        /* Fail closed if the blend geometry ever outgrows this fixed cache. */
        if (distance < ARC_BLEND_LENGTH_MM) { count = 0U; return 0U; }
        samples[count] = p;
    }
    for (i = 0U; i < count+31U; ++i) {
        ArcSample p = samples[i < count ? i : count];
        float x, y, s, c;
        if (i >= count) {
            /* Include the controller's existing extra 30mm exit allowance. */
            p.x -= (float)(i-count); p.s = 1.0f; p.c = 0.0f;
        }
        p.x *= change; p.s *= change;
        switch (before->from.heading) {
        case 1: x = -p.y; y = p.x; s = p.c; c = -p.s; break;
        case 2: x = -p.x; y = -p.y; s = -p.s; c = -p.c; break;
        case -1: x = p.y; y = -p.x; s = -p.c; c = p.s; break;
        default: x = p.x; y = p.y; s = p.s; c = p.c; break;
        }
        if (!presetBodyClearComponents(before->to.x+x,before->to.y+y,s,c,regions)) return 0U;
    }
    return 1U;
}
static uint8_t presetCostLess(PresetCost a, PresetCost b)
{
    if (a.distance != b.distance) return a.distance < b.distance;
    if (a.lateral != b.lateral) return a.lateral < b.lateral;
    return a.reverse < b.reverse;
}
/* The score is distance plus finite manoeuvre costs: avoiding a short reverse
 * or lateral movement must not force an otherwise unnecessary trip around the map. */
static PresetCost presetTravelCost(PresetCost cost, PresetPose from, PresetPose to)
{
    unsigned distance = (unsigned)(abs(to.x-from.x)+abs(to.y-from.y));
    int8_t direction = presetDirection(from,to);
    cost.distance += distance;
    if (direction != from.heading && abs(presetHeadingChange(from.heading,direction)) != 2) {
        cost.lateral += distance; cost.distance += distance*2U;
    } else if (direction != from.heading) {
        cost.reverse += distance; cost.distance += distance/2U;
    }
    return cost;
}
/* Cost hint only. Actual selected arcs still need adjacent-leg length checks
 * and the full swept-body test in presetSelectArcs. */
static uint8_t presetTurnHasArc(PresetPose p, int8_t heading, uint16_t regions)
{
    PresetLeg before, after;
    memset(&before,0,sizeof(before)); memset(&after,0,sizeof(after));
    before.from = before.to = after.from = after.to = p;
    after.from.heading = after.to.heading = heading;
    return presetArcClear(&before,&after,regions);
}
/* Nominal geometry remains in map mm. Existing calibrated event distances are
 * scaled by their original nominal span, rather than adding a full-route offset
 * to a 100mm terminal segment (which could reverse its commanded direction). */
static RouteLeg presetStationLeg(RouteLeg calibrated, PresetPose from, PresetPose to, unsigned span)
{
    int distance = calibrated.upMm ? to.y-from.y : to.x-from.x;
    unsigned tuned = (unsigned)abs(calibrated.upMm ? calibrated.upMm : calibrated.rightMm);
    int scaled = (int)(((uint32_t)abs(distance)*tuned+span/2U)/span);
    if (distance < 0) scaled = -scaled;
    if (calibrated.upMm) calibrated.upMm = (int16_t)scaled;
    else calibrated.rightMm = (int16_t)scaled;
    return calibrated;
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
static uint8_t presetConnect(PresetRoute *route, PresetPose start, PresetPose station,
                              uint8_t taskIndex, RouteLeg calibrated)
{
    static PresetCost costs[PRESET_STATE_COUNT];
    static int16_t previous[PRESET_STATE_COUNT];
    static uint8_t visited[PRESET_STATE_COUNT];
    static uint16_t reverse[PRESET_STATE_COUNT];
    /* Lazy arc hints are reused between station searches for the same map. */
    static uint8_t arcHints[PRESET_NODE_COUNT * 4U][4];
    static uint16_t hintRegions;
    PresetPose targets[3], selected;
    int startNode = presetNodeAt(start), targetNodes[3], finish = -1;
    unsigned i, count = 0U, targetCount = taskIndex == 19U ? 1U : 3U;
    uint8_t direct = 0U;
    unsigned span = taskIndex == 3U ? 900U : taskIndex == 19U ? 150U : 800U;
    const PresetCost infinity = {0xFFFFFFFFUL,0xFFFFFFFFUL,0xFFFFFFFFUL};
    PresetCost finishCost = infinity;
    if (startNode < 0 || !presetSegmentClear(start,start,route->regions) ||
        !presetSegmentClear(station,station,route->regions)) return 0U;
    if (hintRegions != route->regions) {
        memset(arcHints,0,sizeof(arcHints)); hintRegions = route->regions;
    }
    for (i = 0; i < targetCount; ++i) {
        PresetPose endpoint;
        RouteLeg terminal;
        targets[i] = station;
        if (i == 2U) { targetNodes[i] = presetNodeAt(station); continue; }
        if (calibrated.upMm) targets[i].y += i ? 100 : -100;
        else targets[i].x += taskIndex == 19U ? -150 : i ? 100 : -100;
        targetNodes[i] = presetNodeAt(targets[i]);
        terminal = presetStationLeg(calibrated,targets[i],station,span);
        endpoint = targets[i]; endpoint.x += terminal.rightMm; endpoint.y += terminal.upMm;
        if (!presetSegmentClear(targets[i],station,route->regions) ||
            !presetSegmentClear(targets[i],endpoint,route->regions)) targetNodes[i] = -1;
    }
    for (i = 0; i < PRESET_STATE_COUNT; ++i) { costs[i] = infinity; previous[i] = -1; visited[i] = 0U; }
    costs[startNode*PRESET_NODE_STATES+presetHeadingIndex(start.heading)].lateral = 0U;
    costs[startNode*PRESET_NODE_STATES+presetHeadingIndex(start.heading)].reverse = 0U;
    costs[startNode*PRESET_NODE_STATES+presetHeadingIndex(start.heading)].distance = 0U;
    for (;;) {
        int best = -1;
        PresetCost cost = infinity;
        unsigned node, heading, neighbor, forwardLength = 0U;
        PresetPose p;
        for (i = 0; i < PRESET_STATE_COUNT; ++i)
            if (!visited[i] && presetCostLess(costs[i],cost)) { best = (int)i; cost = costs[i]; }
        if (best < 0 || !presetCostLess(cost,finishCost)) break;
        node = (unsigned)best/PRESET_NODE_STATES; heading = (unsigned)best%4U; p = presetNode(node,heading);
        if ((unsigned)best%PRESET_NODE_STATES >= 4U) {
            int cursor = best;
            while (previous[cursor] >= 0 && forwardLength <= PRESET_ARC_TRIM+2U) {
                unsigned prior = (unsigned)previous[cursor];
                PresetPose from = presetNode(prior/PRESET_NODE_STATES,prior%4U);
                PresetPose to = presetNode((unsigned)cursor/PRESET_NODE_STATES,(unsigned)cursor%4U);
                if (from.heading != p.heading || (from.x == to.x && from.y == to.y) ||
                    presetDirection(from,to) != p.heading) break;
                forwardLength += (unsigned)(abs(to.x-from.x)+abs(to.y-from.y)); cursor = (int)prior;
            }
        }
        for (i = 0; i < targetCount; ++i) if ((int)node == targetNodes[i]) {
            PresetCost next = cost;
            if (i == 2U) {
                if (p.heading != station.heading) {
                    if (!presetTurnClear(station,route->regions)) continue;
                    next.distance += abs(presetHeadingChange(p.heading,station.heading)) == 2 ? 240U : 720U;
                }
            } else {
                if (p.heading != station.heading) continue;
                next = presetTravelCost(cost,p,station);
            }
            if (presetCostLess(next,finishCost)) {
                finish = best; finishCost = next; selected = p; direct = (uint8_t)(i == 2U);
            }
        }
        visited[best] = 1U;
        if (presetTurnClear(p,route->regions)) for (neighbor = 0; neighbor < 4U; ++neighbor) {
            unsigned state = node*PRESET_NODE_STATES+neighbor, hint = node*4U+heading;
            PresetCost next = cost;
            uint8_t forwardArrival = (uint8_t)(forwardLength > PRESET_ARC_TRIM+2U);
            if (neighbor == heading) continue;
            if (neighbor == (heading+2U)%4U) next.distance += 240U;
            else {
                if (!arcHints[hint][neighbor]) arcHints[hint][neighbor] =
                    presetTurnHasArc(p,presetHeading(neighbor),route->regions) ? 1U : 2U;
                next.distance += forwardArrival && arcHints[hint][neighbor] == 1U ? 120U : 720U;
            }
            if (presetCostLess(next,costs[state])) { costs[state] = next; previous[state] = (int16_t)best; }
        }
        for (neighbor = 0; neighbor < PRESET_NODE_COUNT; ++neighbor) {
            unsigned state;
            PresetPose q = presetNode(neighbor,heading);
            PresetCost next = cost;
            if (!((neighbor/PRESET_GRID_COLUMNS == node/PRESET_GRID_COLUMNS && (neighbor+1U == node || node+1U == neighbor)) ||
                  (neighbor%PRESET_GRID_COLUMNS == node%PRESET_GRID_COLUMNS &&
                   (neighbor+PRESET_GRID_COLUMNS == node || node+PRESET_GRID_COLUMNS == neighbor)))) continue;
            /* The boundary anchors serve only the start-zone entrances. */
            if ((p.x == 2250 || q.x == 2250) && !(p.y == q.y && (p.y == 150 || p.y == 2250))) continue;
            if ((p.y == 150 || p.y == 2250) && p.x < 2100) continue;
            if ((q.y == 150 || q.y == 2250) && q.x < 2100) continue;
            if (!presetSegmentClear(p,q,route->regions)) continue;
            state = neighbor*PRESET_NODE_STATES+heading+(presetDirection(p,q) == p.heading ? 4U : 0U);
            next = presetTravelCost(next,p,q);
            if (presetCostLess(next,costs[state])) { costs[state] = next; previous[state] = (int16_t)best; }
        }
    }
    if (finish < 0) return 0U;
    do {
        if (count >= PRESET_STATE_COUNT) return 0U;
        reverse[count++] = (uint16_t)finish; finish = previous[finish];
    } while (finish >= 0);
    for (i = count-1U; i > 0U; --i) {
        PresetPose from = presetNode(reverse[i]/PRESET_NODE_STATES,reverse[i]%4U);
        PresetPose to = presetNode(reverse[i-1U]/PRESET_NODE_STATES,reverse[i-1U]%4U);
        if (!presetAppend(route,from,to,taskIndex,0)) return 0U;
    }
    if (direct) {
        PresetLeg *last;
        PresetPose endpoint;
        if (!route->count) return 0U;
        last = &route->legs[route->count-1U];
        if (last->drive.event || last->taskIndex != taskIndex ||
            last->to.x != station.x || last->to.y != station.y) return 0U;
        /* Direct arrival can approach along the other map axis. Apply an old
         * axis calibration only when that axis is actually being traversed. */
        if ((last->drive.upMm != 0) == (calibrated.upMm != 0))
            last->drive = presetStationLeg(calibrated,last->from,station,span);
        else last->drive.event = calibrated.event;
        last->to.heading = last->drive.heading = station.heading;
        last->drive.departureHeading = last->from.heading;
        endpoint = last->from; endpoint.x += last->drive.rightMm; endpoint.y += last->drive.upMm;
        return presetSegmentClear(last->from,endpoint,route->regions);
    }
    if (route->count) {
        PresetLeg *last = &route->legs[route->count-1U];
        if (!last->drive.event && last->taskIndex == taskIndex &&
            last->from.heading == selected.heading && last->to.heading == selected.heading &&
            last->to.x == selected.x && last->to.y == selected.y &&
            presetDirection(last->from,last->to) == presetDirection(selected,station)) {
            /* Preserve one uninterrupted station approach for calibration and
             * observation lead time; never merge across a departure turn. */
            selected = last->from; --route->count;
        }
    }
    calibrated = presetStationLeg(calibrated,selected,station,span);
    {
        PresetPose endpoint = selected;
        endpoint.x += calibrated.rightMm; endpoint.y += calibrated.upMm;
        if (!presetSegmentClear(selected,endpoint,route->regions)) return 0U;
    }
    return presetAppend(route,selected,station,taskIndex,&calibrated);
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
    /* Workstations and working headings remain fixed. Enter directly when safe,
     * or use a short approach without reserving an entire map-side corridor. */
    static const uint8_t events[8] = {1,3,6,8,10,13,15,19};
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
        PresetPose station = stations[stage];
        if (stage == 6U && zone == 2U) station.y = 150;
        calibrated = routeDistanceLeg(events[stage+1U],zone);
        /* Search-only turns are executed by the event's departure heading or
         * its station heading; both swept turns were checked by the search. */
        if (!presetConnect(route,current,station,events[stage+1U],calibrated)) goto fail;
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
