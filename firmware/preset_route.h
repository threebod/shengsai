#ifndef SHENGSAI_PRESET_ROUTE_H
#define SHENGSAI_PRESET_ROUTE_H
#include <stdlib.h>
#include <string.h>
#include "../shared/obstacle_regions.h"
#include "route_plan.h"
#include "route_candidates.h"
#include "arc_route_blend.h"

#define PRESET_LEG_CAPACITY 128U
#define PRESET_ARC_TRIM 249
/* Original four-point bends retain their selected-obstacle/boundary scope. */
#define PRESET_ARC_FOUR_POINT 2U
/* RAW table projects about 80mm into the field; clear it by 100mm before turning. */
#define PRESET_RAW_CLEAR_MM 100
#define PRESET_RAW_RIGHT_MM 50

typedef struct {
    RouteLeg drive;
    PresetPose from, to;
    uint8_t taskIndex, arc;
    uint16_t templateId;
} PresetLeg;
typedef struct {
    PresetLeg legs[PRESET_LEG_CAPACITY];
    uint16_t count, index, regions;
} PresetRoute;

static const SsObstacleRegion presetFixed[] = {
    {550,550,1000,1000}, {1400,550,1850,1000},
    {550,1400,1000,1850}, {1400,1400,1850,1850},
    {0,910,150,1490}, {910,0,1490,150}
};
static uint8_t presetOverlap(float x0, float y0, float x1, float y1, const SsObstacleRegion *r)
{ return (uint8_t)(x0 < r->x1 && x1 > r->x0 && y0 < r->y1 && y1 > r->y0); }
static uint8_t presetRectangleClear(float x0, float y0, float x1, float y1, uint16_t regions)
{
    unsigned i;
    if (x0 < 150 || x1 > 2250 || y0 < 150 || y1 > 2250) return 0U;
    for (i = 0; i < sizeof(presetFixed)/sizeof(presetFixed[0]); ++i)
        if (presetOverlap(x0-150,y0-150,x1+150,y1+150,&presetFixed[i])) return 0U;
    for (i = 0; i < SS_OBSTACLE_REGION_COUNT; ++i)
        if ((regions & (1U << i)) && presetOverlap(x0-150,y0-150,x1+150,y1+150,&ssObstacleRegions[i])) return 0U;
    return 1U;
}
static uint8_t presetSegmentClear(PresetPose from, PresetPose to, uint16_t regions)
{
    if (from.x != to.x && from.y != to.y) return 0U;
    return presetRectangleClear(from.x < to.x ? from.x : to.x,
                                from.y < to.y ? from.y : to.y,
                                from.x > to.x ? from.x : to.x,
                                from.y > to.y ? from.y : to.y, regions);
}
static uint8_t presetRecenterClear(const PresetRoute *route, float forward, float right)
{
    const PresetLeg *arrived;
    float x0, y0, x1, y1, x, y, dx, dy;
    if (!route->index || route->index > route->count ||
        !(forward >= -2400.0f && forward <= 2400.0f &&
          right >= -2400.0f && right <= 2400.0f)) return 0U;
    arrived = &route->legs[route->index-1U];
    if (!arrived->drive.event) return 0U;
    x = arrived->from.x + arrived->drive.rightMm;
    y = arrived->from.y + arrived->drive.upMm;
    x0 = x < arrived->to.x ? x : arrived->to.x;
    x1 = x > arrived->to.x ? x : arrived->to.x;
    y0 = y < arrived->to.y ? y : arrived->to.y;
    y1 = y > arrived->to.y ? y : arrived->to.y;
    switch (arrived->drive.heading) {
    case 1: dx = -forward; dy = right; break;
    case 2: dx = -right; dy = -forward; break;
    case -1: dx = forward; dy = -right; break;
    default: dx = right; dy = forward; break;
    }
    /* Bound both map axes, including a position command whose wheel pairs
     * finish at different times. This covers the entire return envelope. */
    return presetRectangleClear(x0+(dx < 0 ? dx : 0), y0+(dy < 0 ? dy : 0),
                                x1+(dx > 0 ? dx : 0), y1+(dy > 0 ? dy : 0), route->regions);
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
static uint8_t presetBodyClearScope(float x, float y, float s, float c, uint16_t regions,
                                    uint8_t checkFixed)
{
    float extent = 152.0f*(fabsf(s)+fabsf(c));
    unsigned i;
    /* 1mm arc samples, 2mm dilation bounds travel and corner rotation between samples. */
    if (x-extent < 0.0f || x+extent > 2400.0f || y-extent < 0.0f || y+extent > 2400.0f) return 0U;
    if (checkFixed) for (i = 0; i < sizeof(presetFixed)/sizeof(presetFixed[0]); ++i)
        if (presetBodyHits(x,y,s,c,152.0f,&presetFixed[i])) return 0U;
    for (i = 0; i < SS_OBSTACLE_REGION_COUNT; ++i)
        if ((regions & (1U << i)) && presetBodyHits(x,y,s,c,152.0f,&ssObstacleRegions[i])) return 0U;
    return 1U;
}
static uint8_t presetBodyClearComponents(float x, float y, float s, float c, uint16_t regions)
{ return presetBodyClearScope(x,y,s,c,regions,1U); }
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
    float cornerY = before->to.y;
    int8_t heading = before->from.heading;
    int8_t change = presetHeadingChange(heading, after->from.heading);
    /* RC-3 keeps nominal station anchors, but its lower bend executes 50mm lower. */
    if (before->templateId == 20U && before->drive.upMm < 0)
        cornerY -= PRESET_RC3_EXTRA_DOWN_MM;
    if (before->from.x != before->to.x || before->from.y != before->to.y) {
        int8_t incoming = presetHeadingChange(heading,presetDirection(before->from,before->to));
        int8_t outgoing = presetHeadingChange(after->from.heading,presetDirection(after->from,after->to));
        if ((incoming != 0 && abs(incoming) != 2) ||
            (outgoing != 0 && abs(outgoing) != 2) ||
            (incoming == 0) != (outgoing == 0)) return 0U;
        if (incoming) heading = presetDirection(before->from,before->to);
    }
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
        switch (heading) {
        case 1: x = -p.y; y = p.x; s = p.c; c = -p.s; break;
        case 2: x = -p.x; y = -p.y; s = -p.s; c = -p.c; break;
        case -1: x = p.y; y = -p.x; s = -p.c; c = p.s; break;
        default: x = p.x; y = p.y; s = p.s; c = p.c; break;
        }
        if (!presetBodyClearScope(before->to.x+x,cornerY+y,s,c,regions,
                                  (uint8_t)(before->arc != PRESET_ARC_FOUR_POINT))) return 0U;
    }
    return 1U;
}
/* A failed candidate leaves the already-built route prefix intact. There is
 * no search, arc insertion/removal, heading substitution or distance rescaling. */
static uint8_t presetTryTemplate(PresetRoute *route, const PresetGroup *group,
                                uint16_t templateId, uint8_t taskIndex, const char *event)
{
    const PresetCandidate *candidate;
    PresetPose current = group->start;
    unsigned i, first = route->count;
    if (!templateId || templateId > PRESET_TEMPLATE_COUNT) return 0U;
    candidate = &presetTemplates[templateId];
    if (!candidate->count || first+candidate->count > PRESET_LEG_CAPACITY) return 0U;
    for (i = 0U; i < candidate->count; ++i) {
        const PresetStep *step = &candidate->steps[i];
        PresetLeg *leg = &route->legs[first+i];
        memset(leg,0,sizeof(*leg));
        current.heading = step->departureHeading;
        leg->from = current;
        leg->to.x = step->x; leg->to.y = step->y; leg->to.heading = step->arrivalHeading;
        leg->drive.upMm = step->upMm; leg->drive.rightMm = step->rightMm;
        leg->drive.departureHeading = step->departureHeading; leg->drive.heading = step->commandHeading;
        leg->drive.event = i+1U == candidate->count ? event : 0;
        leg->taskIndex = taskIndex == 1U && !i ? 0U : taskIndex;
        leg->arc = step->arc; leg->templateId = templateId;
        current = leg->to;
    }
    if (current.x != group->goal.x || current.y != group->goal.y ||
        current.heading != group->goal.heading) return 0U;
    for (i = first; i < first+candidate->count; ++i) {
        const PresetLeg *leg = &route->legs[i];
        int dx = leg->to.x-leg->from.x, dy = leg->to.y-leg->from.y;
        int available = abs(leg->drive.upMm)+abs(leg->drive.rightMm);
        int trim = (leg->arc ? PRESET_ARC_TRIM : 0) +
                   (i > first && route->legs[i-1U].arc ? PRESET_ARC_TRIM : 0);
        if ((!dx && !dy) || (dx && (!leg->drive.rightMm || leg->drive.upMm || dx*leg->drive.rightMm <= 0)) ||
            (dy && (!leg->drive.upMm || leg->drive.rightMm || dy*leg->drive.upMm <= 0)) ||
            available <= trim || !presetSegmentClear(leg->from,leg->to,route->regions)) return 0U;
        /* RC-1 shifts right by 50mm and keeps the full 1680mm final down leg.
         * Include the extra 100mm departure in the complete movement envelope. */
        if (templateId == 17U &&
            !presetRectangleClear(leg->from.x, leg->to.y - PRESET_RAW_CLEAR_MM,
                                  leg->from.x + PRESET_RAW_RIGHT_MM,
                                  leg->from.y, route->regions)) return 0U;
        /* Cover the lower RC-3 command path and the nominal station alignment band. */
        if (templateId == 20U && i > first &&
            !presetRectangleClear(leg->from.x < leg->to.x ? leg->from.x : leg->to.x,
                                  (leg->from.y < leg->to.y ? leg->from.y : leg->to.y) - PRESET_RC3_EXTRA_DOWN_MM,
                                  leg->from.x > leg->to.x ? leg->from.x : leg->to.x,
                                  leg->from.y > leg->to.y ? leg->from.y : leg->to.y, route->regions)) return 0U;
        /* Calibrated motor commands are not nominal map displacements.
         * Select corridors by map endpoints; keep station alignment unchanged. */
        /* Station alignment and the fixed template define parking turns.
         * Do not reject or alter them using a rotational-clearance model. */
        if (leg->arc) {
            const PresetLeg *after;
            if (leg->arc > PRESET_ARC_FOUR_POINT || i+1U == first+candidate->count || leg->drive.event) return 0U;
            after = &route->legs[i+1U];
            if (leg->drive.heading != after->from.heading || available <= trim+2 ||
                abs(dx)+abs(dy) <= trim+2 ||
                abs(after->drive.upMm)+abs(after->drive.rightMm) <= PRESET_ARC_TRIM+32 ||
                abs(after->to.x-after->from.x)+abs(after->to.y-after->from.y) <= PRESET_ARC_TRIM+32 ||
                !presetArcClear(leg,after,route->regions)) return 0U;
        } else {
            if (leg->drive.heading != leg->to.heading) return 0U;
        }
    }
    route->count = (uint16_t)(first+candidate->count);
    return 1U;
}
static uint8_t presetGroupConnect(PresetRoute *route, unsigned groupIndex,
                                 uint8_t taskIndex, const char *event)
{
    const PresetGroup *group;
    unsigned i;
    if (groupIndex >= PRESET_GROUP_COUNT) return 0U;
    group = &presetGroups[groupIndex];
    for (i = 0U; i < group->count; ++i)
        if (presetTryTemplate(route,group,group->templates[i],taskIndex,event)) return 1U;
    return 0U;
}
static uint8_t presetBuild(PresetRoute *route, uint8_t zone, uint16_t regions)
{
    static const uint8_t events[8] = {1,3,6,8,10,13,15,19};
    unsigned stage;
    memset(route,0,sizeof(*route));
    if ((zone != 1U && zone != 2U) || !ss_obstacle_regions_valid(regions)) return 0U;
    route->regions = regions;
    for (stage = 0U; stage < 8U; ++stage) {
        unsigned group = stage == 0U ? (zone == 1U ? PRESET_GROUP_S1 : PRESET_GROUP_S2) :
                         stage == 1U ? (zone == 1U ? PRESET_GROUP_Q1 : PRESET_GROUP_Q2) :
                         stage == 2U || stage == 5U ? PRESET_GROUP_RC :
                         stage == 3U || stage == 6U ? PRESET_GROUP_CT :
                         stage == 4U ? PRESET_GROUP_TR :
                         (zone == 1U ? PRESET_GROUP_H1 : PRESET_GROUP_H2);
        RouteLeg calibrated = routeDistanceLeg(events[stage],zone);
        if (!presetGroupConnect(route,group,events[stage],calibrated.event)) {
            route->count = route->index = 0U; return 0U;
        }
    }
    return 1U;
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
/* RC-1 executes this fixed departure profile from the visual alignment pose.
 * The map leg retains station anchors; the local 50mm right shift is not undone. */
static RouteLeg presetExecutionLeg(const PresetRoute *route, uint8_t step)
{
    RouteLeg leg = presetActiveLeg(route);
    if (route->legs[route->index].templateId != 17U) return leg;
    if (step < 2U) {
        leg.upMm = step == 0U ? -PRESET_RAW_CLEAR_MM : 0;
        leg.rightMm = step == 1U ? PRESET_RAW_RIGHT_MM : 0;
        leg.departureHeading = leg.heading = 1;
        leg.event = 0;
    }
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
static uint8_t presetAdvanceExecution(PresetRoute *route, uint8_t *taskIndex,
                                      uint8_t *step)
{
    if (route->legs[route->index].templateId == 17U && *step < 2U) {
        ++*step;
        *taskIndex = route->legs[route->index].taskIndex;
        return 1U;
    }
    *step = 0U;
    return presetAdvance(route, taskIndex);
}
#endif
