#include <stdio.h>
#include "../firmware/preset_route.h"

/* Read-only host export: execute the production planner, never the hardware runtime. */
static void rectangle(const SsObstacleRegion *r)
{
    printf("[%d,%d,%d,%d]", r->x0, r->y0, r->x1, r->y1);
}

int main(void)
{
    static PresetRoute route;
    unsigned i, zone, a, b;
    float distance = 0, x = 0, y = -PRESET_ARC_TRIM, degrees, curvature;
    printf("{\"size\":2400,\"body\":300,\"trim\":%d,\"fixed\":[", PRESET_ARC_TRIM);
    for (i = 0; i < sizeof(presetFixed)/sizeof(presetFixed[0]); ++i) {
        if (i) putchar(',');
        rectangle(&presetFixed[i]);
    }
    printf("],\"regions\":[");
    for (i = 0; i < SS_OBSTACLE_REGION_COUNT; ++i) {
        if (i) putchar(',');
        rectangle(&ssObstacleRegions[i]);
    }
    /* Same 1 mm midpoint integration as the production swept-body check. */
    printf("],\"arc\":[[0,%.4f,0]", y);
    while (distance < ARC_BLEND_LENGTH_MM) {
        float step = ARC_BLEND_LENGTH_MM-distance < 1 ? ARC_BLEND_LENGTH_MM-distance : 1;
        arcBlendShape(distance+step*0.5f, &degrees, &curvature);
        x -= sinf(degrees*0.01745329252f)*step;
        y += cosf(degrees*0.01745329252f)*step;
        distance += step;
        if (((unsigned)distance % 4U) == 0U || distance >= ARC_BLEND_LENGTH_MM) {
            arcBlendShape(distance, &degrees, &curvature);
            printf(",[%.4f,%.4f,%.4f]", x, y, degrees);
        }
    }
    printf("],\"routes\":{");
    for (zone = 1; zone <= 2; ++zone) for (a = 1; a <= 15; ++a) for (b = a+1; b <= 15; ++b) {
        uint8_t valid = presetBuild(&route, (uint8_t)zone, (uint16_t)((1U << (a-1U)) | (1U << (b-1U))));
        if (zone != 1 || a != 1 || b != 2) putchar(',');
        printf("\"%u-%u-%u\":", zone, a, b);
        if (!valid) { printf("null"); continue; }
        putchar('[');
        for (i = 0; i < route.count; ++i) {
            const PresetLeg *l = &route.legs[i];
            RouteLeg active;
            route.index = (uint16_t)i;
            active = presetActiveLeg(&route);
            if (i) putchar(',');
            printf("{\"from\":[%d,%d,%d],\"to\":[%d,%d,%d],\"task\":%u,\"arc\":%u,\"fourPointArc\":%u,\"experimentalArc\":%u,"
                   "\"up\":%d,\"right\":%d,\"departure\":%d,\"heading\":%d,\"event\":",
                   l->from.x,l->from.y,l->from.heading,l->to.x,l->to.y,l->to.heading,
                   l->taskIndex,!!l->arc,l->arc == PRESET_ARC_FOUR_POINT,l->arc == PRESET_ARC_EXPERIMENTAL_713,
                   active.upMm,active.rightMm,active.departureHeading,active.heading);
            if (l->drive.event) printf("\"%s\"",l->drive.event); else printf("null");
            putchar('}');
        }
        putchar(']');
    }
    printf("}}\n");
    return 0;
}
