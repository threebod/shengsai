#ifndef SHENGSAI_OBSTACLE_REGIONS_H
#define SHENGSAI_OBSTACLE_REGIONS_H
#include <stdint.h>

#define SS_OBSTACLE_REGION_COUNT 15U
typedef struct { int16_t x0, y0, x1, y1; } SsObstacleRegion;
/* mm, bottom-left origin. Numbering matches hmi/obstacle_selector.HMI. */
static const SsObstacleRegion ssObstacleRegions[SS_OBSTACLE_REGION_COUNT] = {
    {0,1850,550,2400}, {550,1850,1000,2400}, {1400,1850,1850,2400},
    {0,1400,550,1850}, {1000,1400,1400,1850}, {1850,1400,2400,1850},
    {550,1000,1000,1400}, {1000,1000,1400,1400}, {1400,1000,1850,1400},
    {0,550,550,1000}, {1000,550,1400,1000}, {1850,550,2400,1000},
    {0,0,550,550}, {550,0,1000,550}, {1400,0,1850,550}
};
static uint8_t ss_obstacle_regions_valid(uint16_t regions)
{
    uint16_t rest;
    if (regions & 0x8000U) return 0U;
    rest = (uint16_t)(regions & (regions - 1U));
    return (uint8_t)(!(rest & (rest - 1U)));
}
#endif
