#ifndef SHENGSAI_PLACEMENT_HEIGHTS_H
#define SHENGSAI_PLACEMENT_HEIGHTS_H

#include "mechanism_action.h"

typedef struct {
    uint16_t firstLayerDmm;
    uint16_t secondLayerDmm;
    uint16_t rawTrayDmm;
    uint16_t returnTrayDmm;
} PlacementHeights;

static PlacementHeights placementHeights = {1270U, 600U, 170U, 200U};

static uint8_t placementHeightsValid(const PlacementHeights *heights)
{
    return heights->firstLayerDmm <= MECHANISM_LIFT_MAX_DMM &&
           heights->secondLayerDmm <= MECHANISM_LIFT_MAX_DMM &&
           heights->rawTrayDmm <= MECHANISM_LIFT_MAX_DMM &&
           heights->returnTrayDmm <= MECHANISM_LIFT_MAX_DMM;
}

#endif
