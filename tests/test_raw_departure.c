#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include "../firmware/preset_route.h"

int main(void)
{
    static PresetRoute route;
    unsigned zone, a, b, planned = 0U, middle = 0U, reverse = 0U;
    for (zone = 1U; zone <= 2U; ++zone)
        for (a = 1U; a <= 15U; ++a)
            for (b = a + 1U; b <= 15U; ++b) {
                uint8_t task = 0U, step = 0U;
                unsigned stations = 0U;
                if (!presetBuild(&route, (uint8_t)zone,
                                 (uint16_t)((1U << (a - 1U)) | (1U << (b - 1U))))) continue;
                ++planned;
                while (route.index < route.count) {
                    uint16_t index = route.index;
                    RouteLeg leg = presetExecutionLeg(&route, step);
                    if (route.legs[index].templateId == 17U) {
                        int16_t forward, right;
                        assert(route.legs[index].taskIndex == 6U || route.legs[index].taskIndex == 13U);
                        if (step == 0U) {
                            ++middle;
                            assert(leg.upMm == -100 && leg.rightMm == 0 && !leg.event);
                            assert(leg.departureHeading == 1 && leg.heading == 1);
                            routeBody(leg.upMm, leg.rightMm, 1, &forward, &right);
                            assert(forward == 0 && right == -100);
                        } else if (step == 1U) {
                            assert(leg.upMm == 0 && leg.rightMm == 50 && !leg.event);
                            assert(leg.departureHeading == 1 && leg.heading == 1);
                            routeBody(leg.upMm, leg.rightMm, 1, &forward, &right);
                            assert(forward == -50 && right == 0);
                        } else {
                            assert(step == 2U && leg.upMm == -1680 && leg.rightMm == 0 && leg.event);
                            assert(leg.departureHeading == 2 && leg.heading == -1);
                        }
                    } else if (route.legs[index].templateId == 20U) {
                        const PresetLeg *current = &route.legs[index];
                        int16_t forward, right;
                        assert(step == 0U && index > 0U);
                        assert(current->taskIndex == 6U || current->taskIndex == 13U);
                        assert(route.legs[index-1U].drive.heading == leg.departureHeading);
                        routeBody(leg.upMm, leg.rightMm, leg.departureHeading, &forward, &right);
                        assert(forward < 0 && right == 0);
                        if (leg.rightMm > 0) {
                            ++reverse;
                            assert(leg.rightMm == 651 && leg.upMm == 0 && !leg.event);
                            assert(leg.departureHeading == 1 && leg.heading == 0 && current->arc == 1U);
                            assert(route.legs[index-1U].drive.heading == leg.departureHeading);
                            assert(presetArcClear(current,&route.legs[index+1U],route.regions));
                        } else if (leg.upMm) {
                            PresetLeg unshifted = *current;
                            assert(leg.upMm == -1232 && leg.rightMm == 0 && !leg.event);
                            assert(current->drive.upMm == -1730);
                            assert(leg.departureHeading == 0 && leg.heading == -1 && current->arc == 1U);
                            assert(presetArcClear(current,&route.legs[index+1U],route.regions));
                            unshifted.templateId = 0U;
                            assert(!presetArcClear(&unshifted,&route.legs[index+1U],route.regions));
                        } else {
                            assert(leg.rightMm == -651 && leg.event && !current->arc);
                            assert(leg.departureHeading == -1 && leg.heading == -1);
                        }
                    } else {
                        RouteLeg original = presetActiveLeg(&route);
                        assert(step == 0U && leg.upMm == original.upMm && leg.rightMm == original.rightMm);
                        assert(leg.departureHeading == original.departureHeading && leg.heading == original.heading);
                        assert(leg.event == original.event);
                    }
                    if (leg.event) ++stations;
                    {
                        uint8_t result = presetAdvanceExecution(&route, &task, &step);
                        if (step) {
                            assert(result == 1U && route.index == index && step <= 2U);
                            assert(task == route.legs[index].taskIndex);
                        } else {
                            assert(route.index == index + 1U);
                            if (result == 2U) assert(leg.event);
                        }
                    }
                }
                assert(stations == 8U && step == 0U);
            }
    assert(planned == 208U && middle == 260U && reverse > 0U);
    /* A 50mm right bias touches the nominal boundary; one more mm hits fixed blocks. */
    assert(presetRectangleClear(1200, 300, 1250, 2080, 0U));
    assert(!presetRectangleClear(1200, 300, 1251, 2080, 0U));
    puts("PASS RAW departure: centre unchanged, RC-3 two reverse bends/down50 without parking turns, station events and two rounds/zones");
    return 0;
}
