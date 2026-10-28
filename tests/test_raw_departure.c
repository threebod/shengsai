#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include "../firmware/preset_route.h"

int main(void)
{
    static PresetRoute route;
    unsigned zone, a, b, planned = 0U, middle = 0U;
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
                            assert(leg.upMm == 0 && leg.rightMm == 30 && !leg.event);
                            assert(leg.departureHeading == 1 && leg.heading == 1);
                            routeBody(leg.upMm, leg.rightMm, 1, &forward, &right);
                            assert(forward == -30 && right == 0);
                        } else {
                            assert(step == 2U && leg.upMm == -1580 && leg.rightMm == 0 && leg.event);
                            assert(leg.departureHeading == 2 && leg.heading == -1);
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
    assert(planned == 208U && middle == 260U);
    /* The complete 30mm right-biased corridor fits; larger bias reaches fixed blocks. */
    assert(presetRectangleClear(1200, 400, 1230, 2080, 0U));
    assert(!presetRectangleClear(1200, 400, 1260, 2080, 0U));
    puts("PASS RAW departure: down100/right30/down1580, headings, station events, two rounds/zones and unchanged other routes");
    return 0;
}
