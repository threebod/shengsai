#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdint.h>

#include "../tjc_obstacles.h"

static uint16_t packet(TjcObstacleParser *parser, uint8_t first, uint8_t second,
                       uint8_t checksum)
{
    uint16_t regions = 0xFFFFU;
    assert(tjc_obstacles_feed(parser, 0xA5U, &regions) == 0U);
    assert(tjc_obstacles_feed(parser, 0x5AU, &regions) == 0U);
    assert(tjc_obstacles_feed(parser, 0x4FU, &regions) == 0U);
    assert(tjc_obstacles_feed(parser, first, &regions) == 0U);
    assert(tjc_obstacles_feed(parser, second, &regions) == 0U);
    if (!tjc_obstacles_feed(parser, checksum, &regions)) {
        assert(regions == 0xFFFFU);
        return 0xFFFFU;
    }
    return regions;
}

int main(void)
{
    TjcObstacleParser parser = {{0}, 0};
    uint16_t regions;
    uint8_t first, second;
    for (first = 0U; first <= 15U; ++first) {
        for (second = 0U; second <= 15U; ++second) {
            uint16_t expected = first && first == second ? 0xFFFFU :
                (uint16_t)((first ? 1U << (first - 1U) : 0U) |
                           (second ? 1U << (second - 1U) : 0U));
            assert(packet(&parser, first, second, (uint8_t)(0x4FU + first + second)) == expected);
        }
    }
    assert(packet(&parser, 16U, 0U, 0x5FU) == 0xFFFFU);
    assert(packet(&parser, 0U, 2U, 0U) == 0xFFFFU);
    assert(packet(&parser, 1U, 16U, 0x60U) == 0xFFFFU);
    assert(packet(&parser, 1U, 15U, 0U) == 0xFFFFU);
    /* Boot/touch replies and a truncated frame must not prevent recovery. */
    assert(tjc_obstacles_feed(&parser, 0xFFU, &regions) == 0U);
    assert(tjc_obstacles_feed(&parser, 0x65U, &regions) == 0U);
    assert(tjc_obstacles_feed(&parser, 0xA5U, &regions) == 0U);
    assert(tjc_obstacles_feed(&parser, 0x5AU, &regions) == 0U);
    assert(packet(&parser, 5U, 12U, 0x60U) == 0x0810U);
    assert(packet(&parser, 12U, 5U, 0x60U) == 0x0810U);
    assert(packet(&parser, 0U, 0U, 0x4FU) == 0U);
    assert(packet(&parser, 5U, 0U, 0x54U) == 0x0010U);
    return 0;
}
