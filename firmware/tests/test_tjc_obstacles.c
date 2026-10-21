#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdint.h>

#include "../tjc_obstacles.h"

static uint16_t packet(TjcObstacleParser *parser, uint8_t first, uint8_t second,
                       uint8_t checksum)
{
    assert(tjc_obstacles_feed(parser, 0xA5U) == 0U);
    assert(tjc_obstacles_feed(parser, 0x5AU) == 0U);
    assert(tjc_obstacles_feed(parser, 0x4FU) == 0U);
    assert(tjc_obstacles_feed(parser, first) == 0U);
    assert(tjc_obstacles_feed(parser, second) == 0U);
    return tjc_obstacles_feed(parser, checksum);
}

int main(void)
{
    TjcObstacleParser parser = {{0}, 0};
    uint8_t first, second;
    for (first = 1U; first <= 15U; ++first) {
        for (second = 1U; second <= 15U; ++second) {
            uint16_t expected = first == second ? 0U :
                (uint16_t)((1U << (first - 1U)) | (1U << (second - 1U)));
            assert(packet(&parser, first, second, (uint8_t)(0x4FU + first + second)) == expected);
        }
    }
    assert(packet(&parser, 0U, 2U, 0x51U) == 0U);
    assert(packet(&parser, 1U, 16U, 0x60U) == 0U);
    assert(packet(&parser, 1U, 15U, 0U) == 0U);
    /* Boot/touch replies and a truncated frame must not prevent recovery. */
    assert(tjc_obstacles_feed(&parser, 0xFFU) == 0U);
    assert(tjc_obstacles_feed(&parser, 0x65U) == 0U);
    assert(tjc_obstacles_feed(&parser, 0xA5U) == 0U);
    assert(tjc_obstacles_feed(&parser, 0x5AU) == 0U);
    assert(packet(&parser, 5U, 12U, 0x60U) == 0x0810U);
    assert(packet(&parser, 12U, 5U, 0x60U) == 0x0810U);
    return 0;
}
