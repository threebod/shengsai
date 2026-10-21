#ifndef TJC_OBSTACLES_H
#define TJC_OBSTACLES_H

#include <stdint.h>

/* Screen -> USART6: A5 5A 4F region1 region2 (4F + region1 + region2).
 * Regions are 1..15; a returned mask always contains exactly two bits. */
typedef struct {
    uint8_t bytes[6];
    uint8_t length;
} TjcObstacleParser;

static uint16_t tjc_obstacles_feed(TjcObstacleParser *parser, uint8_t byte)
{
    uint8_t i, first, second;
    if (parser->length == sizeof(parser->bytes)) {
        for (i = 1U; i < sizeof(parser->bytes); ++i)
            parser->bytes[i - 1U] = parser->bytes[i];
        --parser->length;
    }
    parser->bytes[parser->length++] = byte;
    if (parser->length != sizeof(parser->bytes) ||
        parser->bytes[0] != 0xA5U || parser->bytes[1] != 0x5AU ||
        parser->bytes[2] != 0x4FU) return 0U;
    first = parser->bytes[3];
    second = parser->bytes[4];
    if (first < 1U || first > 15U || second < 1U || second > 15U ||
        first == second || parser->bytes[5] != (uint8_t)(0x4FU + first + second))
        return 0U;
    return (uint16_t)((1U << (first - 1U)) | (1U << (second - 1U)));
}

#endif
