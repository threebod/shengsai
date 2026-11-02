#ifndef SHENGSAI_LIDAR_PARSER_H
#define SHENGSAI_LIDAR_PARSER_H
#include "../shared/lidar.h"

/* T-mini Plus: AA 55, CT, LSN, FSA, LSA, XOR, then 3 bytes per sample.
 * The response descriptor and the optional CT CRC byte are skipped while
 * seeking a checked packet. A damaged revolution is never published. */
typedef struct {
    uint8_t bytes[775];
    uint16_t used,count,errors,scanId,period;
    uint8_t synchronized,damaged;
    uint32_t started;
    SsLidarPoint points[SS_LIDAR_MAX_POINTS];
} LidarParser;

static void lidar_parser_reset(LidarParser *parser)
{
    memset(parser,0,sizeof(*parser));
}

static void lidar_parser_lost(LidarParser *parser)
{
    parser->used=0U;parser->damaged=1U;++parser->errors;
}

/* Returns one only at the next checked revolution marker. The caller copies
 * the completed points before feeding the marker's single point below. */
static uint8_t lidar_parser_feed(LidarParser *parser,uint8_t byte,uint32_t now)
{
    unsigned length,n,i;uint16_t checksum,first,last,span;uint8_t ready=0U;
    parser->bytes[parser->used++]=byte;
    for(;;) {
        if(parser->used<2U) return 0U;
        if(parser->bytes[0]!=0xaaU || parser->bytes[1]!=0x55U) goto drop;
        if(parser->used<10U) return 0U;
        n=parser->bytes[3];length=10U+3U*n;
        first=(uint16_t)(ss_get16(parser->bytes+4)>>1);
        last=(uint16_t)(ss_get16(parser->bytes+6)>>1);
        if(!n || first>=SS_LIDAR_ANGLE_TURN || last>=SS_LIDAR_ANGLE_TURN ||
           !(parser->bytes[4]&1U) || !(parser->bytes[6]&1U) ||
           ((parser->bytes[2]&1U) && n!=1U)) goto bad;
        if(parser->used<length) return 0U;
        checksum=(uint16_t)(0x55aaU^ss_get16(parser->bytes+2)^
                 ss_get16(parser->bytes+4)^ss_get16(parser->bytes+6));
        for(i=0;i<n;++i) checksum^=(uint16_t)(parser->bytes[10U+3U*i]^ss_get16(parser->bytes+11U+3U*i));
        if(checksum!=ss_get16(parser->bytes+8)) goto bad;
        if(parser->bytes[2]&1U) {
            ready=(uint8_t)(parser->synchronized && !parser->damaged && parser->count &&
                           now-parser->started>=50U && now-parser->started<=500U);
            parser->period=(uint16_t)(now-parser->started);
            ++parser->scanId;
            /* The checked marker stays in bytes until lidar_parser_next(). */
            return (uint8_t)(ready?1U:2U);
        }
        if(parser->synchronized) {
            span=(uint16_t)((last+SS_LIDAR_ANGLE_TURN-first)%SS_LIDAR_ANGLE_TURN);
            if(parser->count+n>SS_LIDAR_MAX_POINTS) {
                parser->damaged=1U;++parser->errors;
            } else for(i=0;i<n;++i) {
                SsLidarPoint *point=&parser->points[parser->count++];
                uint16_t packed=ss_get16(parser->bytes+11U+3U*i);
                point->angle=(uint16_t)((first+(n>1U?(uint32_t)span*i/(n-1U):0U))%SS_LIDAR_ANGLE_TURN);
                point->distance=(uint16_t)(packed>>2);point->flag=(uint8_t)(packed&3U);
                point->intensity=parser->bytes[10U+3U*i];
            }
        }
        parser->used=0U;return 0U;
bad:
        /* Do not publish a partial circle recovered inside a damaged packet. */
        parser->damaged=1U;parser->synchronized=0U;++parser->errors;
        parser->used=0U;return 0U;
drop:
        --parser->used;memmove(parser->bytes,parser->bytes+1,parser->used);
    }
}

static void lidar_parser_next(LidarParser *parser,uint32_t now)
{
    uint16_t packed=ss_get16(parser->bytes+11);
    parser->count=1U;parser->started=now;parser->synchronized=1U;parser->damaged=0U;
    parser->points[0].angle=(uint16_t)(ss_get16(parser->bytes+4)>>1);
    parser->points[0].distance=(uint16_t)(packed>>2);
    parser->points[0].flag=(uint8_t)(packed&3U);parser->points[0].intensity=parser->bytes[10];
    parser->used=0U;
}
#endif
