#ifndef SHENGSAI_LIDAR_H
#define SHENGSAI_LIDAR_H
#include "protocol.h"

#define SS_LIDAR_MAX_POINTS 1024U
#define SS_LIDAR_HEADER 14U
#define SS_LIDAR_POINT_BYTES 5U
#define SS_LIDAR_CHUNK_POINTS 22U
#define SS_LIDAR_ANGLE_TURN 23040U

typedef struct {
    uint16_t angle, distance;
    uint8_t intensity, flag;
} SsLidarPoint;

static void ss_lidar_put_point(uint8_t *out,const SsLidarPoint *point)
{
    ss_put16(out,point->angle);
    ss_put16(out+2,(uint16_t)((point->distance<<2)|(point->flag&3U)));
    out[4]=point->intensity;
}

static uint8_t ss_lidar_chunk_valid(const uint8_t *data,unsigned length)
{
    unsigned count,total,offset,i;
    if(length<SS_LIDAR_HEADER || (length-SS_LIDAR_HEADER)%SS_LIDAR_POINT_BYTES) return 0U;
    count=(length-SS_LIDAR_HEADER)/SS_LIDAR_POINT_BYTES;
    total=ss_get16(data+6);offset=ss_get16(data+4);
    if(!total || total>SS_LIDAR_MAX_POINTS || !count || count>SS_LIDAR_CHUNK_POINTS ||
       offset>=total || count>total-offset) return 0U;
    for(i=0;i<count;++i)
        if(ss_get16(data+SS_LIDAR_HEADER+i*SS_LIDAR_POINT_BYTES)>=SS_LIDAR_ANGLE_TURN) return 0U;
    return 1U;
}
#endif
