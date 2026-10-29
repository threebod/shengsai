#ifndef SHENGSAI_PROTOCOL_H
#define SHENGSAI_PROTOCOL_H
#include <stdint.h>
#include <string.h>
#include "obstacle_regions.h"
#define SS_ROUTE_REVISION 1U
#define SS_MAX_PAYLOAD 128
#define SS_MAX_FRAME 138
#define SS_LATERAL_TEST_MIN_MM 100
#define SS_LATERAL_TEST_MAX_MM 2000
#define SS_STRAIGHT_TEST_MM 1680
#define SS_STRAIGHT_TEST_RPM 240
enum { SS_REQUEST=1, SS_RESPONSE=2, SS_EVENT=4, SS_ERROR=8 };
enum {
    SS_HELLO=1, SS_STATUS, SS_HEARTBEAT, SS_STOP, SS_ESTOP, SS_CLEAR,
    SS_GET_RINGS=0x10, SS_SET_RINGS, SS_GET_SCALE, SS_APPLY_SCALE, SS_SAVE_SCALE,
    SS_GET_LOG=0x17,
    SS_SERVO=0x20, SS_AUX, SS_DRIVE, SS_LATERAL_TEST, SS_STRAIGHT_TEST, SS_TURN_TEST,
    SS_PREPARE=0x30, SS_MISSION, SS_ROUTE, SS_ACTION,
    SS_SAMPLE=0x40, SS_VISION_JOG, SS_ALIGN,
    SS_LOG=0x80, SS_STATE
};
enum { SS_ACTION_AXIS6_TEST=3 };
typedef struct { uint8_t flags, sequence, command; uint16_t length; uint8_t data[SS_MAX_PAYLOAD]; } SsFrame;
typedef struct { uint8_t data[SS_MAX_FRAME]; uint16_t count; } SsParser;
static uint16_t ss_get16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1]<<8)); }
static uint8_t ss_route_request_valid(const uint8_t *data, unsigned length) {
    return (uint8_t)(length==5U && data[0]>=1U && data[0]<=2U &&
        ss_obstacle_regions_valid(ss_get16(data+1)) && ss_get16(data+3)>=10U && ss_get16(data+3)<=250U);
}
static void ss_put16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void ss_put32(uint8_t *p, uint32_t v) { ss_put16(p,(uint16_t)v); ss_put16(p+2,(uint16_t)(v>>16)); }
static uint32_t ss_get32(const uint8_t *p) { return ss_get16(p) | ((uint32_t)ss_get16(p+2)<<16); }
static uint16_t ss_crc(const uint8_t *p, unsigned n) {
    uint16_t c=0xffff; unsigned i;
    while(n--) { c^=(uint16_t)*p++<<8; for(i=0;i<8;++i) c=(uint16_t)((c&0x8000)?(c<<1)^0x1021:c<<1); }
    return c;
}
static unsigned ss_encode(const SsFrame *f, uint8_t *out) {
    unsigned n=f->length;
    if(n>SS_MAX_PAYLOAD) return 0;
    out[0]=0xaa;out[1]=0x55;out[2]=1;out[3]=f->flags;out[4]=f->sequence;out[5]=f->command;
    ss_put16(out+6,(uint16_t)n);memcpy(out+8,f->data,n);ss_put16(out+8+n,ss_crc(out+2,n+6));return n+10;
}
/* Recover by one byte after bad length/CRC, preserving any embedded frame header. */
static int ss_feed(SsParser *p, uint8_t b, SsFrame *out) {
    unsigned n, total;
    if(p->count==SS_MAX_FRAME) { memmove(p->data,p->data+1,--p->count); }
    p->data[p->count++]=b;
    for(;;) {
        if(p->count<2) return 0;
        if(p->data[0]!=0xaa || p->data[1]!=0x55) goto drop;
        if(p->count<8) return 0;
        n=ss_get16(p->data+6);total=n+10;
        if(n>SS_MAX_PAYLOAD || p->data[2]!=1 ||
           !(p->data[3]==1 || p->data[3]==2 || p->data[3]==4 || p->data[3]==10)) goto drop;
        if(p->count<total) return 0;
        if(ss_get16(p->data+8+n)!=ss_crc(p->data+2,n+6)) goto drop;
        out->flags=p->data[3];out->sequence=p->data[4];out->command=p->data[5];out->length=(uint16_t)n;
        memcpy(out->data,p->data+8,n);p->count=(uint16_t)(p->count-total);
        memmove(p->data,p->data+total,p->count);return 1;
drop:   memmove(p->data,p->data+1,--p->count);
    }
}
#endif
