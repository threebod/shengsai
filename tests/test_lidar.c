#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include "../firmware/lidar_parser.h"

static unsigned packet(uint8_t *out,uint8_t ct,unsigned n,unsigned first,unsigned last)
{
    unsigned i;uint16_t check;
    out[0]=0xaa;out[1]=0x55;out[2]=ct;out[3]=(uint8_t)n;
    ss_put16(out+4,(uint16_t)((first<<1)|1U));ss_put16(out+6,(uint16_t)((last<<1)|1U));
    check=(uint16_t)(0x55aa^ss_get16(out+2)^ss_get16(out+4)^ss_get16(out+6));
    for(i=0;i<n;++i){
        out[10+3*i]=(uint8_t)(10+i);ss_put16(out+11+3*i,(uint16_t)(((1000+i)<<2)|(i&3)));
        check^=(uint16_t)(out[10+3*i]^ss_get16(out+11+3*i));
    }
    ss_put16(out+8,check);return 10+3*n;
}
static uint8_t feed(LidarParser *p,const uint8_t *bytes,unsigned n,uint32_t now)
{
    unsigned i;uint8_t result=0;
    for(i=0;i<n;++i){uint8_t r=lidar_parser_feed(p,bytes[i],now);if(r)result=r;}
    return result;
}
static void marker(LidarParser *p,uint32_t now,uint8_t expected)
{
    uint8_t bytes[13];unsigned n=packet(bytes,121,1,0,0);
    assert(feed(p,bytes,n,now)==expected);lidar_parser_next(p,now);
}
int main(void)
{
    LidarParser p;uint8_t bytes[775],data[128]={0};unsigned n,i;
    const uint8_t manual[]={0xaa,0x55,0x20,0x13,0x95,0xa9,0x63,0,0x5a,0xe8,
        1,8,0xb9,1,0xac,0xb8,1,0xf8,0xb6,1,0x88,0xb5,1,0x58,0xae,1,0xc0,0xab,
        1,4,0xaa,1,0x78,0xa9,1,0x34,0xa9,1,0xc4,0xad,1,0x34,0xaf,1,0x78,0xae,
        1,0x2c,0xad,1,2,0,0,0x62,0xad,1,0x60,0xad,1,0xf0,0xab,1,0x88,0xad,1,0xfe,0xad};
    const uint8_t descriptor[]={0xa5,0x5a,5,0,0,0x40,0x81,0x7f};
    lidar_parser_reset(&p);assert(!feed(&p,descriptor,sizeof(descriptor),0));
    marker(&p,100,2);
    assert(!feed(&p,manual,sizeof(manual),150));assert(!p.errors && p.count==20);
    assert(p.points[1].angle==(0xa995U>>1) && p.points[1].distance==(0xb908U>>2));
    assert(p.points[19].angle==(0x0063U>>1));
    n=packet(bytes,0,4,359*64,64);assert(!feed(&p,bytes,n,180));
    assert(p.points[20].angle==359*64 && p.points[23].angle==64);
    assert(p.points[22].angle<SS_LIDAR_ANGLE_TURN && p.points[22].flag==2);
    n=packet(bytes,0,1,90*64,90*64);assert(!feed(&p,bytes,n,200));assert(p.points[24].angle==90*64);
    n=packet(bytes,121,1,0,0);assert(feed(&p,bytes,n,267)==1 && p.count==25 && p.period==167);
    lidar_parser_next(&p,267);
    /* A checksum error invalidates a whole revolution; the next marker resyncs. */
    n=packet(bytes,0,4,0,100);bytes[10]^=1;assert(!feed(&p,bytes,n,300) && p.errors==1);
    marker(&p,434,2);marker(&p,601,1);
    lidar_parser_lost(&p);marker(&p,768,2);marker(&p,935,1);
    /* Reject timing gaps and overflowing scans rather than presenting partial data. */
    marker(&p,2000,2);
    n=packet(bytes,0,255,0,1000);
    for(i=0;i<5;++i)assert(!feed(&p,bytes,n,2020+i));
    assert(p.damaged && p.count<=SS_LIDAR_MAX_POINTS);marker(&p,2167,2);
    marker(&p,2168,2);
    /* Bad headers cannot grow the parser beyond the largest legal packet. */
    memset(bytes,0,sizeof(bytes));bytes[0]=0xaa;bytes[1]=0x55;
    assert(!feed(&p,bytes,sizeof(bytes),2200) && p.used<sizeof(p.bytes));
    marker(&p,2335,2);
    ss_put16(data+6,23);ss_put16(data+4,0);
    for(i=0;i<22;++i){SsLidarPoint pt={(uint16_t)(i*64),7161,100,3};ss_lidar_put_point(data+SS_LIDAR_HEADER+5*i,&pt);}
    assert(ss_lidar_chunk_valid(data,124));
    assert((ss_get16(data+SS_LIDAR_HEADER+2)>>2)==7161 && (ss_get16(data+SS_LIDAR_HEADER+2)&3)==3);
    assert(!ss_lidar_chunk_valid(data,123));ss_put16(data+4,2);assert(!ss_lidar_chunk_valid(data,124));
    ss_put16(data+4,0);ss_put16(data+SS_LIDAR_HEADER,23040);assert(!ss_lidar_chunk_valid(data,124));
    puts("PASS lidar: manual XOR vector, descriptor/CRC skip, angle wrap, flags, single point, damaged revolution, RX loss, timing and packet bounds");
    return 0;
}
