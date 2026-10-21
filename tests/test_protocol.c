#include <assert.h>
#include <stdio.h>
#include "../shared/protocol.h"
int main(void){
    SsFrame q={0},r;SsParser p={0};uint8_t bytes[SS_MAX_FRAME],bad[SS_MAX_FRAME];unsigned i,n,seen=0;
    assert(ss_crc((const uint8_t *)"123456789",9)==0x29b1);
    q.flags=SS_REQUEST;q.command=SS_SET_RINGS;q.sequence=255;q.length=12;
    ss_put16(q.data,(uint16_t)-320);ss_put16(q.data+2,2637);n=ss_encode(&q,bytes);
    assert(n==22 && bytes[0]==0xaa && bytes[1]==0x55 && (int16_t)ss_get16(bytes+8)==-320);
    for(i=0;i<n-1;++i)assert(!ss_feed(&p,bytes[i],&r));
    assert(ss_feed(&p,bytes[n-1],&r));assert(r.command==q.command && r.sequence==255 && !memcmp(r.data,q.data,12));
    memcpy(bad,bytes,n);bad[10]^=1;
    for(i=0;i<n;++i)assert(!ss_feed(&p,bad[i],&r));
    for(i=0;i<n*2;++i)seen+=ss_feed(&p,bytes[i%n],&r);
    assert(seen==2);
    p.count=0;q.length=128;memset(q.data,0xaa,128);q.data[127]=0x55;n=ss_encode(&q,bytes);
    for(i=0;i<n;++i)seen=ss_feed(&p,bytes[i],&r);
    assert(seen && r.length==128);q.length=129;assert(!ss_encode(&q,bytes));
    p.count=0;{uint8_t header[]={0xaa,0x55,1,1,1,1,255,255};for(i=0;i<8;++i)assert(!ss_feed(&p,header[i],&r));}
    q.length=0;n=ss_encode(&q,bytes);for(i=0;i<n;++i)seen=ss_feed(&p,bytes[i],&r);assert(seen);
    puts("PASS protocol CRC, signed fields, split/coalesced frames, corrupt frame recovery and bounds");return 0;
}
