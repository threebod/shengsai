#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../shared/protocol.h"

enum {UART5=5,USART_IT_TXE=0x80,DISABLE=0,ENABLE=1};
static uint8_t txEnabled,injectInterrupt;
static uint8_t wire[65536];static unsigned wireLength,irqCalls;
static void publishBarrier(void);
#define HOST_TX_BARRIER() publishBarrier()
static void USART_ITConfig(int uart,int interrupt,int enabled){assert(uart==UART5 && interrupt==USART_IT_TXE);txEnabled=(uint8_t)enabled;}
static void USART_SendData(int uart,uint8_t byte){assert(uart==UART5 && wireLength<sizeof(wire));wire[wireLength++]=byte;}
#include "../firmware/host_tx.h"
static void publishBarrier(void){if(injectInterrupt){injectInterrupt=0;++irqCalls;hostTxInterrupt();}}

#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
static SsParser hostParser;
static volatile uint8_t hostRx[512];static volatile uint16_t hostHead,hostTail;
static volatile uint8_t hostOverflow;
typedef struct {uint8_t valid;uint32_t at;SsFrame request,response;} HostCached;
static HostCached hostCache[16];static uint8_t hostCacheIndex;
static uint8_t hostOperationFailed,hostCancelled,jogCanFault,hostEStop,hostDriveActive,motionMode;
static uint32_t clockMs,hostDriveAt,hostStateAt;
static unsigned executed,stops,servoStops,otherServices,errors;
static uint8_t busy,fillDuringExecute;
static uint8_t filler[HOST_TX_CAPACITY];
static void stopServoMotion(void){++servoStops;}
static void stopAllMotors(void){++stops;busy=motionMode=0;}
static void serialSendString(const char *text){assert(strstr(text,"RX queue overflow"));++errors;}
static uint8_t hostBusy(void){return busy;}
static void serviceObstacleHostStart(void){++otherServices;}
static void hostSnapshot(SsFrame *r){r->length=21;memset(r->data,0,21);}
static void hostExecute(const SsFrame *q,SsFrame *r){
    unsigned i;++executed;memset(r,0,sizeof(*r));r->flags=SS_RESPONSE;
    r->command=q->command;r->sequence=q->sequence;r->length=q->command==SS_GET_LOG?128:1;
    for(i=1;i<r->length;++i)r->data[i]=(uint8_t)(q->sequence+i);
    if(q->command==SS_ACTION){busy=1;motionMode=3;}
    if(q->command==SS_STOP || q->command==SS_ESTOP)stopAllMotors();
    if(fillDuringExecute){assert(hostTxPush(&hostTx,filler,hostTxFree(&hostTx),0));txEnabled=1;}
}
#include "../firmware/host_service.h"

static void reset(void){
    memset(&hostTx,0,sizeof(hostTx));memset(&hostParser,0,sizeof(hostParser));memset(hostCache,0,sizeof(hostCache));
    hostHead=hostTail=hostOverflow=0;hostCacheIndex=0;wireLength=irqCalls=0;txEnabled=injectInterrupt=0;
    clockMs=hostDriveAt=hostStateAt=0;executed=stops=servoStops=otherServices=errors=0;
    hostOperationFailed=hostCancelled=jogCanFault=hostEStop=hostDriveActive=motionMode=busy=fillDuringExecute=0;
    hostReplyPending=hostOperationActive=hostOperationSequence=hostOperationCommand=0;
}
static void receive(uint8_t sequence,uint8_t command,unsigned length){
    SsFrame q={0};uint8_t bytes[SS_MAX_FRAME];unsigned i,n;
    q.flags=SS_REQUEST;q.command=command;q.sequence=sequence;q.length=(uint16_t)length;
    n=ss_encode(&q,bytes);
    for(i=0;i<n;++i){uint16_t next=(uint16_t)((hostHead+1U)%sizeof(hostRx));assert(next!=hostTail);hostRx[hostHead]=bytes[i];hostHead=next;}
}
static void drain(void){unsigned limit=0;while(txEnabled){++irqCalls;hostTxInterrupt();assert(++limit<HOST_TX_CAPACITY+2);}}
static unsigned decode(SsFrame *frames,unsigned capacity){
    SsParser parser={0};SsFrame frame;unsigned i,n=0;
    for(i=0;i<wireLength;++i)if(ss_feed(&parser,wire[i],&frame)){assert(n<capacity);frames[n++]=frame;}
    assert(!parser.count);return n;
}
int main(void){
    SsFrame a={0},b={0},decoded[32];unsigned i,j,n,head;
    uint8_t expected[SS_MAX_FRAME*2],encoded[SS_MAX_FRAME];unsigned first,second;
    reset();a.flags=b.flags=SS_EVENT;a.command=SS_LOG;b.command=SS_STATE;
    a.length=128;b.length=21;
    for(i=0;i<128;++i)a.data[i]=(uint8_t)i;
    a.sequence=1;b.sequence=2;
    first=ss_encode(&a,expected);second=ss_encode(&b,expected+first);
    /* Interrupt before the head publication cannot see a partial new packet. */
    injectInterrupt=1;assert(hostSend(&a) && !wireLength && txEnabled);
    assert(hostSend(&b));drain();assert(wireLength==first+second && !memcmp(wire,expected,wireLength));
    assert(!txEnabled);hostTxInterrupt();assert(!txEnabled);assert(hostSend(&b) && txEnabled);drain();
    /* Exercise packet boundaries and the byte-ring wrap over many cycles. */
    reset();
    for(i=0;i<1000;++i){a.sequence=(uint8_t)i;assert(hostSend(&a));drain();
        assert(decode(decoded,32)==1 && decoded[0].sequence==(uint8_t)i);wireLength=0;}
    reset();
    while(hostSendOptional(&a)){}
    assert(hostTxFree(&hostTx)>=2U*SS_MAX_FRAME);head=hostTx.head;
    assert(!hostSendOptional(&a) && hostTx.head==head);
    assert(hostSend(&a));b.length=1;assert(hostSend(&b));drain();assert(decode(decoded,32)>=2);
    reset();a.length=129;assert(!hostSend(&a) && !txEnabled && !hostTx.head);a.length=128;
    assert(hostTxPush(&hostTx,filler,HOST_TX_CAPACITY-1,0));head=hostTx.head;
    assert(!hostSend(&a) && head==hostTx.head);
    /* Garbage RX bytes and coalesced requests are bounded per service tick. */
    reset();for(i=0;i<150;++i)hostRx[hostHead++]=0;
    hostService();assert(hostTail==64 && otherServices==1 && !executed);
    hostService();assert(hostTail==128 && otherServices==2);
    reset();for(i=1;i<=5;++i)receive((uint8_t)i,SS_STATUS,0);
    hostService();assert(executed==2 && hostTail==20 && otherServices==1);
    hostService();assert(executed==4);hostService();assert(executed==5);
    drain();assert(decode(decoded,32)==5);
    /* Backpressure leaves requests unexecuted until a complete reply fits.
     * Other main-loop services keep receiving a slot while the wire drains. */
    reset();for(i=1;i<=10;++i)receive((uint8_t)i,SS_GET_LOG,12);
    for(i=0;i<10;++i)hostService();assert(executed==7 && otherServices==10 && hostTail!=hostHead);
    for(i=0;i<2000 && (hostHead!=hostTail || txEnabled);++i){
        hostService();if(txEnabled)hostTxInterrupt();
    }
    drain();assert(hostHead==hostTail && executed==10 && decode(decoded,32)==10);
    for(i=0;i<10;++i){assert(decoded[i].flags==SS_RESPONSE && decoded[i].sequence==i+1 && decoded[i].length==128);
        for(j=1;j<128;++j)assert(decoded[i].data[j]==(uint8_t)(i+1+j));}
    wireLength=0;receive(10,SS_GET_LOG,12);hostService();drain();assert(executed==10 && decode(decoded,32)==1);
    /* A failed enqueue retains the actual reply; execution never repeats. */
    reset();fillDuringExecute=1;receive(70,SS_ACTION,1);hostService();
    assert(executed==1 && hostReplyPending && hostOperationActive);
    busy=motionMode=0;hostService();assert(hostReplyPending && hostOperationActive && executed==1);
    drain();wireLength=0;fillDuringExecute=0;hostService();drain();n=decode(decoded,32);
    assert(n==2 && decoded[0].flags==SS_RESPONSE && decoded[1].flags==SS_EVENT && decoded[1].sequence==70);
    assert(!hostReplyPending && !hostOperationActive && executed==1);
    /* An unsent completion cannot be overwritten by the next motion command. */
    reset();receive(80,SS_ACTION,1);hostService();drain();wireLength=0;busy=motionMode=0;
    assert(hostTxPush(&hostTx,filler,hostTxFree(&hostTx),0));txEnabled=1;receive(81,SS_ACTION,1);
    head=hostTail;hostService();assert(executed==1 && hostTail==head && hostOperationActive);
    drain();wireLength=0;hostService();drain();n=decode(decoded,32);
    assert(n==2 && decoded[0].flags==SS_EVENT && decoded[0].sequence==80 && decoded[1].sequence==81);
    busy=motionMode=0;hostService();drain();
    reset();hostOverflow=1;receive(90,SS_ACTION,1);hostService();
    assert(stops==1 && servoStops==1 && errors==1 && hostTail==hostHead && !executed && hostOperationFailed);
    reset();hostDriveActive=motionMode=busy=1;clockMs=301;
    assert(hostTxPush(&hostTx,filler,hostTxFree(&hostTx),0));hostService();
    assert(stops==1 && !hostDriveActive && otherServices==1);
    /* Wire framing remains the original binary protocol. */
    assert(ss_encode(&a,encoded)==SS_MAX_FRAME);
    puts("PASS UART5 TXE: atomic packets, IRQ restart, wrap, reserved replies, bounded RX, backpressure/cache, retained completion, overflow/watchdog");
    return 0;
}
