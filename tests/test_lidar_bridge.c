#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include "../shared/lidar.h"

/* Peripheral stand-ins: exercise the real IRQ rings and bridge without motors. */
typedef struct {uint32_t SR,DR;} TestUart;
typedef struct {unsigned GPIO_Pin,GPIO_Mode,GPIO_OType,GPIO_Speed,GPIO_PuPd;} GPIO_InitTypeDef;
typedef struct {unsigned USART_BaudRate,USART_Mode;} USART_InitTypeDef;
typedef struct {unsigned NVIC_IRQChannel,NVIC_IRQChannelPreemptionPriority,NVIC_IRQChannelSubPriority,NVIC_IRQChannelCmd;} NVIC_InitTypeDef;
static TestUart testUartPeripheral;
#define USART1 (&testUartPeripheral)
enum {RESET=0,DISABLE=0,ENABLE=1,USART_SR_RXNE=32,USART_SR_ORE=8,USART_SR_FE=2,
      USART_SR_NE=4,USART_SR_PE=1,USART_IT_TXE=128,USART_IT_RXNE=32,
      RCC_AHB1Periph_GPIOA=0,RCC_APB2Periph_USART1=0,GPIOA=0,GPIO_PinSource9=9,
      GPIO_PinSource10=10,GPIO_AF_USART1=7,GPIO_Pin_9=512,GPIO_Pin_10=1024,
      GPIO_Mode_AF=2,GPIO_OType_PP=0,GPIO_Speed_50MHz=0,GPIO_PuPd_UP=1,
      USART_Mode_Rx=1,USART_Mode_Tx=2,USART1_IRQn=37};
#define __DMB() ((void)0)
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
#define RCC_AHB1PeriphClockCmd(...) ((void)0)
#define RCC_APB2PeriphClockCmd(...) ((void)0)
#define GPIO_PinAFConfig(...) ((void)0)
#define GPIO_Init(...) ((void)0)
#define USART_Init(...) ((void)0)
#define NVIC_Init(...) ((void)0)
#define USART_Cmd(...) ((void)0)
#define GPIO_StructInit(p) memset(p,0,sizeof(*(p)))
#define USART_StructInit(p) memset(p,0,sizeof(*(p)))
static uint8_t txEnabled,wire[64];static unsigned wireCount;
static void USART_ITConfig(TestUart *port,unsigned interrupt,unsigned enabled)
{assert(port==USART1);if(interrupt==USART_IT_TXE)txEnabled=(uint8_t)enabled;}
static unsigned USART_GetITStatus(TestUart *port,unsigned interrupt)
{assert(port==USART1 && interrupt==USART_IT_TXE);return txEnabled;}
static void USART_SendData(TestUart *port,uint8_t byte)
{assert(port==USART1 && wireCount<sizeof(wire));wire[wireCount++]=byte;}
static uint32_t clockMs,hostHeartbeatStamp;
static uint8_t hostEStop,hostHeartbeatActive,hostReplyPending;
#define HOST_HEARTBEAT_TIMEOUT_MS 1000U
#define HOST_TX_CAPACITY 1024U
static unsigned hostTx,freeBytes=1023,frameCount;
static SsFrame frames[64];
static unsigned hostTxFree(const unsigned *queue){assert(queue==&hostTx);return freeBytes;}
static uint8_t hostSendOptional(const SsFrame *frame)
{assert(frameCount<64);frames[frameCount++]=*frame;return 1;}
#include "../firmware/lidar_runtime.h"

static void receive(const uint8_t *bytes,unsigned count)
{
    unsigned i;
    for(i=0;i<count;++i){testUartPeripheral.SR=USART_SR_RXNE;testUartPeripheral.DR=bytes[i];USART1_IRQHandler();testUartPeripheral.SR=0;}
    serviceLidar();
}
static unsigned makePacket(uint8_t *out,uint8_t ct,unsigned n)
{
    unsigned i;uint16_t check;
    out[0]=0xaa;out[1]=0x55;out[2]=ct;out[3]=(uint8_t)n;
    ss_put16(out+4,1);ss_put16(out+6,1);
    check=(uint16_t)(0x55aa^ss_get16(out+2));
    for(i=0;i<n;++i){out[10+i*3]=12;ss_put16(out+11+i*3,4000);check^=12^4000;}
    ss_put16(out+8,check);return 10+3*n;
}
int main(void)
{
    uint8_t bytes[32];unsigned n,i;
    lidarUartInit();hostHeartbeatActive=1;
    assert(lidarSetEnabled(1,77));
    while(txEnabled){testUartPeripheral.SR=0;USART1_IRQHandler();}
    assert(wireCount==2 && wire[0]==0xa5 && wire[1]==0x60);
    freeBytes=0;clockMs=100;n=makePacket(bytes,121,1);receive(bytes,n);
    clockMs=150;n=makePacket(bytes,0,4);receive(bytes,n);
    clockMs=267;n=makePacket(bytes,121,1);receive(bytes,n);
    assert(lidarSnapshotCount==5 && !frameCount);
    freeBytes=1023;hostReplyPending=1;clockMs=300;serviceLidar();assert(!frameCount);
    hostReplyPending=0;serviceLidar();assert(frameCount==1 && !lidarSnapshotCount);
    assert(frames[0].command==SS_LIDAR_SCAN && ss_lidar_chunk_valid(frames[0].data,frames[0].length));
    assert(ss_get16(frames[0].data)==77 && ss_get16(frames[0].data+6)==5 &&
           ss_get16(frames[0].data+8)==167 && ss_get16(frames[0].data+12)==33);
    /* Whole-frame pacing and backpressure leave host replies first. */
    lidarSnapshotCount=1024;lidarSnapshotOffset=0;lidarCapturedAt=clockMs;
    for(i=0;i<1024;++i){lidarSnapshot[i].angle=(uint16_t)i;lidarSnapshot[i].distance=1000;}
    serviceLidar();assert(frameCount==1);
    clockMs+=19;serviceLidar();assert(frameCount==1);
    clockMs++;freeBytes=800;serviceLidar();assert(frameCount==1);
    freeBytes=1023;serviceLidar();assert(frameCount==2 && lidarSnapshotOffset==22);
    clockMs+=20;serviceLidar();assert(frameCount==3 && ss_get16(frames[2].data+4)==22);
    hostHeartbeatStamp=clockMs;clockMs=lidarCapturedAt+2001;hostHeartbeatStamp=clockMs;
    serviceLidar();assert(!lidarSnapshotCount && frameCount==3);
    /* RX overflow drops the partial revolution; loss is visible in diagnostics. */
    lidarRxLost=1;serviceLidar();assert(lidarParser.damaged && lidarParser.errors==1);
    assert(lidarSetEnabled(1,88));assert(lidarToken==88 && !lidarSnapshotCount && !lidarParser.synchronized);
    clockMs=hostHeartbeatStamp+1001;serviceLidar();assert(!lidarEnabled);
    while(txEnabled){testUartPeripheral.SR=0;USART1_IRQHandler();}
    assert(wire[wireCount-2]==0xa5 && wire[wireCount-1]==0x65);
    hostHeartbeatStamp=clockMs;assert(lidarSetEnabled(1,99));hostEStop=1;serviceLidar();assert(!lidarEnabled);
    puts("PASS lidar bridge: USART1 commands, IRQ RX, snapshots, 20ms pacing, reserved replies, stale discard, RX overflow, sessions, heartbeat and estop");
    return 0;
}
