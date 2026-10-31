#ifndef SHENGSAI_LIDAR_RUNTIME_H
#define SHENGSAI_LIDAR_RUNTIME_H
#include "lidar_parser.h"

#define LIDAR_RX_SIZE 4096U
static volatile uint8_t lidarRx[LIDAR_RX_SIZE],lidarRxLost;
static volatile uint16_t lidarRxHead,lidarRxTail;
static volatile uint8_t lidarTx[8],lidarTxHead,lidarTxTail;
static uint8_t lidarEnabled;
static uint16_t lidarToken,lidarSnapshotCount,lidarSnapshotOffset,lidarSnapshotId,lidarSnapshotPeriod;
static uint32_t lidarCapturedAt,lidarSentAt;
static LidarParser lidarParser;
static SsLidarPoint lidarSnapshot[SS_LIDAR_MAX_POINTS];

void USART1_IRQHandler(void)
{
    uint32_t status=USART1->SR;
    if(status&(USART_SR_RXNE|USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)) {
        uint8_t byte=(uint8_t)USART1->DR;
        uint16_t next=(uint16_t)((lidarRxHead+1U)&(LIDAR_RX_SIZE-1U));
        if(status&(USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)) lidarRxLost=1U;
        if(next==lidarRxTail) lidarRxLost=1U;
        else {lidarRx[lidarRxHead]=byte;__DMB();lidarRxHead=next;}
    }
    if(USART_GetITStatus(USART1,USART_IT_TXE)!=RESET) {
        if(lidarTxTail!=lidarTxHead) {
            USART_SendData(USART1,lidarTx[lidarTxTail]);
            lidarTxTail=(uint8_t)((lidarTxTail+1U)&7U);
        }
        if(lidarTxTail==lidarTxHead) USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
    }
}

static void lidarUartInit(void)
{
    GPIO_InitTypeDef gpio;USART_InitTypeDef uart;NVIC_InitTypeDef nvic;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
    GPIO_StructInit(&gpio);gpio.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_10;
    gpio.GPIO_Mode=GPIO_Mode_AF;gpio.GPIO_OType=GPIO_OType_PP;
    gpio.GPIO_Speed=GPIO_Speed_50MHz;gpio.GPIO_PuPd=GPIO_PuPd_UP;GPIO_Init(GPIOA,&gpio);
    USART_StructInit(&uart);uart.USART_BaudRate=230400U;
    uart.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;USART_Init(USART1,&uart);
    nvic.NVIC_IRQChannel=USART1_IRQn;nvic.NVIC_IRQChannelPreemptionPriority=2U;
    nvic.NVIC_IRQChannelSubPriority=0U;nvic.NVIC_IRQChannelCmd=ENABLE;NVIC_Init(&nvic);
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);USART_Cmd(USART1,ENABLE);
}

static uint8_t lidarSetEnabled(uint8_t enabled,uint16_t token)
{
    uint8_t head=lidarTxHead;
    if((uint8_t)((lidarTxTail-head-1U)&7U)<2U) return 0U;
    lidarTx[head]=0xa5U;head=(uint8_t)((head+1U)&7U);
    lidarTx[head]=enabled?0x60U:0x65U;head=(uint8_t)((head+1U)&7U);
    __DMB();lidarTxHead=head;USART_ITConfig(USART1,USART_IT_TXE,ENABLE);
    lidarEnabled=enabled;lidarToken=token;
    lidarSnapshotCount=lidarSnapshotOffset=0U;
    __disable_irq();lidarRxTail=lidarRxHead;lidarRxLost=0U;__enable_irq();
    lidar_parser_reset(&lidarParser);
    lidarCapturedAt=clockMs-1000U;lidarSentAt=clockMs;
    return 1U;
}

static void serviceLidar(void)
{
    unsigned budget=512U;
    if(lidarEnabled && (hostEStop || !hostHeartbeatActive ||
       clockMs-hostHeartbeatStamp>HOST_HEARTBEAT_TIMEOUT_MS)) {
        lidarSetEnabled(0U,0U);
    }
    if(lidarRxLost) {
        __disable_irq();lidarRxTail=lidarRxHead;lidarRxLost=0U;__enable_irq();
        lidar_parser_lost(&lidarParser);
    }
    if(lidarSnapshotCount && clockMs-lidarCapturedAt>2000U) lidarSnapshotCount=0U;
    while(budget-- && lidarRxTail!=lidarRxHead) {
        uint8_t byte=lidarRx[lidarRxTail],result;
        lidarRxTail=(uint16_t)((lidarRxTail+1U)&(LIDAR_RX_SIZE-1U));
        if(!lidarEnabled) continue;
        result=lidar_parser_feed(&lidarParser,byte,clockMs);
        if(result==1U && !lidarSnapshotCount && clockMs-lidarCapturedAt>=1000U) {
            lidarSnapshotCount=lidarParser.count;lidarSnapshotOffset=0U;
            lidarSnapshotId=lidarParser.scanId;lidarSnapshotPeriod=lidarParser.period;
            memcpy(lidarSnapshot,lidarParser.points,lidarSnapshotCount*sizeof(lidarSnapshot[0]));
            lidarCapturedAt=clockMs;
        }
        if(result) lidar_parser_next(&lidarParser,clockMs);
    }
    /* Only a small optional frame every 20ms, after host request servicing.
     * Never accumulate point-cloud packets behind control responses. */
    if(lidarEnabled && lidarSnapshotCount && !hostReplyPending &&
       clockMs-lidarSentAt>=20U && hostTxFree(&hostTx)>=HOST_TX_CAPACITY-SS_MAX_FRAME) {
        SsFrame frame;unsigned i,n=lidarSnapshotCount-lidarSnapshotOffset;
        if(n>SS_LIDAR_CHUNK_POINTS) n=SS_LIDAR_CHUNK_POINTS;
        frame.flags=SS_EVENT;frame.sequence=0U;frame.command=SS_LIDAR_SCAN;
        frame.length=(uint16_t)(SS_LIDAR_HEADER+n*SS_LIDAR_POINT_BYTES);
        ss_put16(frame.data,lidarToken);ss_put16(frame.data+2,lidarSnapshotId);
        ss_put16(frame.data+4,lidarSnapshotOffset);ss_put16(frame.data+6,lidarSnapshotCount);
        ss_put16(frame.data+8,lidarSnapshotPeriod);ss_put16(frame.data+10,lidarParser.errors);
        ss_put16(frame.data+12,(uint16_t)(clockMs-lidarCapturedAt));
        for(i=0;i<n;++i) ss_lidar_put_point(frame.data+SS_LIDAR_HEADER+i*SS_LIDAR_POINT_BYTES,
                                         &lidarSnapshot[lidarSnapshotOffset+i]);
        if(hostSendOptional(&frame)) {
            lidarSentAt=clockMs;lidarSnapshotOffset=(uint16_t)(lidarSnapshotOffset+n);
            if(lidarSnapshotOffset==lidarSnapshotCount) lidarSnapshotCount=0U;
        }
    }
}
#endif
