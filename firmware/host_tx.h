#ifndef SHENGSAI_HOST_TX_H
#define SHENGSAI_HOST_TX_H
#include "../shared/protocol.h"
#include "host_tx_queue.h"

static HostTxQueue hostTx;

/* Log/state frames cannot consume space reserved for a maximum response and
 * a completion event. A rejected enqueue has sent no part of its packet. */
static uint8_t hostQueueFrame(const SsFrame *frame,uint8_t optional)
{
    uint8_t bytes[SS_MAX_FRAME];unsigned n=ss_encode(frame,bytes);
    if(!hostTxPush(&hostTx,bytes,n,optional?2U*SS_MAX_FRAME:0U)) return 0U;
    USART_ITConfig(UART5,USART_IT_TXE,ENABLE);
    return 1U;
}
static uint8_t hostSend(const SsFrame *frame) { return hostQueueFrame(frame,0U); }
static uint8_t hostSendOptional(const SsFrame *frame) { return hostQueueFrame(frame,1U); }

/* Only transfer a byte. Formatting, CRC and all request processing stay in the
 * main loop. Empty disables TXE; the next complete enqueue enables it again. */
static void hostTxInterrupt(void)
{
    uint8_t byte;
    if(hostTxPop(&hostTx,&byte)) USART_SendData(UART5,byte);
    if(hostTx.head==hostTx.tail) USART_ITConfig(UART5,USART_IT_TXE,DISABLE);
}
#endif
