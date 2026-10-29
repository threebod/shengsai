#ifndef SHENGSAI_HOST_TX_QUEUE_H
#define SHENGSAI_HOST_TX_QUEUE_H
#include <stdint.h>

#define HOST_TX_CAPACITY 1024U
#ifndef HOST_TX_BARRIER
#define HOST_TX_BARRIER() ((void)0)
#endif

/* Single main-loop producer, single UART5 interrupt consumer. The producer
 * publishes the new head only after copying the entire encoded packet. */
typedef struct {
    volatile uint16_t head,tail;
    uint8_t bytes[HOST_TX_CAPACITY];
} HostTxQueue;

static uint16_t hostTxFree(const HostTxQueue *queue)
{
    return (uint16_t)((queue->tail-queue->head-1U)&(HOST_TX_CAPACITY-1U));
}
static uint8_t hostTxPush(HostTxQueue *queue,const uint8_t *bytes,
                          unsigned length,unsigned reserve)
{
    unsigned i;uint16_t head=queue->head,free=hostTxFree(queue);
    if(!length || reserve>free || length>free-reserve) return 0U;
    for(i=0;i<length;++i) {
        queue->bytes[head]=bytes[i];head=(uint16_t)((head+1U)&(HOST_TX_CAPACITY-1U));
    }
    HOST_TX_BARRIER();queue->head=head;
    return 1U;
}
static uint8_t hostTxPop(HostTxQueue *queue,uint8_t *byte)
{
    uint16_t tail=queue->tail;
    if(tail==queue->head) return 0U;
    HOST_TX_BARRIER();*byte=queue->bytes[tail];
    queue->tail=(uint16_t)((tail+1U)&(HOST_TX_CAPACITY-1U));
    return 1U;
}
#endif
