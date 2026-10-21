#include "can.h"
#include "delay.h"
#include "jog_can.h"

/* Local transport: do not change the competition project's shared CAN driver. */
__IO CAN_t can = {0};
volatile uint8_t jogCanFault;
static volatile uint8_t driveStatusExpected, driveStatusReady, driveStatus[4];

static volatile uint8_t replyReady[6][3];
static volatile int32_t replyValue[6][3];
static int replySlot(uint8_t fn) { return fn==0x3a?0:fn==0x33?1:fn==0x36?2:-1; }
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx; uint32_t id, value; int slot; unsigned i;
    while(CAN_MessagePending(CAN1,CAN_FIFO0)) {
        CAN_Receive(CAN1,CAN_FIFO0,&rx);
        id=rx.ExtId>>8;
        if(rx.IDE!=CAN_Id_Extended || rx.RTR!=CAN_RTR_Data || id<1 || id>6 ||
           rx.ExtId!=(id<<8) || rx.DLC<3 || rx.Data[rx.DLC-1]!=0x6b) continue;
        slot=replySlot(rx.Data[0]); if(slot<0) continue;
        if(slot==0 && rx.DLC==3) value=rx.Data[1];
        else if(slot>0 && rx.DLC==7) {
            value=0;for(i=2;i<6;++i) value=(value<<8)|rx.Data[i];
            if(value>0x7fffffffUL) { jogCanFault=1;continue; }
            if(rx.Data[1]) value=(uint32_t)(-(int32_t)value);
        } else continue;
        /* Fault bits remain latched until consumed by the main loop. */
        if(slot==0 && replyReady[id-1][0] &&
           (!(replyValue[id-1][0]&1) || (replyValue[id-1][0]&12))) continue;
        replyValue[id-1][slot]=(int32_t)value;replyReady[id-1][slot]=1;
        if(slot==0 && id<=4 && (driveStatusExpected&(1U<<(id-1)))) {
            uint8_t bit=(uint8_t)(1U<<(id-1));
            if(!(driveStatusReady&bit) || ((driveStatus[id-1]&1) && !(driveStatus[id-1]&12)))
                driveStatus[id-1]=(uint8_t)value;
            driveStatusReady|=bit;
        }
    }
}
void jogCanClearReply(uint8_t id, uint8_t fn) {
    int s=replySlot(fn);if(id<1 || id>6 || s<0) return;
    __disable_irq();replyReady[id-1][s]=0;__enable_irq();
}
uint8_t jogCanTakeReply(uint8_t id,uint8_t fn,int32_t *value) {
    int s=replySlot(fn);uint8_t ready;
    if(id<1 || id>6 || s<0) return 0;
    __disable_irq();ready=replyReady[id-1][s];
    if(ready) { *value=replyValue[id-1][s];replyReady[id-1][s]=0; }
    __enable_irq();return ready;
}
void jogCanResetDriveStatus(void)
{
    __disable_irq();
    driveStatusExpected = driveStatusReady = 0U;
    __enable_irq();
}

void jogCanExpectDriveStatus(uint8_t motorId)
{
    if (motorId < 1U || motorId > 4U) return;
    __disable_irq();
    driveStatusExpected |= (uint8_t)(1U << (motorId - 1U));
    __enable_irq();
}

uint8_t jogCanTakeDriveStatus(uint8_t motorId, uint8_t *status)
{
    uint8_t bit, ready;
    if (motorId < 1U || motorId > 4U) return 0U;
    bit = (uint8_t)(1U << (motorId - 1U));
    __disable_irq();
    ready = driveStatusReady & bit;
    if (ready) {
        *status = driveStatus[motorId - 1U];
        driveStatusReady &= (uint8_t)~bit;
    }
    __enable_irq();
    return ready != 0U;
}

uint8_t can_GetRxFlag(void) { return 0U; }

void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    CanTxMsg tx;
    uint8_t offset = 2U, packet = 0U, mailbox, i;
    uint16_t wait;
    if (len < 3U || len > 16U) { jogCanFault = 1U; return; }
    while (offset < len) {
        tx.StdId = 0U;
        tx.ExtId = ((uint32_t)cmd[0] << 8) | packet++;
        tx.IDE = CAN_Id_Extended;
        tx.RTR = CAN_RTR_Data;
        tx.Data[0] = cmd[1];
        tx.DLC = 1U;
        for (i = 1U; i < 8U && offset < len; ++i) {
            tx.Data[i] = cmd[offset++];
            ++tx.DLC;
        }
        mailbox = CAN_Transmit(CAN1, &tx);
        if (mailbox == CAN_TxStatus_NoMailBox) {
            jogCanFault = 1U;
            return;
        }
        for (wait = 0U; wait < 50U; ++wait) {
            if (CAN_TransmitStatus(CAN1, mailbox) == CAN_TxStatus_Ok) break;
            delay_us(100U);
        }
        if (wait == 50U) {
            CAN_CancelTransmit(CAN1, mailbox);
            jogCanFault = 1U;
            return;
        }
    }
}
