#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "can.h"
#include "../firmware/jog_can.h"
static CanRxMsg incoming[16];static unsigned head,tail,txs;static uint8_t failTx;
extern volatile uint8_t jogCanFault;
void CAN1_RX0_IRQHandler(void);
uint8_t CAN_MessagePending(int b,int f){(void)b;(void)f;return tail<head;}
void CAN_Receive(int b,int f,CanRxMsg *r){(void)b;(void)f;*r=incoming[tail++];}
uint8_t CAN_Transmit(int b,CanTxMsg *tx){(void)b;assert(tx->IDE==CAN_Id_Extended && tx->DLC<=8);++txs;return failTx?CAN_TxStatus_NoMailBox:0;}
uint8_t CAN_TransmitStatus(int b,uint8_t box){(void)b;(void)box;return CAN_TxStatus_Ok;}
void CAN_CancelTransmit(int b,uint8_t box){(void)b;(void)box;}
void delay_us(uint32_t t){(void)t;}
static void status(unsigned id,uint8_t v){CanRxMsg *r=&incoming[head++];memset(r,0,sizeof(*r));r->IDE=CAN_Id_Extended;r->ExtId=id<<8;r->DLC=3;r->Data[0]=0x3a;r->Data[1]=v;r->Data[2]=0x6b;}
static void position(unsigned id,uint8_t fn,int32_t value){
    CanRxMsg *r=&incoming[head++];uint32_t v=value<0?(uint32_t)-value:(uint32_t)value;
    memset(r,0,sizeof(*r));r->IDE=CAN_Id_Extended;r->ExtId=id<<8;r->DLC=7;r->Data[0]=fn;r->Data[1]=value<0;
    r->Data[2]=v>>24;r->Data[3]=v>>16;r->Data[4]=v>>8;r->Data[5]=v;r->Data[6]=0x6b;
}
int main(void){
    int32_t value;uint8_t s;uint8_t command[13]={5,0xfd,0,0,30,50,0,0,0,10,0,1,0x6b};
    status(5,3);status(6,1);position(5,0x33,-65536);position(6,0x36,32768);CAN1_RX0_IRQHandler();
    assert(tail==head);assert(jogCanTakeReply(5,0x3a,&value) && value==3);
    assert(jogCanTakeReply(6,0x3a,&value) && value==1);
    assert(jogCanTakeReply(5,0x33,&value) && value==-65536);
    assert(jogCanTakeReply(6,0x36,&value) && value==32768);
    assert(!jogCanTakeReply(5,0x33,&value));
    head=tail=0;status(5,7);status(5,3);CAN1_RX0_IRQHandler();assert(jogCanTakeReply(5,0x3a,&value) && value==7);
    head=tail=0;status(6,3);incoming[0].Data[2]=0;CAN1_RX0_IRQHandler();assert(!jogCanTakeReply(6,0x3a,&value));
    head=tail=0;jogCanExpectDriveStatus(2);status(2,3);CAN1_RX0_IRQHandler();assert(jogCanTakeDriveStatus(2,&s) && s==3);
    can_SendCmd(command,13);assert(txs==2 && !jogCanFault);failTx=1;can_SendCmd(command,13);assert(jogCanFault);
    puts("PASS CAN FIFO drain, per-motor/per-type mailboxes, signed positions, fault latching, invalid checksum and TX failure");return 0;
}
