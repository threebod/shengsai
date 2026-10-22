#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "can.h"
#include "../firmware/jog_can.h"
/* Compile the production command encoder against the host CAN stub. */
#define __CAN_H
#include "../firmware/vendor/can_ZDT/Emm_V5.c"
static CanRxMsg incoming[16];static unsigned head,tail,txs;static uint8_t failTx;
static CanTxMsg transmitted[32];
extern volatile uint8_t jogCanFault;
void CAN1_RX0_IRQHandler(void);
uint8_t CAN_MessagePending(int b,int f){(void)b;(void)f;return tail<head;}
void CAN_Receive(int b,int f,CanRxMsg *r){(void)b;(void)f;*r=incoming[tail++];}
uint8_t CAN_Transmit(int b,CanTxMsg *tx){(void)b;assert(tx->IDE==CAN_Id_Extended && tx->DLC<=8);assert(txs<32);transmitted[txs++]=*tx;return failTx?CAN_TxStatus_NoMailBox:0;}
uint8_t CAN_TransmitStatus(int b,uint8_t box){(void)b;(void)box;return CAN_TxStatus_Ok;}
void CAN_CancelTransmit(int b,uint8_t box){(void)b;(void)box;}
void delay_us(uint32_t t){(void)t;}
static void status(unsigned id,uint8_t v){CanRxMsg *r=&incoming[head++];memset(r,0,sizeof(*r));r->IDE=CAN_Id_Extended;r->ExtId=id<<8;r->DLC=3;r->Data[0]=0x3a;r->Data[1]=v;r->Data[2]=0x6b;}
static void position(unsigned id,uint8_t fn,int32_t value){
    CanRxMsg *r=&incoming[head++];uint32_t v=value<0?(uint32_t)-value:(uint32_t)value;
    memset(r,0,sizeof(*r));r->IDE=CAN_Id_Extended;r->ExtId=id<<8;r->DLC=7;r->Data[0]=fn;r->Data[1]=value<0;
    r->Data[2]=v>>24;r->Data[3]=v>>16;r->Data[4]=v>>8;r->Data[5]=v;r->Data[6]=0x6b;
}
static void expectFrame(unsigned index,uint32_t id,const uint8_t *data,uint8_t len){
    assert(index<txs && transmitted[index].ExtId==id && transmitted[index].RTR==CAN_RTR_Data);
    assert(transmitted[index].DLC==len && memcmp(transmitted[index].Data,data,len)==0);
}
int main(void){
    int32_t value;uint8_t s;uint8_t command[13]={5,0xfd,0,0,30,50,0,0,0,10,0,1,0x6b};
    /* Golden bytes from STD_Emm_CAN position/synchronous-motion examples. */
    const uint8_t horizontalFirst[]={0xfd,1,0x03,0xe8,200,1,2,3};
    const uint8_t horizontalLast[]={0xfd,4,0,1,0x6b};
    const uint8_t liftFirst[]={0xfd,0,0x07,0xd0,240,0,0,0x0c};
    const uint8_t liftLast[]={0xfd,0x80,0,1,0x6b};
    const uint8_t shortFirst[]={0xfd,1,0,30,50,0,0,0};
    const uint8_t shortLast[]={0xfd,1,0,1,0x6b};
    const uint8_t sync[]={0xff,0x66,0x6b};
    const uint8_t target[]={0x33,0x6b},actual[]={0x36,0x6b},flag[]={0x3a,0x6b};
    Emm_V5_Pos_Control(6,1,1000,200,0x01020304, false,true);
    expectFrame(0,0x600,horizontalFirst,8);expectFrame(1,0x601,horizontalLast,5);
    Emm_V5_Pos_Control(5,0,2000,240,3200,false,true);
    expectFrame(2,0x500,liftFirst,8);expectFrame(3,0x501,liftLast,5);
    Emm_V5_Pos_Control(6,1,30,50,1,false,true);
    expectFrame(4,0x600,shortFirst,8);expectFrame(5,0x601,shortLast,5);
    Emm_V5_Synchronous_motion(0);expectFrame(6,0,sync,3);
    Emm_V5_Read_Sys_Params(6,S_TPOS);expectFrame(7,0x600,target,2);
    Emm_V5_Read_Sys_Params(6,S_CPOS);expectFrame(8,0x600,actual,2);
    Emm_V5_Read_Sys_Params(6,S_FLAG);expectFrame(9,0x600,flag,2);
    assert(txs==10 && !jogCanFault);txs=0;
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
