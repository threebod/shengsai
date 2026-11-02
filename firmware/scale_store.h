#ifndef SHENGSAI_SCALE_STORE_H
#define SHENGSAI_SCALE_STORE_H
#include <stdint.h>
#include "../shared/protocol.h"
/* Sector 8 is separate from ring/gripper/obstacle sectors 9/10/11. */
#ifndef SCALE_STORE_BASE
#define SCALE_STORE_BASE 0x08080000UL
#define SCALE_STORE_WORDS (0x20000U/4U)
#endif
static uint8_t scaleValid(uint16_t f,uint16_t r) { return f>=25 && f<=2000 && r>=25 && r<=2000; }
static uint8_t scaleRecord(uint32_t i,uint16_t *f,uint16_t *r) {
    const volatile uint32_t *w=(const volatile uint32_t *)SCALE_STORE_BASE;
    uint8_t bytes[8];uint32_t data=w[i+1];
    if((w[i]!=0x56534301UL && w[i]!=0x56534302UL) || w[i+3]!=~w[i]) return 0;
    ss_put32(bytes,w[i]);ss_put32(bytes+4,data);
    if(w[i+2]!=ss_crc(bytes,8)) return 0;
    *f=(uint16_t)data;*r=(uint16_t)(data>>16);
    if(!scaleValid(*f,*r) || (w[i]==0x56534301UL && (*f<50U || *r<50U))) return 0;
    /* Old records use one 320x240 pixel; adapt only at the storage boundary. */
    if(w[i]==0x56534301UL) {*f=(uint16_t)((*f+1U)/2U);*r=(uint16_t)((*r+1U)/2U);}
    return scaleValid(*f,*r);
}
static uint8_t scaleLoad(uint16_t *f,uint16_t *r) {
    uint32_t i;uint16_t a,b;uint8_t found=0;
    for(i=0;i+3<SCALE_STORE_WORDS;i+=4) if(scaleRecord(i,&a,&b)) { *f=a;*r=b;found=1; }
    return found;
}
static uint8_t scaleSave(uint16_t f,uint16_t r) {
    volatile const uint32_t *w=(volatile const uint32_t *)SCALE_STORE_BASE;
    uint32_t i,j,next=0,data[4];uint16_t a,b;uint8_t bytes[8];FLASH_Status s=FLASH_COMPLETE;
    if(!scaleValid(f,r)) return 0;
    if(scaleLoad(&a,&b) && a==f && b==r) return 1;
    for(i=0;i+3<SCALE_STORE_WORDS;i+=4)
        for(j=0;j<4;++j) if(w[i+j]!=0xffffffffUL) next=i+4;
    data[0]=0x56534302UL;data[1]=f|((uint32_t)r<<16);
    ss_put32(bytes,data[0]);ss_put32(bytes+4,data[1]);data[2]=ss_crc(bytes,8);data[3]=~data[0];
    /* Do not erase the last valid setting automatically when the log is full. */
    if(next+3>=SCALE_STORE_WORDS) return 0;
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
    for(i=0;i<4 && s==FLASH_COMPLETE;++i) s=FLASH_ProgramWord(SCALE_STORE_BASE+(next+i)*4,data[i]);
    FLASH_Lock();
    if(FLASH->ACR&FLASH_ACR_DCEN) { FLASH_DataCacheCmd(DISABLE);FLASH_DataCacheReset();FLASH_DataCacheCmd(ENABLE); }
    return s==FLASH_COMPLETE && scaleRecord(next,&a,&b) && a==f && b==r;
}
#endif
