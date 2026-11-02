#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
static uint32_t words[20];static unsigned writes,failAt,locked=1;
typedef int FLASH_Status;
#define FLASH_COMPLETE 0
#define FLASH_FLAG_EOP 1
#define FLASH_FLAG_OPERR 2
#define FLASH_FLAG_WRPERR 4
#define FLASH_FLAG_PGAERR 8
#define FLASH_FLAG_PGPERR 16
#define FLASH_FLAG_PGSERR 32
#define FLASH_ACR_DCEN 1
#define DISABLE 0
#define ENABLE 1
static struct { unsigned ACR; } registers={1};
#define FLASH (&registers)
#define SCALE_STORE_BASE ((uintptr_t)words)
#define SCALE_STORE_WORDS 20
static void FLASH_Unlock(void){locked=0;}
static void FLASH_Lock(void){locked=1;}
static void FLASH_ClearFlag(unsigned f){assert(f==63);}
static void FLASH_DataCacheCmd(unsigned enabled){FLASH->ACR=enabled;}
static void FLASH_DataCacheReset(void){assert(!FLASH->ACR);}
static int FLASH_ProgramWord(uintptr_t addr,uint32_t v){
    uint32_t *p=(uint32_t *)addr;assert(!locked && p>=words && p<words+20);
    if(failAt && writes+1==failAt)return 1;
    assert(*p==0xffffffff);*p&=v;++writes;return 0;
}
#include "../firmware/scale_store.h"
static void legacyRecord(uint16_t f,uint16_t r) {
    uint8_t bytes[8];memset(words,255,sizeof(words));
    words[0]=0x56534301UL;words[1]=f|((uint32_t)r<<16);
    ss_put32(bytes,words[0]);ss_put32(bytes+4,words[1]);
    words[2]=ss_crc(bytes,8);words[3]=~words[0];
}
int main(void){
    uint16_t f=1111,r=1143;unsigned before;
    legacyRecord(1111,1143);before=writes;
    assert(scaleLoad(&f,&r) && f==556 && r==572 && writes==before);
    assert(scaleLoad(&f,&r) && f==556 && r==572 && writes==before);
    assert(scaleSave(600,650) && words[4]==0x56534302UL);
    assert(scaleLoad(&f,&r) && f==600 && r==650);
    legacyRecord(50,2000);assert(scaleLoad(&f,&r) && f==25 && r==1000);
    words[2]^=1;assert(!scaleLoad(&f,&r));
    memset(words,255,sizeof(words));assert(!scaleLoad(&f,&r));
    assert(!scaleSave(0,1000));assert(scaleSave(1111,1143) && locked);before=writes;
    assert(scaleSave(1111,1143) && writes==before);
    failAt=writes+4;assert(!scaleSave(1000,1200) && locked);failAt=0;
    f=r=0;assert(scaleLoad(&f,&r) && f==1111 && r==1143);
    assert(scaleSave(1200,1300));f=r=0;assert(scaleLoad(&f,&r) && f==1200 && r==1300);
    assert(scaleSave(1300,1400));assert(scaleSave(1400,1500));assert(!scaleSave(1500,1600));
    assert(scaleLoad(&f,&r) && f==1400 && r==1500);
    puts("PASS scale validation, commit-last interrupted-write recovery, readback, duplicate write suppression, full log preserves prior setting");return 0;
}
