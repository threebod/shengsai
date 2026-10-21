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
int main(void){
    uint16_t f=1111,r=1143;unsigned before;
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
