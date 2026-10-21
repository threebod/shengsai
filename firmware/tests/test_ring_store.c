#include <assert.h>
#include <stdint.h>
#include <string.h>

static uint32_t flashWords[20];
static unsigned programs, erases, unlocked, failProgram, failErase, failSecond;
typedef enum { FLASH_COMPLETE, FLASH_ERROR } FLASH_Status;
#define FLASH_FLAG_EOP 1U
#define FLASH_FLAG_OPERR 2U
#define FLASH_FLAG_WRPERR 4U
#define FLASH_FLAG_PGAERR 8U
#define FLASH_FLAG_PGPERR 16U
#define FLASH_FLAG_PGSERR 32U
#define FLASH_Sector_9 9U
#define VoltageRange_3 3U
#define FLASH_ACR_DCEN 1U
#define DISABLE 0U
#define ENABLE 1U
static struct { unsigned ACR; } flashRegisters = {1U};
#define FLASH (&flashRegisters)
static void FLASH_DataCacheCmd(unsigned enabled) { FLASH->ACR = enabled; }
static void FLASH_DataCacheReset(void) { assert(!FLASH->ACR); }
#define RING_STORE_BASE ((uintptr_t)flashWords)
#define RING_STORE_WORDS 20U

static void FLASH_Unlock(void) { unlocked = 1U; }
static void FLASH_Lock(void) { unlocked = 0U; }
static void FLASH_ClearFlag(unsigned flags) { assert(flags == 63U); }
static FLASH_Status FLASH_EraseSector(unsigned sector, unsigned voltage)
{
    assert(unlocked && sector == 9U && voltage == 3U);
    if (failErase) return FLASH_ERROR;
    ++erases;
    memset(flashWords, 0xFF, sizeof(flashWords));
    return FLASH_COMPLETE;
}
static FLASH_Status FLASH_ProgramWord(uintptr_t address, uint32_t word)
{
    uint32_t *destination = (uint32_t *)address;
    assert(unlocked && destination >= flashWords && destination < flashWords + 20);
    assert(*destination == 0xFFFFFFFFU);
    if (failProgram || (failSecond && ((destination - flashWords) % 6 == 5))) return FLASH_ERROR;
    ++programs;
    *destination &= word;
    return FLASH_COMPLETE;
}
#include "../ring_store.h"
int main(void)
{
    const RingPose first[3]={{-55,3038},{-320,2612},{430,2280}};
    RingPose next[3]={{-85,3001},{-350,2600},{400,2300}}, loaded[3]={{0,0},{0,0},{0,0}};
    memset(flashWords,0xFF,sizeof(flashWords));
    assert(!ringPosesLoad(loaded) && loaded[0].horizontalDmm==0);
    assert(ringPosesSave(first) && !unlocked);
    assert(ringPosesLoad(loaded) && memcmp(loaded,first,sizeof(first))==0);
    unsigned before=programs;
    assert(ringPosesSave(first) && programs==before);
    /* An interrupted record must not replace the preceding complete record. */
    failSecond=1;assert(!ringPosesSave(next) && !unlocked);failSecond=0;
    assert(ringPosesLoad(loaded) && memcmp(loaded,first,sizeof(first))==0);
    failProgram=1;assert(!ringPosesSave(next) && !unlocked);failProgram=0;
    assert(ringPosesSave(next));
    next[0].horizontalDmm=-90;
    failErase=1;assert(!ringPosesSave(next) && !unlocked);failErase=0;
    assert(ringPosesLoad(loaded) && loaded[0].horizontalDmm==-85);
    assert(ringPosesSave(next) && erases==1);
    memset(loaded,0,sizeof(loaded));
    assert(ringPosesLoad(loaded) && memcmp(loaded,next,sizeof(next))==0);
    flashWords[1]^=1U;
    assert(!ringPosesLoad(loaded));
    next[0].horizontalDmm=-1221;assert(!ringPosesSave(next));
    next[0].horizontalDmm=851;assert(!ringPosesSave(next));
    next[0].horizontalDmm=-1220;next[2].turretDdeg=3601;assert(!ringPosesSave(next));
    next[2].turretDdeg=3600;next[1].horizontalDmm=850;
    assert(ringPosesSave(next) && ringPosesLoad(loaded));
    assert(memcmp(loaded,next,sizeof(next))==0);
    return 0;
}
