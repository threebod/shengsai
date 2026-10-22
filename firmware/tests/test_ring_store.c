#ifdef NDEBUG
#undef NDEBUG
#endif
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

static void legacyRecord(uint32_t index, const RingPose poses[3])
{
    uint8_t i;
    flashWords[index] = 0x524E4701UL;
    for (i = 0U; i < 3U; ++i)
        flashWords[index + i + 1U] = (uint16_t)poses[i].horizontalDmm |
                                     ((uint32_t)poses[i].turretDdeg << 16);
    flashWords[index + 4U] = ringRecordCheck(flashWords + index);
    flashWords[index + 5U] = ~0x524E4701UL;
}

static void testLegacyTurretMigration(void)
{
    RingPose legacy[3] = {{-45,3078},{-320,2637},{430,2300}};
    const RingPose shifted[3] = {{-45,2958},{-320,2517},{430,2180}};
    RingPose loaded[3], edited[3];
    unsigned before;
    memset(flashWords, 0xFF, sizeof(flashWords));
    programs = erases = 0U;
    legacyRecord(0U, legacy);
    assert(ringPosesLoad(loaded) && memcmp(loaded, shifted, sizeof(loaded)) == 0);
    assert(programs == 0U && erases == 0U && (flashWords[1] >> 16) == 3078U);
    /* Reloading a legacy record must not subtract the offset a second time. */
    assert(ringPosesLoad(loaded) && memcmp(loaded, shifted, sizeof(loaded)) == 0);
    /* Explicit save upgrades even when the effective angles did not change. */
    assert(ringPosesSave(loaded) && programs == 6U && flashWords[6] == 0x524E4702UL);
    assert(ringPosesLoad(loaded) && memcmp(loaded, shifted, sizeof(loaded)) == 0);
    before = programs;
    assert(ringPosesSave(loaded) && programs == before);
    memcpy(edited, shifted, sizeof(edited));
    edited[0].turretDdeg = 0U;
    assert(ringPosesSave(edited) && ringPosesLoad(loaded));
    assert(memcmp(loaded, edited, sizeof(loaded)) == 0);

    memset(flashWords, 0xFF, sizeof(flashWords));
    legacyRecord(0U, legacy);
    failSecond = 1U;
    assert(!ringPosesSave(shifted));
    failSecond = 0U;
    assert(ringPosesLoad(loaded) && memcmp(loaded, shifted, sizeof(loaded)) == 0);
    assert(ringPosesSave(shifted));
    flashWords[13] ^= 1U; /* Corrupt the newer record; fall back to legacy + offset. */
    assert(ringPosesLoad(loaded) && memcmp(loaded, shifted, sizeof(loaded)) == 0);

    memset(flashWords, 0xFF, sizeof(flashWords));
    legacy[0].turretDdeg = 119U;
    legacyRecord(0U, legacy);
    assert(!ringPosesLoad(loaded)); /* Do not wrap a negative converted angle. */
    legacy[0].turretDdeg = 3601U;
    legacyRecord(0U, legacy);
    assert(!ringPosesLoad(loaded)); /* Validate the original range as well. */
    legacy[0].turretDdeg = 120U;
    legacy[1].turretDdeg = 3600U;
    legacyRecord(0U, legacy);
    assert(ringPosesLoad(loaded) && loaded[0].turretDdeg == 0U && loaded[1].turretDdeg == 3480U);
}

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
    testLegacyTurretMigration();
    return 0;
}
