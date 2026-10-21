#include <assert.h>
#include <stdint.h>
#include <string.h>

static uint32_t flashWords[8];
static unsigned programs, erases, unlocked, failProgram, failErase, failSecond;
typedef enum { FLASH_COMPLETE, FLASH_ERROR } FLASH_Status;
#define FLASH_FLAG_EOP 1U
#define FLASH_FLAG_OPERR 2U
#define FLASH_FLAG_WRPERR 4U
#define FLASH_FLAG_PGAERR 8U
#define FLASH_FLAG_PGPERR 16U
#define FLASH_FLAG_PGSERR 32U
#define FLASH_Sector_10 10U
#define VoltageRange_3 3U
#define FLASH_ACR_DCEN 1U
#define DISABLE 0U
#define ENABLE 1U
static struct { unsigned ACR; } flashRegisters = {1U};
#define FLASH (&flashRegisters)
static void FLASH_DataCacheCmd(unsigned enabled) { FLASH->ACR = enabled; }
static void FLASH_DataCacheReset(void) { assert(!FLASH->ACR); }
#define GRIPPER_STORE_BASE ((uintptr_t)flashWords)
#define GRIPPER_STORE_WORDS 8U

static void FLASH_Unlock(void) { unlocked = 1U; }
static void FLASH_Lock(void) { unlocked = 0U; }
static void FLASH_ClearFlag(unsigned flags) { assert(flags == 63U); }
static FLASH_Status FLASH_EraseSector(unsigned sector, unsigned voltage)
{
    assert(unlocked && sector == 10U && voltage == 3U);
    if (failErase) return FLASH_ERROR;
    ++erases;
    memset(flashWords, 0xFF, sizeof(flashWords));
    return FLASH_COMPLETE;
}
static FLASH_Status FLASH_ProgramWord(uintptr_t address, uint32_t word)
{
    uint32_t *destination = (uint32_t *)address;
    assert(unlocked && destination >= flashWords && destination < flashWords + 8);
    assert(*destination == 0xFFFFFFFFU);
    if (failProgram || (failSecond && ((destination - flashWords) & 1))) return FLASH_ERROR;
    ++programs;
    *destination &= word;
    return FLASH_COMPLETE;
}
#include "../gripper_store.h"

int main(void)
{
    uint16_t open = 112, close = 60;
    memset(flashWords, 0xFF, sizeof(flashWords));
    assert(!gripperAnglesLoad(&open, &close) && open == 112 && close == 60);
    assert(gripperAnglesSave(120, 55) && !unlocked);
    assert(gripperAnglesLoad(&open, &close) && open == 120 && close == 55);
    unsigned before = programs;
    assert(gripperAnglesSave(120, 55) && programs == before);
    assert(!gripperAnglesSave(271, 60) && !gripperAnglesSave(0, 0));
    assert(!gripperAnglesSave(60, 60) && !gripperAnglesSave(59, 60));
    assert(programs == before);
    flashWords[2] = 0x4700FFFFU; /* Incomplete or corrupt record must be ignored. */
    assert(gripperAnglesLoad(&open, &close) && open == 120 && close == 55);
    failProgram = 1;
    assert(!gripperAnglesSave(125, 56) && !unlocked);
    assert(gripperAnglesLoad(&open, &close) && open == 120 && close == 55);
    failProgram = 0;
    assert(gripperAnglesSave(125, 56));
    failSecond = 1;
    assert(!gripperAnglesSave(130, 57) && !unlocked);
    assert(gripperAnglesLoad(&open, &close) && open == 125 && close == 56);
    failSecond = 0; failErase = 1;
    assert(!gripperAnglesSave(130, 57) && !unlocked);
    assert(gripperAnglesLoad(&open, &close) && open == 125 && close == 56);
    failErase = 0;
    assert(gripperAnglesSave(130, 57) && erases == 1 && !unlocked);
    open = close = 0; /* Simulate fresh RAM on reboot. */
    assert(gripperAnglesLoad(&open, &close) && open == 130 && close == 57);
    flashWords[1] ^= 1;
    assert(!gripperAnglesLoad(&open, &close));
    assert(gripperAnglesSave(270, 0));
    assert(gripperAnglesLoad(&open, &close) && open == 270 && close == 0);
    return 0;
}
