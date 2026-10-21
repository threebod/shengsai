#ifndef MECANUM_JOG_GRIPPER_STORE_H
#define MECANUM_JOG_GRIPPER_STORE_H

#include <stdint.h>

/* Sector 10 is reserved by the Keil IROM limit; sector 11 stores obstacles. */
#ifndef GRIPPER_STORE_BASE
#define GRIPPER_STORE_BASE 0x080C0000UL
#define GRIPPER_STORE_WORDS (0x20000U / 4U)
#endif

static uint8_t gripperAnglesValid(uint16_t open, uint16_t close)
{
    return open <= 270U && close < open;
}

static uint8_t gripperRecordValid(uint32_t data, uint32_t check)
{
    return check == ~data && (data & 0xFFFC0000UL) == 0x47000000UL &&
           gripperAnglesValid((uint16_t)((data >> 9) & 511U), (uint16_t)(data & 511U));
}

static uint8_t gripperAnglesLoad(uint16_t *open, uint16_t *close)
{
    volatile const uint32_t *words = (volatile const uint32_t *)GRIPPER_STORE_BASE;
    uint32_t i, data;
    uint8_t found = 0U;
    for (i = 0U; i + 1U < GRIPPER_STORE_WORDS; i += 2U) {
        data = words[i];
        if (!gripperRecordValid(data, words[i + 1U])) continue;
        *open = (uint16_t)((data >> 9) & 511U);
        *close = (uint16_t)(data & 511U);
        found = 1U;
    }
    return found;
}

static uint8_t gripperAnglesSave(uint16_t open, uint16_t close)
{
    volatile const uint32_t *words = (volatile const uint32_t *)GRIPPER_STORE_BASE;
    uint32_t i, next = 0U, record;
    uint16_t previousOpen = 0U, previousClose = 0U;
    FLASH_Status status;
    if (!gripperAnglesValid(open, close)) return 0U;
    if (gripperAnglesLoad(&previousOpen, &previousClose) &&
        previousOpen == open && previousClose == close) return 1U;
    for (i = 0U; i + 1U < GRIPPER_STORE_WORDS; i += 2U)
        if (words[i] != 0xFFFFFFFFUL || words[i + 1U] != 0xFFFFFFFFUL) next = i + 2U;
    record = 0x47000000UL | ((uint32_t)open << 9) | close;
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    if (next == GRIPPER_STORE_WORDS) {
        status = FLASH_EraseSector(FLASH_Sector_10, VoltageRange_3);
        if (status != FLASH_COMPLETE) { FLASH_Lock(); return 0U; }
        next = 0U;
    }
    status = FLASH_ProgramWord(GRIPPER_STORE_BASE + next * 4U, record);
    if (status == FLASH_COMPLETE)
        status = FLASH_ProgramWord(GRIPPER_STORE_BASE + (next + 1U) * 4U, ~record);
    FLASH_Lock();
    if (FLASH->ACR & FLASH_ACR_DCEN) {
        FLASH_DataCacheCmd(DISABLE);
        FLASH_DataCacheReset();
        FLASH_DataCacheCmd(ENABLE);
    }
    return status == FLASH_COMPLETE && gripperRecordValid(words[next], words[next + 1U]);
}

#endif
