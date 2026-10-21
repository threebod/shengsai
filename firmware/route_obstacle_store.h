#ifndef ROUTE_OBSTACLE_STORE_H
#define ROUTE_OBSTACLE_STORE_H

#include <stdint.h>

/* STM32F407 sector 11 is reserved by the Keil IROM limit. The caller
 * supplies the STM32 flash driver, or its simulated equivalent in tests. */
#ifndef ROUTE_OBSTACLE_STORE_BASE
#define ROUTE_OBSTACLE_STORE_BASE 0x080E0000UL
#define ROUTE_OBSTACLE_STORE_WORDS (0x20000U / 4U)
#endif

static uint8_t routeObstacleRecordValid(uint32_t word)
{
    uint8_t position = (uint8_t)(word & 3U);
    return word == (0x4F425300UL | ((uint32_t)(position ^ 3U) << 4) | position);
}

static uint8_t routeObstacleLoad(void)
{
    volatile const uint32_t *words = (volatile const uint32_t *)ROUTE_OBSTACLE_STORE_BASE;
    uint32_t i;
    uint8_t position = 0U;
    for (i = 0U; i < ROUTE_OBSTACLE_STORE_WORDS; ++i)
        if (routeObstacleRecordValid(words[i])) position = (uint8_t)(words[i] & 3U);
    return position;
}

static uint8_t routeObstacleSave(uint8_t position)
{
    volatile const uint32_t *words = (volatile const uint32_t *)ROUTE_OBSTACLE_STORE_BASE;
    uint32_t i, next = 0U, last = 0xFFFFFFFFUL, record;
    FLASH_Status status;
    if (position > 3U) return 0U;
    for (i = 0U; i < ROUTE_OBSTACLE_STORE_WORDS; ++i) {
        if (words[i] != 0xFFFFFFFFUL) next = i + 1U;
        if (routeObstacleRecordValid(words[i])) last = words[i];
    }
    record = 0x4F425300UL | ((uint32_t)(position ^ 3U) << 4) | position;
    if (last == record) return 1U;
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    if (next == ROUTE_OBSTACLE_STORE_WORDS) {
        status = FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
        if (status != FLASH_COMPLETE) { FLASH_Lock(); return 0U; }
        next = 0U;
    }
    status = FLASH_ProgramWord(ROUTE_OBSTACLE_STORE_BASE + next * 4U, record);
    FLASH_Lock();
    /* The scan may have cached erased words before they were programmed. */
    if (FLASH->ACR & FLASH_ACR_DCEN) {
        FLASH_DataCacheCmd(DISABLE);
        FLASH_DataCacheReset();
        FLASH_DataCacheCmd(ENABLE);
    }
    return status == FLASH_COMPLETE && words[next] == record;
}

#endif
