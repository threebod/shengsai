#ifndef MECANUM_JOG_RING_STORE_H
#define MECANUM_JOG_RING_STORE_H

#include "ring_pose.h"

/* Independent sector 9; sectors 10/11 retain gripper/obstacle settings. */
#ifndef RING_STORE_BASE
#define RING_STORE_BASE 0x080A0000UL
#define RING_STORE_WORDS (0x20000U / 4U)
#endif

#define RING_RECORD_LEGACY 0x524E4701UL
#define RING_RECORD_CURRENT 0x524E4702UL

static uint32_t ringRecordCheck(const uint32_t data[4])
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint8_t i, bit;
    for (i = 0U; i < 4U; ++i) {
        crc ^= data[i];
        for (bit = 0U; bit < 32U; ++bit)
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
    }
    return crc;
}

static uint8_t ringRecordRead(uint32_t index, RingPose poses[3])
{
    volatile const uint32_t *words = (volatile const uint32_t *)RING_STORE_BASE;
    uint32_t data[4];
    uint8_t i;
    for (i = 0U; i < 4U; ++i) data[i] = words[index + i];
    if ((data[0] != RING_RECORD_LEGACY && data[0] != RING_RECORD_CURRENT) ||
        words[index + 5U] != ~data[0] ||
        words[index + 4U] != ringRecordCheck(data)) return 0U;
    for (i = 0U; i < 3U; ++i) {
        poses[i].horizontalDmm = (int16_t)(data[i + 1U] & 65535U);
        poses[i].turretDdeg = (uint16_t)(data[i + 1U] >> 16);
    }
    /* Both record formats contain absolute angles; load them unchanged. */
    return ringPosesValid(poses);
}

static uint8_t ringPosesLoad(RingPose poses[3])
{
    RingPose candidate[3];
    uint32_t i;
    uint8_t found = 0U;
    for (i = 0U; i + 5U < RING_STORE_WORDS; i += 6U)
        if (ringRecordRead(i, candidate)) {
            memcpy(poses, candidate, sizeof(candidate));
            found = 1U;
        }
    return found;
}

static uint8_t ringPosesSave(const RingPose poses[3])
{
    volatile const uint32_t *words = (volatile const uint32_t *)RING_STORE_BASE;
    RingPose previous[3];
    uint32_t i, j, next = 0U, data[6];
    FLASH_Status status = FLASH_COMPLETE;
    if (!ringPosesValid(poses)) return 0U;
    if (ringPosesLoad(previous) && memcmp(previous, poses, sizeof(previous)) == 0) return 1U;
    for (i = 0U; i + 5U < RING_STORE_WORDS; i += 6U)
        for (j = 0U; j < 6U; ++j)
            if (words[i + j] != 0xFFFFFFFFUL) next = i + 6U;
    data[0] = RING_RECORD_CURRENT;
    for (i = 0U; i < 3U; ++i)
        data[i + 1U] = (uint16_t)poses[i].horizontalDmm | ((uint32_t)poses[i].turretDdeg << 16);
    data[4] = ringRecordCheck(data);
    data[5] = ~data[0]; /* Commit marker is written last. */
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    if (next + 5U >= RING_STORE_WORDS) {
        status = FLASH_EraseSector(FLASH_Sector_9, VoltageRange_3);
        next = 0U;
    }
    for (i = 0U; i < 6U && status == FLASH_COMPLETE; ++i)
        status = FLASH_ProgramWord(RING_STORE_BASE + (next + i) * 4U, data[i]);
    FLASH_Lock();
    if (FLASH->ACR & FLASH_ACR_DCEN) {
        FLASH_DataCacheCmd(DISABLE);
        FLASH_DataCacheReset();
        FLASH_DataCacheCmd(ENABLE);
    }
    return status == FLASH_COMPLETE && ringRecordRead(next, previous) &&
           memcmp(previous, poses, sizeof(previous)) == 0;
}

#endif
