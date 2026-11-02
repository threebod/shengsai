#ifndef MECANUM_JOG_RING_STORE_H
#define MECANUM_JOG_RING_STORE_H

#include "ring_pose.h"
#include "placement_heights.h"

/* Independent sector 9; sectors 10/11 retain gripper/obstacle settings. */
#ifndef RING_STORE_BASE
#define RING_STORE_BASE 0x080A0000UL
#define RING_STORE_WORDS (0x20000U / 4U)
#endif

#define RING_RECORD_LEGACY 0x524E4701UL
#define RING_RECORD_CURRENT 0x524E4702UL
#define RAW_PICK_RECORD 0x50494301UL
#define PLACEMENT_HEIGHTS_RECORD 0x504C4801UL

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

static uint8_t ringRecordDataRead(uint32_t index, uint32_t data[6])
{
    volatile const uint32_t *words = (volatile const uint32_t *)RING_STORE_BASE;
    uint8_t i;
    for (i = 0U; i < 6U; ++i) data[i] = words[index + i];
    return (data[0] == RING_RECORD_LEGACY || data[0] == RING_RECORD_CURRENT ||
            data[0] == PLACEMENT_HEIGHTS_RECORD || data[0] == RAW_PICK_RECORD) &&
           data[5] == ~data[0] && data[4] == ringRecordCheck(data);
}

static uint8_t poseRecordRead(uint32_t index, RingPose poses[3], uint32_t magic)
{
    uint32_t data[6];
    uint8_t i;
    if (!ringRecordDataRead(index, data) ||
        (magic == RAW_PICK_RECORD ? data[0] != magic :
         (data[0] != RING_RECORD_LEGACY && data[0] != RING_RECORD_CURRENT))) return 0U;
    for (i = 0U; i < 3U; ++i) {
        poses[i].horizontalDmm = (int16_t)(data[i + 1U] & 65535U);
        poses[i].turretDdeg = (uint16_t)(data[i + 1U] >> 16);
    }
    /* Both record formats contain absolute angles; load them unchanged. */
    return ringPosesValid(poses);
}

static uint8_t poseSettingsLoad(RingPose poses[3], uint32_t magic)
{
    RingPose candidate[3];
    uint32_t i;
    uint8_t found = 0U;
    for (i = 0U; i + 5U < RING_STORE_WORDS; i += 6U)
        if (poseRecordRead(i, candidate, magic)) {
            memcpy(poses, candidate, sizeof(candidate));
            found = 1U;
        }
    return found;
}

static uint8_t placementHeightsRead(uint32_t index, PlacementHeights *heights)
{
    uint32_t data[6];
    if (!ringRecordDataRead(index, data) || data[0] != PLACEMENT_HEIGHTS_RECORD) return 0U;
    heights->firstLayerDmm = (uint16_t)data[1];
    heights->secondLayerDmm = (uint16_t)(data[1] >> 16);
    heights->rawTrayDmm = (uint16_t)data[2];
    heights->returnTrayDmm = (uint16_t)(data[2] >> 16);
    return placementHeightsValid(heights);
}

static uint8_t placementHeightsLoad(PlacementHeights *heights)
{
    PlacementHeights candidate;
    uint32_t i;
    uint8_t found = 0U;
    for (i = 0U; i + 5U < RING_STORE_WORDS; i += 6U)
        if (placementHeightsRead(i, &candidate)) {
            *heights = candidate;
            found = 1U;
        }
    return found;
}

static uint8_t ringPosesLoad(RingPose poses[3])
{
    return poseSettingsLoad(poses, RING_RECORD_CURRENT);
}

/* Settings share sector 9; preserve both other types when it fills. */
static uint8_t ringRecordAppend(uint32_t data[6])
{
    volatile const uint32_t *words = (volatile const uint32_t *)RING_STORE_BASE;
    uint32_t i, j, next = 0U, other[2][6], candidate[6];
    uint8_t foundOther[2] = {0U, 0U}, type, ownType;
    FLASH_Status status = FLASH_COMPLETE;
    ownType = data[0] == PLACEMENT_HEIGHTS_RECORD ? 1U : data[0] == RAW_PICK_RECORD ? 2U : 0U;
    for (i = 0U; i + 5U < RING_STORE_WORDS; i += 6U) {
        for (j = 0U; j < 6U; ++j)
            if (words[i + j] != 0xFFFFFFFFUL) next = i + 6U;
        if (ringRecordDataRead(i, candidate)) {
            type = candidate[0] == PLACEMENT_HEIGHTS_RECORD ? 1U : candidate[0] == RAW_PICK_RECORD ? 2U : 0U;
            if (type != ownType) {
                if (type > ownType) --type;
                memcpy(other[type], candidate, sizeof(candidate));
                foundOther[type] = 1U;
            }
        }
    }
    data[4] = ringRecordCheck(data);
    data[5] = ~data[0]; /* Commit marker is written last. */
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    if (next + 5U >= RING_STORE_WORDS) {
        status = FLASH_EraseSector(FLASH_Sector_9, VoltageRange_3);
        next = 0U;
        for (type = 0U; type < 2U; ++type) if (foundOther[type]) {
            for (i = 0U; i < 6U && status == FLASH_COMPLETE; ++i)
                status = FLASH_ProgramWord(RING_STORE_BASE + (next + i) * 4U, other[type][i]);
            next += 6U;
        }
    }
    for (i = 0U; i < 6U && status == FLASH_COMPLETE; ++i)
        status = FLASH_ProgramWord(RING_STORE_BASE + (next + i) * 4U, data[i]);
    FLASH_Lock();
    if (FLASH->ACR & FLASH_ACR_DCEN) {
        FLASH_DataCacheCmd(DISABLE);
        FLASH_DataCacheReset();
        FLASH_DataCacheCmd(ENABLE);
    }
    return status == FLASH_COMPLETE && ringRecordDataRead(next, candidate) &&
           memcmp(candidate, data, sizeof(candidate)) == 0;
}

static uint8_t poseSettingsSave(const RingPose poses[3], uint32_t magic)
{
    RingPose previous[3];
    uint32_t data[6];
    uint8_t i;
    if (!ringPosesValid(poses)) return 0U;
    if (poseSettingsLoad(previous, magic) && memcmp(previous, poses, sizeof(previous)) == 0) return 1U;
    data[0] = magic;
    for (i = 0U; i < 3U; ++i)
        data[i + 1U] = (uint16_t)poses[i].horizontalDmm | ((uint32_t)poses[i].turretDdeg << 16);
    return ringRecordAppend(data);
}

static uint8_t ringPosesSave(const RingPose poses[3])
{
    return poseSettingsSave(poses, RING_RECORD_CURRENT);
}

static uint8_t placementHeightsSave(const PlacementHeights *heights)
{
    PlacementHeights previous;
    uint32_t data[6] = {PLACEMENT_HEIGHTS_RECORD, 0U, 0U, 0U, 0U, 0U};
    if (!placementHeightsValid(heights)) return 0U;
    if (placementHeightsLoad(&previous) && memcmp(&previous, heights, sizeof(previous)) == 0) return 1U;
    data[1] = heights->firstLayerDmm | ((uint32_t)heights->secondLayerDmm << 16);
    data[2] = heights->rawTrayDmm | ((uint32_t)heights->returnTrayDmm << 16);
    return ringRecordAppend(data);
}

#endif
