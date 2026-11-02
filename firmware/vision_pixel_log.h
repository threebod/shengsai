#ifndef SHENGSAI_VISION_PIXEL_LOG_H
#define SHENGSAI_VISION_PIXEL_LOG_H

#include <stdio.h>
#include "task_log.h"
#include "vision_control.h"

typedef struct {
    uint32_t task, token, at;
} VisionPixelLog;

/* Call only for accepted measurements. Terminal frames bypass throttling. */
static void visionPixelLogRecord(TaskLog *log, VisionPixelLog *state,
                                  uint32_t now, uint32_t sample, uint8_t step,
                                  const VisionPacket *packet,
                                  const VisionCalibration *calibration,
                                  const char *phase, const int16_t *wheels,
                                  uint32_t controlMs, uint32_t sendMs)
{
    char line[224];
    int length;
    if (!strcmp(phase,"TRACK") && state->task == log->task &&
        state->token == packet->token && now-state->at < 100U) return;
    length = snprintf(line, sizeof(line),
        "PIX ms=%lu step=%u phase=%s token=%lu mode=%u selector=%u target=%u u=%d v=%d au=%d av=%d du=%d dv=%d age=%lu cmd=%u m1=%d m2=%d m3=%d m4=%d ctrl_ms=%lu tx_ms=%lu",
        (unsigned long)now, (unsigned)step, phase, (unsigned long)packet->token,
        (unsigned)packet->mode, (unsigned)packet->selector, (unsigned)packet->target,
        (int)packet->value[0], (int)packet->value[1],
        (int)calibration->anchorU, (int)calibration->anchorV,
        (int)packet->value[0]-calibration->anchorU,
        (int)packet->value[1]-calibration->anchorV, (unsigned long)(now-sample),
        (unsigned)(wheels != 0), wheels ? (int)wheels[0] : 0, wheels ? (int)wheels[1] : 0,
        wheels ? (int)wheels[2] : 0, wheels ? (int)wheels[3] : 0,
        (unsigned long)controlMs, (unsigned long)sendMs);
    if (length > 0 && length < (int)sizeof(line)) {
        taskLogAppend(log, (const uint8_t *)line, (unsigned)length);
        state->task = log->task;
        state->token = packet->token;
        state->at = now;
    }
}

#endif
