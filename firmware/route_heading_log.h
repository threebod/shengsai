#ifndef SHENGSAI_ROUTE_HEADING_LOG_H
#define SHENGSAI_ROUTE_HEADING_LOG_H

#include <stdio.h>
#include "task_log.h"
#include "straight_control.h"

typedef struct {
    uint32_t task, at, sample;
    uint16_t segment, templateId;
    uint8_t step, phase;
} RouteHeadingLog;

static long routeHeadingLogCdeg(float degrees)
{
    float value = headingError(degrees, 0.0f) * 100.0f;
    return (long)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

/* Record in RAM only: no UART traffic or connection dependency in control ticks.
 * Angles are signed centidegrees; phase D/T/C/A = drive/turn/curve/arrival. */
static void routeHeadingLogRecord(TaskLog *log, RouteHeadingLog *state,
                                  uint32_t now, uint32_t sample,
                                  uint8_t step, uint16_t segment,
                                  uint16_t templateId, uint8_t phase,
                                  float target, float actual)
{
    char line[128];
    int length;
    if (state->task == log->task && state->step == step &&
        state->segment == segment && state->templateId == templateId &&
        state->phase == phase &&
        (now - state->at < 500U || sample == state->sample)) return;
    length = snprintf(line, sizeof(line),
        "YAW ms=%lu step=%u seg=%u tpl=%u phase=%c target=%ld actual=%ld error=%ld age=%lu",
        (unsigned long)now, (unsigned)step, (unsigned)segment,
        (unsigned)templateId, (int)phase, routeHeadingLogCdeg(target),
        routeHeadingLogCdeg(actual), routeHeadingLogCdeg(headingError(target, actual)),
        (unsigned long)(now - sample));
    if (length > 0 && length < (int)sizeof(line)) {
        taskLogAppend(log, (const uint8_t *)line, (unsigned)length);
        state->task = log->task;
        state->at = now;
        state->sample = sample;
        state->step = step;
        state->segment = segment;
        state->templateId = templateId;
        state->phase = phase;
    }
}

#endif
