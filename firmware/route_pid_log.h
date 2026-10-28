#ifndef SHENGSAI_ROUTE_PID_LOG_H
#define SHENGSAI_ROUTE_PID_LOG_H

#include <stdio.h>
#include "task_log.h"
#include "route_plan.h"
#include "straight_control.h"

typedef struct {
    uint32_t now, sample;
    uint16_t segment, templateId, dt;
    uint8_t step, sub;
    int8_t yawSign;
    int16_t wheels[4];
    float target, actual, raw, scale;
} RoutePidSample;

typedef struct {
    uint32_t task, span, at, sample, previousAt, elapsed;
    int32_t turnArea;
    int16_t previousTurn4;
    uint16_t segment, templateId;
    uint8_t step, sub;
} RoutePidLog;

static long routePidLogRound(float value)
{
    return (long)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

/* RAM-only straight-line diagnostics. Wheel mixtures are command values in
 * quarter RPM, including trims/limits; mean turn covers the preceding interval. */
static void routePidLogRecord(TaskLog *log, RoutePidLog *state,
                               const RoutePidSample *s,
                               const RouteHeadingPidGains *gains,
                               const RouteHeadingPid *pid)
{
    char line[224];
    int length;
    int16_t forward4 = s->wheels[0] + s->wheels[1] + s->wheels[2] + s->wheels[3];
    int16_t right4 = -s->wheels[0] + s->wheels[1] - s->wheels[2] + s->wheels[3];
    int16_t turn4 = s->wheels[0] - s->wheels[1] - s->wheels[2] + s->wheels[3];
    long meanTurn4;
    if (state->task != log->task || state->segment != s->segment ||
        state->templateId != s->templateId || state->step != s->step ||
        state->sub != s->sub || s->now - state->previousAt > 250U) {
        ++state->span;
        state->task = log->task;
        state->segment = s->segment;
        state->templateId = s->templateId;
        state->step = s->step;
        state->sub = s->sub;
        state->at = s->now - 100U;
        state->sample = s->sample - 1U;
        state->previousAt = s->now;
        state->turnArea = 0;
        state->elapsed = 0U;
        length = snprintf(line, sizeof(line),
            "PIDCFG span=%lu kp_m=%ld ki_m=%ld kd_m=%ld limit_m=%ld integral_m=%ld",
            (unsigned long)state->span, routePidLogRound(gains->kp * 1000.0f),
            routePidLogRound(gains->ki * 1000.0f), routePidLogRound(gains->kd * 1000.0f),
            routePidLogRound(ROUTE_HEADING_MAX_RPM * 1000.0f),
            routePidLogRound(ROUTE_HEADING_INTEGRAL_LIMIT * 1000.0f));
        if (length > 0 && length < (int)sizeof(line))
            taskLogAppend(log, (const uint8_t *)line, (unsigned)length);
    } else {
        uint32_t elapsed = s->now - state->previousAt;
        state->turnArea += state->previousTurn4 * (int32_t)elapsed;
        state->elapsed += elapsed;
        state->previousAt = s->now;
    }
    state->previousTurn4 = turn4;
    if (s->now - state->at < 100U || s->sample == state->sample) return;
    meanTurn4 = state->elapsed ? routePidLogRound((float)state->turnArea / state->elapsed) : turn4;
    length = snprintf(line, sizeof(line),
        "PID %lu,%lu,%lu,%u,%u,%u,%u,%ld,%ld,%d,%d,%d,%ld,%ld,%ld,%ld,%ld,%u,%d",
        (unsigned long)s->now, (unsigned long)s->sample, (unsigned long)state->span,
        (unsigned)s->step, (unsigned)s->segment, (unsigned)s->sub, (unsigned)s->templateId,
        routePidLogRound(headingError(s->target, 0.0f) * 100.0f),
        routePidLogRound(headingError(s->actual, 0.0f) * 100.0f),
        (int)forward4, (int)right4, (int)turn4, meanTurn4,
        routePidLogRound(s->raw * 100.0f), routePidLogRound(pid->integral * 1000.0f),
        routePidLogRound(pid->filteredRate * 100.0f), routePidLogRound(s->scale * 1000.0f),
        (unsigned)s->dt, (int)s->yawSign);
    if (length > 0 && length < (int)sizeof(line)) {
        taskLogAppend(log, (const uint8_t *)line, (unsigned)length);
        state->at = s->now;
        state->sample = s->sample;
        state->turnArea = 0;
        state->elapsed = 0U;
    }
}

#endif
