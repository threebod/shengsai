#ifndef SHENGSAI_TURN_TEST_LOG_H
#define SHENGSAI_TURN_TEST_LOG_H
#include "route_heading_log.h"
#include "route_plan.h"

typedef struct {
    uint32_t sample;
    uint8_t sampleValid, commandValid;
    int16_t wheels[4];
} TurnTestLog;

/* RAM only. Commands retain their control-clock timestamps; IMU samples retain
 * their own timestamps, so identification need not confuse the two clocks. */
static void turnTestLogRecord(TaskLog *log, TurnTestLog *state,
                               uint32_t now, uint32_t sample, float target, float yaw,
                               float raw, float ramp, const RouteTurnPid *pid,
                               const int16_t *wheels, uint8_t band)
{
    char line[160];
    int n;
    if (!state->commandValid || memcmp(state->wheels,wheels,sizeof(state->wheels))) {
        n=snprintf(line,sizeof(line),"TC %lu,%d,%d,%d,%d",(unsigned long)now,
                   (int)wheels[0],(int)wheels[1],(int)wheels[2],(int)wheels[3]);
        if (n>0 && n<(int)sizeof(line)) taskLogAppend(log,(const uint8_t *)line,(unsigned)n);
        memcpy(state->wheels,wheels,sizeof(state->wheels));state->commandValid=1U;
    }
    if (state->sampleValid && state->sample==sample) return;
    n=snprintf(line,sizeof(line),"TY %lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%u",
               (unsigned long)now,(unsigned long)sample,routeHeadingLogCdeg(target),
               routeHeadingLogCdeg(yaw),routeHeadingLogCdeg(headingError(target,yaw)),
               (long)(raw*100.0f+(raw>=0?0.5f:-0.5f)),
               (long)(ramp*100.0f+(ramp>=0?0.5f:-0.5f)),
               (long)(pid->integral*1000.0f+(pid->integral>=0?0.5f:-0.5f)),(unsigned)band);
    if (n>0 && n<(int)sizeof(line)) taskLogAppend(log,(const uint8_t *)line,(unsigned)n);
    state->sample=sample;state->sampleValid=1U;
}

static void turnTestLogBegin(TaskLog *log, TurnTestLog *state, uint32_t now,
                              int8_t direction, int8_t sign, float start, float target, const uint16_t *trims)
{
    char line[224];int n;
    memset(state,0,sizeof(*state));
    n=snprintf(line,sizeof(line),"TURN_TEST ms=%lu direction=%d yaw_sign=%d start_cd=%ld target_cd=%ld",
               (unsigned long)now,(int)direction,(int)sign,
               routeHeadingLogCdeg(start),routeHeadingLogCdeg(target));
    if(n>0 && n<(int)sizeof(line)) taskLogAppend(log,(const uint8_t *)line,(unsigned)n);
    n=snprintf(line,sizeof(line),
        "TURN_CFG kp_m=%ld ki_m=%ld kd_far_m=%ld kd_near_m=%ld switch_cd=%ld integral_m=%ld max_rpm=%u accel_m=%ld control_ms=20 stable_ms=200 tolerance_cd=50 trim1=%u trim2=%u trim3=%u trim4=%u",
        (long)(ROUTE_TURN_KP*1000),(long)(ROUTE_TURN_KI*1000),
        (long)(ROUTE_TURN_KD_FAR*1000),(long)(ROUTE_TURN_KD_NEAR*1000),
        (long)(ROUTE_TURN_SWITCH_DEG*100),(long)(ROUTE_TURN_INTEGRAL_LIMIT*1000),
        (unsigned)ROUTE_TURN_RPM,(long)(ROUTE_TURN_ACCEL*1000),
        (unsigned)trims[0],(unsigned)trims[1],(unsigned)trims[2],(unsigned)trims[3]);
    if(n>0 && n<(int)sizeof(line)) taskLogAppend(log,(const uint8_t *)line,(unsigned)n);
}

static void turnTestLogEnd(TaskLog *log, TurnTestLog *state, uint32_t now,
                            uint32_t sample, float target, float yaw, uint8_t aborted)
{
    char line[144];int n;
    /* Stop_Now bypasses routeSent; no PID step is associated with this sample. */
    if (!state->commandValid || state->wheels[0] || state->wheels[1] || state->wheels[2] || state->wheels[3]) {
        n=snprintf(line,sizeof(line),"TC %lu,0,0,0,0",(unsigned long)now);
        if(n>0 && n<(int)sizeof(line)) taskLogAppend(log,(const uint8_t *)line,(unsigned)n);
        memset(state->wheels,0,sizeof(state->wheels));state->commandValid=1U;
    }
    n=snprintf(line,sizeof(line),"TURN_END result=%s ms=%lu imu_ms=%lu actual_cd=%ld error_cd=%ld",
               aborted?"aborted":"complete",(unsigned long)now,(unsigned long)sample,
               routeHeadingLogCdeg(yaw),routeHeadingLogCdeg(headingError(target,yaw)));
    if(n>0 && n<(int)sizeof(line)) taskLogAppend(log,(const uint8_t *)line,(unsigned)n);
}
#endif
