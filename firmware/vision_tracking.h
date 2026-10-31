#ifndef SHENGSAI_VISION_TRACKING_H
#define SHENGSAI_VISION_TRACKING_H

#include <float.h>
#include <math.h>
#include "vision_control.h"
#include "route_plan.h"

#define VISION_TRACK_FRESH_MS 350U
#define VISION_TRACK_ACK_MS 1000U
#define VISION_TRACK_WIRE_MS 32U
#define VISION_TRACK_CONTROL_MS 10U
/* The watchdog may be relaxed without slowing the existing error response. */
#define VISION_TRACK_RESPONSE_MS 500U

typedef enum {
    VISION_TRACK_WAIT = 0, VISION_TRACK_RUN, VISION_TRACK_DONE,
    VISION_TRACK_FAILED, VISION_TRACK_IDLE, VISION_TRACK_HOLD
} VisionTrackingState;

typedef struct {
    VisionTrackingState state;
    const char *reason;
    uint32_t token, requestAt, durationMs, deadlineMs, updatedAt, controlAt, lastFrameAt;
    uint16_t sequence, limitRpm;
    uint8_t mode, selector, target, sequenceValid, confirmations;
    float forwardMm, rightMm, forwardRpm, rightRpm;
    float driveForwardRpm, driveRightRpm, budgetMm, travelledMm;
} VisionTracking;

static uint8_t vision_tracking_finite(float value)
{ return (uint8_t)(value >= -FLT_MAX && value <= FLT_MAX); }

static VisionTrackingState vision_tracking_stop(VisionTracking *tracking,
                                                 VisionTrackingState state,
                                                 const char *reason)
{
    tracking->state = state;
    tracking->reason = reason;
    tracking->forwardRpm = tracking->rightRpm = 0.0f;
    tracking->driveForwardRpm = tracking->driveRightRpm = 0.0f;
    return state;
}

static void vision_tracking_cancel(VisionTracking *tracking)
{
    memset(tracking, 0, sizeof(*tracking));
    tracking->state = VISION_TRACK_IDLE;
    tracking->reason = "CANCELLED";
}

static void vision_tracking_begin(VisionTracking *tracking, uint32_t token,
                                   uint8_t mode, uint8_t selector, uint8_t target,
                                   float initialForwardMm, float initialRightMm,
                                   uint16_t rpm, uint32_t durationMs, uint32_t now)
{
    memset(tracking, 0, sizeof(*tracking));
    tracking->state = VISION_TRACK_WAIT;
    tracking->reason = "NONE";
    tracking->token = token;
    tracking->mode = mode; tracking->selector = selector; tracking->target = target;
    tracking->requestAt = tracking->updatedAt = tracking->controlAt = now;
    tracking->durationMs = durationMs;
    tracking->limitRpm = rpm;
    if (!vision_tracking_finite(initialForwardMm) || !vision_tracking_finite(initialRightMm) ||
        !durationMs || !rpm || rpm > 30U || selector > 6U ||
        !((mode == VISION_MODE_RING && target >= 1U && target <= 3U) ||
          (mode == VISION_MODE_MATERIAL && target == 2U && selector >= 1U))) {
        vision_tracking_stop(tracking, VISION_TRACK_FAILED, "TRACK_START_INVALID");
        return;
    }
    tracking->budgetMm = fabsf(initialForwardMm) + fabsf(initialRightMm);
    if (!vision_tracking_finite(tracking->budgetMm))
        vision_tracking_stop(tracking, VISION_TRACK_FAILED, "TRACK_START_INVALID");
    else if (mode != VISION_MODE_RING && tracking->budgetMm == 0.0f)
        vision_tracking_stop(tracking, VISION_TRACK_DONE, "TRACK_BUDGET");
}

static VisionTrackingState vision_tracking_observe(VisionTracking *tracking,
                                                    const VisionCalibration *calibration,
                                                    const VisionPacket *packet,
                                                    uint32_t rxStamp, uint32_t now)
{
    uint16_t sequence, advance;
    uint32_t age;
    int16_t du, dv;
    if (tracking->state != VISION_TRACK_WAIT && tracking->state != VISION_TRACK_RUN &&
        tracking->state != VISION_TRACK_HOLD)
        return tracking->state;
    if (tracking->state == VISION_TRACK_WAIT && now - tracking->requestAt >= VISION_TRACK_ACK_MS)
        return vision_tracking_stop(tracking, VISION_TRACK_FAILED, "TRACK_ACK_TIMEOUT");
    if (packet->type != VISION_MESSAGE_RESULT || packet->token != tracking->token ||
        packet->mode != tracking->mode || packet->selector != tracking->selector ||
        packet->target != tracking->target || !(packet->flags & VISION_FLAG_TRACKING))
        return tracking->state;
    sequence = (uint16_t)packet->value[6];
    advance = (uint16_t)(sequence - tracking->sequence);
    if (tracking->sequenceValid && (!advance || advance >= 0x8000U)) return tracking->state;
    age = (uint16_t)packet->value[7] + VISION_TRACK_WIRE_MS;
    if (now - rxStamp > VISION_TRACK_FRESH_MS ||
        age > VISION_TRACK_FRESH_MS - (now - rxStamp)) {
        return tracking->state;
    }
    age += now - rxStamp;
    if (!vision_calibration_valid(calibration) ||
        packet->flags != (VISION_FLAG_TRACKING | VISION_FLAG_VALID) || !vision_quality_valid(packet) ||
        packet->value[0] < calibration->roi[0] ||
        packet->value[0] >= (int32_t)calibration->roi[0] + calibration->roi[2] ||
        packet->value[1] < calibration->roi[1] ||
        packet->value[1] >= (int32_t)calibration->roi[1] + calibration->roi[3] ||
        (tracking->mode == VISION_MODE_MATERIAL && packet->value[5] != tracking->selector))
        return vision_tracking_stop(tracking, VISION_TRACK_FAILED, "TRACK_TARGET_INVALID");
    du = (int16_t)(packet->value[0] - calibration->anchorU);
    dv = (int16_t)(packet->value[1] - calibration->anchorV);
    vision_map_error(calibration, du, dv, &tracking->forwardMm, &tracking->rightMm);
    if (!vision_tracking_finite(tracking->forwardMm) || !vision_tracking_finite(tracking->rightMm))
        return vision_tracking_stop(tracking, VISION_TRACK_FAILED, "TRACK_CALIBRATION_INVALID");
    if (tracking->state == VISION_TRACK_RUN && now - tracking->lastFrameAt > VISION_TRACK_FRESH_MS)
        tracking->confirmations = 0U;
    if (tracking->state == VISION_TRACK_HOLD) {
        /* Waiting consumes neither the material motion duration nor its travel budget. */
        tracking->deadlineMs += now - tracking->controlAt;
        tracking->updatedAt = tracking->controlAt = now;
        tracking->state = VISION_TRACK_RUN;
        tracking->reason = "NONE";
    }
    tracking->sequence = sequence; tracking->sequenceValid = 1U;
    tracking->lastFrameAt = now - age;
    if (du >= 0 && du <= VISION_ALIGN_TOLERANCE_PX &&
        dv >= 0 && dv <= VISION_ALIGN_TOLERANCE_PX) {
        if (tracking->mode != VISION_MODE_RING || ++tracking->confirmations >= 3U)
            return vision_tracking_stop(tracking, VISION_TRACK_DONE, "TRACK_IN_RANGE");
        tracking->forwardMm = tracking->rightMm = 0.0f;
        tracking->forwardRpm = tracking->rightRpm = 0.0f;
        tracking->driveForwardRpm = tracking->driveRightRpm = 0.0f;
    } else tracking->confirmations = 0U;
    if (tracking->state == VISION_TRACK_WAIT) {
        tracking->state = VISION_TRACK_RUN;
        tracking->deadlineMs = now + tracking->durationMs;
        tracking->updatedAt = tracking->controlAt = now;
    }
    return tracking->state;
}

static float vision_tracking_ramp(float current, float target, uint32_t dt)
{
    /* A stale direction must not persist after the measured error changes sign. */
    if ((current > 0.0f && target < 0.0f) || (current < 0.0f && target > 0.0f)) return 0.0f;
    return routeSlew(current, target,
        fabsf(target) < fabsf(current) ? ROUTE_BRAKE_RPM_S : ROUTE_ACCEL_RPM_S, dt);
}

/* Outputs are body-axis RPM, rounded here so the budget integrates sent commands. */
static VisionTrackingState vision_tracking_tick(VisionTracking *tracking, uint32_t now,
                                                 float forwardScale, float lateralScale,
                                                 float *forwardRpm, float *rightRpm)
{
    uint32_t dt;
    uint8_t reversed = 0U;
    float forward, right, total, distance, remaining;
    *forwardRpm = *rightRpm = 0.0f;
    if (tracking->state == VISION_TRACK_WAIT) {
        if (now - tracking->requestAt >= VISION_TRACK_ACK_MS)
            return vision_tracking_stop(tracking, VISION_TRACK_FAILED, "TRACK_ACK_TIMEOUT");
        return tracking->state;
    }
    if (tracking->state != VISION_TRACK_RUN && tracking->state != VISION_TRACK_HOLD)
        return tracking->state;
    if (!vision_tracking_finite(forwardScale) || !vision_tracking_finite(lateralScale) ||
        forwardScale <= 0.0f || lateralScale <= 0.0f)
        return vision_tracking_stop(tracking, VISION_TRACK_FAILED, "TRACK_SCALE_INVALID");
    dt = now - tracking->updatedAt;
    tracking->travelledMm += (fabsf(tracking->driveForwardRpm) * forwardScale +
        fabsf(tracking->driveRightRpm) * lateralScale) * (ROUTE_MM_PER_REV / 60000.0f) * dt;
    tracking->updatedAt = now;
    if (tracking->state == VISION_TRACK_HOLD) return tracking->state;
    if (now - tracking->lastFrameAt > VISION_TRACK_FRESH_MS) {
        tracking->controlAt = now;
        tracking->confirmations = 0U;
        return vision_tracking_stop(tracking, VISION_TRACK_HOLD, "TRACK_FRAME_TIMEOUT");
    }
    if (tracking->mode != VISION_MODE_RING && (int32_t)(now - tracking->deadlineMs) >= 0)
        return vision_tracking_stop(tracking, VISION_TRACK_DONE, "TRACK_DEADLINE");
    remaining = tracking->budgetMm - tracking->travelledMm;
    if (tracking->mode != VISION_MODE_RING && remaining <= 0.0f)
        return vision_tracking_stop(tracking, VISION_TRACK_DONE, "TRACK_BUDGET");
    if ((tracking->forwardRpm > 0.0f && tracking->forwardMm < 0.0f) ||
        (tracking->forwardRpm < 0.0f && tracking->forwardMm > 0.0f)) {
        tracking->forwardRpm = tracking->driveForwardRpm = 0.0f;
        reversed = 1U;
    }
    if ((tracking->rightRpm > 0.0f && tracking->rightMm < 0.0f) ||
        (tracking->rightRpm < 0.0f && tracking->rightMm > 0.0f)) {
        tracking->rightRpm = tracking->driveRightRpm = 0.0f;
        reversed = 1U;
    }
    if (reversed) tracking->controlAt = now;
    dt = now - tracking->controlAt;
    if (dt < VISION_TRACK_CONTROL_MS) {
        *forwardRpm = tracking->driveForwardRpm; *rightRpm = tracking->driveRightRpm;
        return tracking->state;
    }
    tracking->controlAt = now;
    forward = tracking->forwardMm * (60000.0f / (VISION_TRACK_RESPONSE_MS * ROUTE_MM_PER_REV)) / forwardScale;
    right = tracking->rightMm * (60000.0f / (VISION_TRACK_RESPONSE_MS * ROUTE_MM_PER_REV)) / lateralScale;
    total = fabsf(forward) + fabsf(right);
    if (total > tracking->limitRpm) {
        forward *= tracking->limitRpm / total; right *= tracking->limitRpm / total;
    }
    tracking->forwardRpm = vision_tracking_ramp(tracking->forwardRpm, forward, dt);
    tracking->rightRpm = vision_tracking_ramp(tracking->rightRpm, right, dt);
    total = fabsf(tracking->forwardRpm) + fabsf(tracking->rightRpm);
    if (total > tracking->limitRpm) {
        tracking->forwardRpm *= tracking->limitRpm / total;
        tracking->rightRpm *= tracking->limitRpm / total;
    }
    forward = (float)routeRound(tracking->forwardRpm);
    right = (float)routeRound(tracking->rightRpm);
    /* Ring errors below half an RPM still need the drive's smallest integer command. */
    if (tracking->mode == VISION_MODE_RING) {
        if (forward == 0.0f && tracking->forwardMm != 0.0f) forward = tracking->forwardMm > 0.0f ? 1.0f : -1.0f;
        if (right == 0.0f && tracking->rightMm != 0.0f) right = tracking->rightMm > 0.0f ? 1.0f : -1.0f;
    }
    if (fabsf(forward) + fabsf(right) > tracking->limitRpm) {
        if (fabsf(forward) > fabsf(right)) forward += forward > 0.0f ? -1.0f : 1.0f;
        else right += right > 0.0f ? -1.0f : 1.0f;
    }
    /* Material tracking reserves the complete frame-watchdog interval. */
    distance = (fabsf(forward) * forwardScale + fabsf(right) * lateralScale) *
        (ROUTE_MM_PER_REV / 60000.0f) * VISION_TRACK_FRESH_MS;
    if (tracking->mode != VISION_MODE_RING && distance > remaining) {
        forward = (float)(int16_t)(forward * remaining / distance);
        right = (float)(int16_t)(right * remaining / distance);
        if (forward == 0.0f && right == 0.0f)
            return vision_tracking_stop(tracking, VISION_TRACK_DONE, "TRACK_BUDGET");
    }
    tracking->driveForwardRpm = *forwardRpm = forward;
    tracking->driveRightRpm = *rightRpm = right;
    return tracking->state;
}

#endif
