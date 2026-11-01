#ifndef SHENGSAI_VISION_TRACKING_RUNTIME_H
#define SHENGSAI_VISION_TRACKING_RUNTIME_H
#include "vision_tracking.h"

static VisionTracking visionTracking;
static VisionPacket visionTrackingRequest;
static uint32_t visionTrackingEpoch, visionTrackingTick;
static float visionTrackingYaw;
static RouteHeadingPid visionTrackingHeadingPid;
static int16_t visionTrackingForward, visionTrackingRight;
static uint8_t visionTrackingOpen, visionTrackingStarted, visionTrackingEndPending;

static uint8_t visionTrackingActive(void) { return visionTrackingOpen; }

/* Stops are synchronous; ending the camera stream may wait until the next
 * vision service so an emergency never waits on a UART transmission. */
static void cancelVisionTracking(void)
{
    if (!visionTrackingOpen) return;
    stopDriveMotors();
    routeSentValid = 0U;
    visionTrackingForward = visionTrackingRight = 0;
    visionTrackingStarted = visionTrackingOpen = 0U;
    visionTrackingEndPending = 1U;
    vision_tracking_cancel(&visionTracking);
}

static void flushVisionTrackingEnd(void)
{
    VisionPacket end;
    if (!visionTrackingEndPending) return;
    end = visionTrackingRequest;
    end.value[7] = VISION_TRACK_END;
    visionTrackingEndPending = 0U;
    visionUartSend(&end);
}

static uint8_t visionTrackingEligible(void)
{
    return visionSession.state == VISION_STATE_MOVE && visionSession.iteration > 0U &&
        !visionSession.sampleOnly && !visionPickActive && !rawPickAtStation &&
        !(visionSession.requestFlags & VISION_FLAG_MATERIAL_LAYOUT) &&
        ((visionSession.mode == VISION_MODE_RING && visionSession.iteration == 1U) ||
         (visionSession.iteration <= 2U && visionSession.maxCorrections == 2U &&
          missionActive && missionPhase == MISSION_TEMP_ALIGN &&
          visionSession.mode == VISION_MODE_MATERIAL && visionSession.target == 2U));
}

static uint8_t startVisionTracking(float forwardMm, float rightMm)
{
    const VisionCalibration *calibration = visionCalibration();
    VisionSession requestSession = visionSession;
    float forward, right, largest, pulseEstimate;
    uint32_t pulses, duration;
    uint16_t rpm = visionSession.mode == VISION_MODE_RING ? 15U : 30U;
    if (!visionTrackingEligible() || !vision_calibration_valid(calibration) ||
        !imuValid || clockMs-imuStamp > 250U || motionInterrupted() ||
        !vision_tracking_finite(forwardMm) || !vision_tracking_finite(rightMm) ||
        !vision_tracking_finite(routeForwardScale) || !vision_tracking_finite(routeLateralScale) ||
        routeForwardScale <= 0.0f || routeLateralScale <= 0.0f) return 0U;
    forward = forwardMm/routeForwardScale; right = rightMm/routeLateralScale;
    largest = routeAbs(forward-right);
    if (routeAbs(forward+right) > largest) largest = routeAbs(forward+right);
    pulseEstimate = largest*3200.0f/ROUTE_MM_PER_REV+0.5f;
    if (!vision_tracking_finite(pulseEstimate) || pulseEstimate > (float)(0xFFFFFFFFUL/60000UL)) return 0U;
    pulses = (uint32_t)pulseEstimate;
    if (!pulses) return 0U;
    duration = pulses*60000U/(3200U*rpm)+800U;
    /* Combine the former two correction allowances into one ring session. */
    if (visionSession.mode == VISION_MODE_RING) duration *= 2U;
    flushVisionTrackingEnd();
    vision_session_make_request(&requestSession,calibration,&visionTrackingRequest,clockMs);
    visionTrackingEpoch = (visionTrackingEpoch+1U)&0x7FFFFFFFUL;
    visionTrackingRequest.token = visionTrackingEpoch|0x80000000UL;
    visionTrackingRequest.flags |= VISION_FLAG_TRACKING;
    visionTrackingRequest.value[7] = VISION_TRACK_START;
    vision_tracking_begin(&visionTracking,visionTrackingRequest.token,visionSession.mode,
                          visionSession.selector,visionSession.target,forwardMm,rightMm,rpm,duration,clockMs);
    /* Station alignment holds the route target; standalone alignment holds current yaw. */
    visionTrackingYaw = missionActive && routeWaiting ? routeYaw : imuYaw;
    visionTrackingTick = clockMs;
    visionTrackingForward = visionTrackingRight = 0;
    visionTrackingStarted = 0U;
    visionTrackingOpen = 1U;
    routeSentValid = 0U;
    routeHeadingPidReset(&visionTrackingHeadingPid);
    visionMotionMotorMask = visionMotionQueryId = visionMotionSeenMask = 0U;
    memset(visionMotionStatus,0,sizeof(visionMotionStatus));
    __disable_irq(); visionRxReady = 0U; __enable_irq();
    motionMode = 4U;
    motionStart = clockMs; motionDuration = duration;
    lastControl = clockMs;
    if (!(missionOffline && fullRouteRunning)) {
        hostHeartbeatActive = 1U; hostHeartbeatStamp = clockMs;
    }
    invalidateNavigation("vision_tracking");
    visionUartSend(&visionTrackingRequest);
    return 1U;
}

static void finishVisionTracking(void)
{
    uint32_t now = clockMs, dt = now-visionTrackingTick;
    if (visionSession.mode == VISION_MODE_RING &&
        strcmp(visionTracking.reason,"TRACK_IN_RANGE") != 0) {
        failVision(visionTracking.reason);
        return;
    }
    if (visionTrackingStarted && missionActive && routeWaiting) {
        missionOffsetForward += routeEstimateScaled(visionTrackingForward,dt,routeForwardScale);
        missionOffsetRight += routeEstimateScaled(visionTrackingRight,dt,routeLateralScale);
    }
    visionTrackingTick = now;
    cancelVisionTracking(); motionMode = 0U;
    if (visionSession.mode == VISION_MODE_RING) visionSession.state = VISION_STATE_ALIGNED;
    else vision_session_move_complete(&visionSession,now);
}

static uint8_t visionTrackingDriveHealthy(void)
{
    uint8_t id, status;
    if (!visionTrackingStarted) return 1U;
    for (id = 1U; id <= 4U; ++id) {
        if (!jogCanTakeDriveStatus(id,&status)) continue;
        visionMotionSeenMask |= (uint8_t)(1U<<(id-1U));
        visionMotionStatus[id-1U] = status;
        if (!(status&0x01U) || (status&0x0CU)) {
            printVisionMotionStatus(); failVision("MOTOR_STATUS_FAULT"); return 0U;
        }
    }
    return 1U;
}

static void receiveVisionTracking(void)
{
    VisionPacket packet;
    uint32_t stamp;
    uint16_t sequence = visionTracking.sequence;
    uint8_t sequenceValid = visionTracking.sequenceValid;
    if (!visionTrackingOpen || !visionTrackingDriveHealthy() || !visionRxReady) return;
    __disable_irq();
    packet = visionRxPacket; stamp = visionRxStamp; visionRxReady = 0U;
    __enable_irq();
    if (packet.type == VISION_MESSAGE_RESULT && packet.token == visionTrackingRequest.token &&
        packet.mode == visionSession.mode && packet.selector == visionSession.selector &&
        packet.target == visionSession.target && !(packet.flags&VISION_FLAG_TRACKING)) {
        failVision("CAMERA_TRACKING_UNSUPPORTED"); return;
    }
    vision_tracking_observe(&visionTracking,visionCalibration(),&packet,stamp,clockMs);
    if (visionTracking.sequenceValid && (!sequenceValid || sequence != visionTracking.sequence))
        recordVisionPixels(&packet, visionTracking.lastFrameAt,
                           visionTracking.state == VISION_TRACK_DONE ? "IN_RANGE" : "TRACK");
    if (visionTracking.state == VISION_TRACK_FAILED)
        failVision(visionTracking.reason ? visionTracking.reason : "TRACKING_FAILED");
    else if (visionTracking.state == VISION_TRACK_DONE) finishVisionTracking();
}

static void serviceVisionTrackingMotion(void)
{
    uint32_t now = clockMs, dt = now-visionTrackingTick, stamp;
    uint8_t id, wasHolding;
    float forward, right, yaw, error;
    int16_t sentForward, sentRight, turn, peak, limit, available;
    if (!visionTrackingOpen) return;
    __disable_irq(); yaw = imuYaw; stamp = imuStamp; __enable_irq();
    error = headingError(visionTrackingYaw,yaw);
    if (motionInterrupted() || hostEStop || !imuValid || now-stamp > 250U ||
        routeAbs(error) > 20.0f || (visionTrackingStarted && dt > 150U)) {
        failVision("TRACKING_IMU_CAN_OR_CONTROL_FAULT"); return;
    }
    if (!(missionOffline && fullRouteRunning) && hostHeartbeatActive &&
        now-hostHeartbeatStamp > HOST_HEARTBEAT_TIMEOUT_MS) {
        failVision("HEARTBEAT_TIMEOUT"); return;
    }
    receiveVisionTracking();
    if (!visionTrackingOpen) return;
    visionTrackingTick = now;
    if (visionTrackingStarted && missionActive && routeWaiting) {
        missionOffsetForward += routeEstimateScaled(visionTrackingForward,dt,routeForwardScale);
        missionOffsetRight += routeEstimateScaled(visionTrackingRight,dt,routeLateralScale);
    }
    wasHolding = (uint8_t)(visionTracking.state == VISION_TRACK_HOLD);
    vision_tracking_tick(&visionTracking,now,routeForwardScale,routeLateralScale,&forward,&right);
    if (visionTracking.state == VISION_TRACK_FAILED) {
        failVision(visionTracking.reason ? visionTracking.reason : "TRACKING_FAILED"); return;
    }
    if (visionTracking.state == VISION_TRACK_DONE) {
        finishVisionTracking(); return;
    }
    if (visionTracking.state == VISION_TRACK_HOLD && !wasHolding) {
        stopDriveMotors();
        routeSentValid = 0U;
        visionTrackingForward = visionTrackingRight = 0;
        routeHeadingPidReset(&visionTrackingHeadingPid);
        serialSendString("VISION WAIT: tracking frame timeout; waiting for fresh frame\r\n");
        if (motionInterrupted()) { failVision("TRACKING_STOP_FAILED"); return; }
    }
    if (visionTracking.state != VISION_TRACK_RUN && visionTracking.state != VISION_TRACK_HOLD) return;
    if (!visionTrackingStarted) {
        setAllMotorsEnabled(true);
        if (motionInterrupted()) { failVision("TRACKING_ENABLE_FAILED"); return; }
        visionTrackingStarted = 1U;
        visionMotionMotorMask = 15U;
        jogCanResetDriveStatus();
    }
    if (!visionTrackingDriveHealthy()) return;
    if (now-lastControl >= VISION_MOVE_STATUS_POLL_MS) {
        lastControl = now;
        visionMotionQueryId = (uint8_t)(visionMotionQueryId%4U+1U);
        jogCanExpectDriveStatus(visionMotionQueryId);
        Emm_V5_Read_Sys_Params(visionMotionQueryId,S_FLAG);
    }
    if (visionTracking.state == VISION_TRACK_HOLD) return;
    sentForward = routeRound(forward); sentRight = routeRound(right);
    turn = routeRound(routeHeadingPidStep(&visionTrackingHeadingPid,&routeHeadingGains,
                                         error,yaw,stamp,dt,yawSign));
    limit = (int16_t)visionTracking.limitRpm;
    if (turn > limit) turn = limit;
    if (turn < -limit) turn = -limit;
    available = (int16_t)(limit-abs(turn));
    peak = (int16_t)(abs(sentForward)+abs(sentRight));
    if (peak > available) {
        sentForward = (int16_t)((int32_t)sentForward*available/peak);
        sentRight = (int16_t)((int32_t)sentRight*available/peak);
    }
    /* Include per-wheel trims in the selected wheel-speed ceiling. */
    for (;;) {
        peak = 0;
        for (id = 1U; id <= 4U; ++id) {
            int16_t wheel = routeWheel(id,sentForward,sentRight,turn,motorTrim[id]);
            if (abs(wheel) > peak) peak = (int16_t)abs(wheel);
        }
        if (peak <= limit) break;
        sentForward = (int16_t)((int32_t)sentForward*limit/peak);
        sentRight = (int16_t)((int32_t)sentRight*limit/peak);
        turn = (int16_t)((int32_t)turn*limit/peak);
    }
    sendRouteSpeeds(sentForward,sentRight,turn);
    if (motionInterrupted()) { failVision("TRACKING_SEND_FAILED"); return; }
    visionTrackingForward = sentForward; visionTrackingRight = sentRight;
    visionTracking.driveForwardRpm = (float)sentForward;
    visionTracking.driveRightRpm = (float)sentRight;
}
#endif
