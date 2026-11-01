#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include "../firmware/vision_control.h"
#include "../firmware/route_plan.h"

#define VISION_MOVE_STATUS_POLL_MS 20U
#define HOST_HEARTBEAT_TIMEOUT_MS 1000U
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
enum { S_FLAG, MISSION_IDLE, MISSION_TEMP_ALIGN };
static VisionSession visionSession;
static VisionCalibration calibration = {1,{0,0,320,240},160,120,{-1.111f,0,0,-1.143f}};
static VisionPacket visionRxPacket, lastRequest;
static uint32_t clockMs, visionRxStamp, imuStamp, motionStart, motionDuration, lastControl;
static uint32_t hostHeartbeatStamp;
static uint8_t visionRxReady, imuValid, emergencyStop, hostEStop, fault, motionMode;
static uint8_t missionActive, routeWaiting, missionOffline, fullRouteRunning, missionPhase;
static uint8_t hostHeartbeatActive, visionPickActive, rawPickAtStation, routeSentValid;
static uint8_t visionMotionMotorMask, visionMotionQueryId, visionMotionSeenMask, visionMotionStatus[4];
static uint8_t visionMotionAutomatic, statusReady, statusValues[4];
static uint16_t motorTrim[5] = {1000,1000,1000,1000,1000};
static float imuYaw, routeYaw, missionOffsetForward, missionOffsetRight;
static float routeForwardScale = ROUTE_FORWARD_SCALE, routeLateralScale = ROUTE_LATERAL_SCALE;
static int8_t yawSign = 1;
static RouteHeadingPidGains routeHeadingGains = {ROUTE_HEADING_KP,ROUTE_HEADING_KI,ROUTE_HEADING_KD};
static unsigned enables, stops, sends, requests, failures, queries;
static unsigned pixelSamples;
static int16_t sentForward, sentRight, sentTurn;
static unsigned expectedWheelLimit = 15;
static const char *failureReason;
static void cancelVisionTracking(void);
static void failVision(const char *reason);
static uint8_t motionInterrupted(void) { return fault || emergencyStop; }
static const VisionCalibration *visionCalibration(void) { return &calibration; }
static void stopDriveMotors(void) { ++stops; sentForward = sentRight = sentTurn = 0; }
static void setAllMotorsEnabled(int enabled) { assert(enabled); ++enables; }
static void sendRouteSpeeds(int16_t forward, int16_t right, int16_t turn)
{
    uint8_t id;
    for (id = 1; id <= 4; ++id)
        assert((unsigned)abs(routeWheel(id,forward,right,turn,motorTrim[id])) <= expectedWheelLimit);
    ++sends; sentForward = forward; sentRight = right; sentTurn = turn;
}
static void visionUartSend(const VisionPacket *packet) { ++requests; lastRequest = *packet; }
static void invalidateNavigation(const char *reason) { (void)reason; }
static void printVisionMotionStatus(void) {}
static void serialSendString(const char *value) { (void)value; }
static float headingError(float target, float yaw) { return target-yaw; }
static void jogCanResetDriveStatus(void) { statusReady = 0; }
static uint8_t jogCanTakeDriveStatus(uint8_t id, uint8_t *status)
{
    uint8_t bit = (uint8_t)(1U<<(id-1U));
    if (!(statusReady&bit)) return 0;
    statusReady &= (uint8_t)~bit; *status = statusValues[id-1U]; return 1;
}
static void jogCanExpectDriveStatus(uint8_t id) { assert(id >= 1 && id <= 4); }
static void Emm_V5_Read_Sys_Params(uint8_t id, int flag)
{ assert(id >= 1 && id <= 4 && flag == S_FLAG); ++queries; }
static void recordVisionPixels(const VisionPacket *packet, uint32_t sample, const char *phase)
{
    assert(packet->flags == (VISION_FLAG_TRACKING|VISION_FLAG_VALID));
    assert(sample <= clockMs && phase);
    ++pixelSamples;
}
#include "../firmware/vision_tracking_runtime.h"
#include "../firmware/vision_motion_runtime.h"

static void failVision(const char *reason)
{
    failureReason = reason; ++failures;
    cancelVisionTracking(); motionMode = 0; visionSession.state = VISION_STATE_FAILED;
}
static void reset(void)
{
    vision_tracking_cancel(&visionTracking);
    visionTrackingOpen = visionTrackingStarted = visionTrackingEndPending = 0;
    vision_session_init(&visionSession);
    visionSession.state = VISION_STATE_MOVE; visionSession.mode = VISION_MODE_RING;
    visionSession.target = 2; visionSession.maxCorrections = 0; visionSession.iteration = 1;
    clockMs = imuStamp = hostHeartbeatStamp = 100;
    imuValid = 1; imuYaw = routeYaw = 0;
    visionRxReady = emergencyStop = hostEStop = fault = motionMode = 0;
    missionActive = routeWaiting = missionOffline = fullRouteRunning = 0;
    missionPhase = MISSION_IDLE; hostHeartbeatActive = 1;
    visionPickActive = rawPickAtStation = statusReady = 0;
    enables = stops = sends = requests = failures = queries = pixelSamples = 0;
    sentForward = sentRight = 0; failureReason = NULL;
    expectedWheelLimit = 15;
    for (unsigned id = 0; id <= 4; ++id) motorTrim[id] = 1000;
    missionOffsetForward = missionOffsetRight = 0;
    assert(startVisionTracking(44.44f,22.86f));
    assert(lastRequest.flags&VISION_FLAG_TRACKING);
    assert(lastRequest.value[7] == VISION_TRACK_START && lastRequest.token&0x80000000UL);
}
static void queueFrame(int du, int dv, unsigned sequence, uint8_t flags)
{
    visionRxPacket = visionTrackingRequest;
    visionRxPacket.type = VISION_MESSAGE_RESULT; visionRxPacket.flags = flags;
    visionRxPacket.value[0] = (int16_t)(160+du); visionRxPacket.value[1] = (int16_t)(120+dv);
    visionRxPacket.value[2] = 80; visionRxPacket.value[5] = visionSession.selector;
    visionRxPacket.value[6] = (int16_t)sequence; visionRxPacket.value[7] = 0;
    visionRxStamp = clockMs; visionRxReady = 1;
}
static void step(unsigned dt)
{ clockMs += dt; imuStamp = clockMs; hostHeartbeatStamp = clockMs; serviceMotion(); }
static void run(void)
{
    queueFrame(-40,-20,1,VISION_FLAG_TRACKING|VISION_FLAG_VALID);
    step(20); step(20);
    assert(enables == 1 && sends && visionTrackingStarted);
}
int main(void)
{
    unsigned i;
    uint32_t token;
    float expected;
    int16_t farForward;
    /* Station tracking must correct an initial yaw error in both mission modes. */
    for (i = 0; i < 4; ++i) {
        reset(); cancelVisionTracking();
        missionActive = routeWaiting = 1;
        missionOffline = fullRouteRunning = (uint8_t)(i & 1U);
        routeYaw = i < 2U ? 90.0f : -90.0f;
        imuYaw = routeYaw - 2.0f;
        if (i >= 2U) {
            visionSession.mode = VISION_MODE_MATERIAL; visionSession.selector = 4;
            visionSession.maxCorrections = 2; missionPhase = MISSION_TEMP_ALIGN;
            expectedWheelLimit = 30;
        }
        assert(startVisionTracking(44.44f,22.86f));
        assert(visionTrackingYaw == routeYaw);
        run(); assert(!failures && sentTurn > 0);
        imuYaw = routeYaw - 3.0f;
        queueFrame(-40,-20,2,VISION_FLAG_TRACKING|VISION_FLAG_VALID); step(20);
        assert(visionTrackingYaw == routeYaw && sentTurn > 0 && !failures);
    }
    /* Standalone alignment has no station target and holds its starting yaw. */
    reset(); cancelVisionTracking(); routeYaw = 90.0f; imuYaw = 7.0f;
    assert(startVisionTracking(44.44f,22.86f) && visionTrackingYaw == 7.0f);
    imuYaw = 9.0f; run(); assert(!failures && sentTurn < 0);

    reset();queueFrame(-40,-20,1,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(0);
    step(9);assert(!sentForward && !sentRight);
    step(1);assert(sentForward>0 && sentRight>0);
    /* A fresh accepted axis stops immediately, before the next 10ms ramp tick. */
    for (i = 0; i <= 1; ++i) {
        reset(); run();
        queueFrame((int)i,-20,2,VISION_FLAG_TRACKING|VISION_FLAG_VALID); step(1);
        assert(!sentForward && sentRight > 0 && !failures && !visionTracking.confirmations);
        reset(); run();
        queueFrame(-40,(int)i,2,VISION_FLAG_TRACKING|VISION_FLAG_VALID); step(1);
        assert(sentForward > 0 && !sentRight && !failures && !visionTracking.confirmations);
    }
    reset(); step(20); assert(!enables && !sends && visionTracking.state == VISION_TRACK_WAIT);
    run(); farForward = sentForward; assert(farForward > 0 && sentRight > 0);
    for (i = 2; i <= 5; ++i) { queueFrame(-4,0,i,VISION_FLAG_TRACKING|VISION_FLAG_VALID); step(20); }
    assert(sentForward < farForward && sentRight == 0);
    token = visionTrackingRequest.token;
    queueFrame(1,1,6,VISION_FLAG_TRACKING|VISION_FLAG_VALID); step(20);
    assert(visionTrackingActive() && visionSession.state == VISION_STATE_MOVE && !sentForward && !sentRight);
    queueFrame(1,1,6,VISION_FLAG_TRACKING|VISION_FLAG_VALID); step(20);
    assert(visionTrackingActive() && !sentForward && !sentRight);
    queueFrame(0,1,7,VISION_FLAG_TRACKING|VISION_FLAG_VALID); step(20);
    assert(visionTrackingActive() && visionTracking.confirmations == 2U);
    queueFrame(0,1,8,VISION_FLAG_TRACKING|VISION_FLAG_VALID); step(20);
    assert(!visionTrackingActive() && !motionMode && stops && visionSession.state == VISION_STATE_ALIGNED);
    assert(visionSession.iteration == 1 && requests == 1);
    assert(vision_session_tick(&visionSession,clockMs+300) == VISION_EVENT_NONE);
    flushVisionTrackingEnd(); assert(lastRequest.value[7] == VISION_TRACK_END && lastRequest.token == token);

    reset(); queueFrame(-40,0,1,VISION_FLAG_VALID|VISION_FLAG_STABLE); step(20);
    assert(failures == 1 && !enables && !sends && !strcmp(failureReason,"CAMERA_TRACKING_UNSUPPORTED"));
    reset(); step(1000); assert(failures == 1 && !enables && !sends);
    assert(!strcmp(failureReason,"TRACK_ACK_TIMEOUT"));
    reset(); run(); missionActive = routeWaiting = missionOffline = fullRouteRunning = 1;
    token = visionTrackingRequest.token;
    for(i=0;i<17;++i) step(20);
    assert(!failures && visionTrackingActive() && motionMode == 4 && visionTracking.state == VISION_TRACK_HOLD);
    assert(stops == 1 && !sentForward && !sentRight && !sentTurn && missionActive && routeWaiting);
    expected = missionOffsetForward;
    {
        unsigned stopped = stops, polled = queries;
        imuYaw = -2.0f; /* Holding must stop yaw correction as well as translation. */
        for(i=0;i<100;++i) step(20);
        assert(stops == stopped && queries > polled && !sentForward && !sentRight && !sentTurn);
    }
    assert(!failures && missionOffsetForward == expected && requests == 1 && !visionTrackingEndPending);
    queueFrame(-40,0,2,VISION_FLAG_TRACKING|VISION_FLAG_VALID);visionRxPacket.value[7]=319;step(20);
    assert(visionTracking.state == VISION_TRACK_HOLD && !sentForward);
    queueFrame(-40,0,1,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
    assert(visionTracking.state == VISION_TRACK_HOLD && !sentForward);
    queueFrame(-40,0,2,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);step(20);
    assert(!failures && sentForward > 0 && visionTrackingRequest.token == token && enables == 1 && requests == 1);
    queueFrame(0,0,3,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
    queueFrame(0,0,4,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
    assert(visionTrackingActive() && visionTracking.confirmations == 2U);
    queueFrame(0,0,5,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
    assert(!failures && visionSession.state == VISION_STATE_ALIGNED && missionActive);

    reset();run();for(i=0;i<17;++i) step(20);
    cancelVisionTracking();motionMode=0;vision_session_pause(&visionSession);
    queueFrame(-40,0,2,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
    assert(!failures && !visionTrackingActive() && !sentForward && !sentRight);

    reset(); run(); missionActive = routeWaiting = missionOffline = fullRouteRunning = 1;
    expected = routeEstimateScaled(sentForward,20,routeForwardScale);
    step(20); assert(fabsf(missionOffsetForward-expected) < 0.001f);
    assert(missionOffsetForward < 2.0f && missionOffsetRight < 2.0f);
    cancelVisionTracking(); motionMode = 0; vision_session_pause(&visionSession);
    token = enables; queueFrame(-40,0,3,VISION_FLAG_TRACKING|VISION_FLAG_VALID); step(20);
    assert(enables == token && !visionTrackingActive() && !sentForward && !sentRight);
    reset(); run(); missionActive = routeWaiting = 1;
    expected = routeEstimateScaled(sentForward,5,routeForwardScale);
    clockMs += 5; imuStamp = clockMs;
    queueFrame(0,0,2,VISION_FLAG_TRACKING|VISION_FLAG_VALID); receiveVisionTracking();
    assert(visionTrackingActive());
    queueFrame(0,0,3,VISION_FLAG_TRACKING|VISION_FLAG_VALID); receiveVisionTracking();
    assert(visionTrackingActive() && visionTracking.confirmations == 2U);
    queueFrame(0,0,4,VISION_FLAG_TRACKING|VISION_FLAG_VALID); receiveVisionTracking();
    assert(!visionTrackingActive() && fabsf(missionOffsetForward-expected) < 0.001f);
    reset(); run(); assert(sentForward > 0);
    clockMs += 1; imuStamp = clockMs;
    queueFrame(40,-20,2,VISION_FLAG_TRACKING|VISION_FLAG_VALID); serviceMotion();
    assert(sentForward == 0 && visionTracking.driveForwardRpm == 0);

    for (i = 0; i < 5; ++i) {
        reset(); run();
        if (i == 0) emergencyStop = 1;
        if (i == 1) fault = 1;
        if (i == 2) imuValid = 0;
        if (i == 3) imuYaw = 21.0f;
        if (i == 4) { clockMs += 151; imuStamp = clockMs; }
        serviceMotion(); assert(failures == 1 && !visionTrackingActive() && !motionMode && stops);
    }
    /* Faults while waiting still end the session; late frames cannot restart it. */
    for(i=0;i<4;++i) {
        unsigned j;
        reset();run();for(j=0;j<17;++j) step(20);
        assert(visionTracking.state == VISION_TRACK_HOLD);
        if(i==0) emergencyStop=1;
        if(i==1) fault=1;
        if(i==2) imuValid=0;
        if(i==3) hostHeartbeatStamp=clockMs-1001U;
        serviceMotion();assert(failures == 1 && !visionTrackingActive());
        queueFrame(-40,0,2,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
        assert(failures == 1 && !visionTrackingActive() && !sentForward && !sentRight);
    }
    reset();run();for(i=0;i<17;++i) step(20);
    statusReady=1;statusValues[0]=5;step(20);
    assert(failures == 1 && !strcmp(failureReason,"MOTOR_STATUS_FAULT"));
    reset(); hostHeartbeatStamp = clockMs-1001U; serviceMotion();
    assert(failures == 1 && !strcmp(failureReason,"HEARTBEAT_TIMEOUT") && !enables);
    reset(); run(); statusReady = 1; statusValues[0] = 5; step(20);
    assert(failures == 1 && !strcmp(failureReason,"MOTOR_STATUS_FAULT"));
    reset(); run(); statusReady = 1; statusValues[0] = 5;
    queueFrame(0,0,2,VISION_FLAG_TRACKING|VISION_FLAG_VALID); receiveVisionTracking();
    assert(failures == 1 && visionSession.state == VISION_STATE_FAILED);
    reset(); run(); statusReady = 1; statusValues[0] = 5;
    visionTracking.deadlineMs = clockMs; serviceMotion();
    assert(failures == 1 && visionSession.state == VISION_STATE_FAILED);

    reset(); run(); imuYaw = -5.0f;
    motorTrim[1] = 1600; motorTrim[2] = 900; motorTrim[3] = 1300;
    queueFrame(-40,-20,2,VISION_FLAG_TRACKING|VISION_FLAG_VALID); step(20);
    assert(sentTurn > 0 && visionTracking.driveForwardRpm == sentForward && visionTracking.driveRightRpm == sentRight);
    reset(); cancelVisionTracking(); visionSession.iteration = 2;
    assert(!startVisionTracking(44.44f,22.86f)); /* No second ring tracking session. */

    reset(); cancelVisionTracking();
    assert(startVisionTracking(2.222f,0));
    queueFrame(-2,0,1,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
    for(i=2;i<8;++i){queueFrame(-2,0,i,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);}
    assert(!failures && visionTrackingActive() && sentForward>0);
    queueFrame(0,0,8,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
    queueFrame(0,0,9,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
    assert(visionTrackingActive() && visionTracking.confirmations == 2U);
    queueFrame(0,0,10,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
    assert(!failures && visionSession.state==VISION_STATE_ALIGNED);

    reset(); cancelVisionTracking();
    i = requests; assert(!startVisionTracking(NAN,20)); assert(requests == i && !enables);
    routeForwardScale = NAN; assert(!startVisionTracking(20,20));
    routeForwardScale = ROUTE_FORWARD_SCALE; assert(requests == i && !enables);

    /* A ring ignores the former deadline while fresh feedback keeps arriving. */
    reset(); run(); visionTracking.deadlineMs = clockMs;
    for(i=2;i<52;++i) { queueFrame(-2,0,i,VISION_FLAG_TRACKING|VISION_FLAG_VALID); step(20); }
    assert(!failures && visionTrackingActive() && visionSession.state == VISION_STATE_MOVE && sentForward>0);
    queueFrame(0,0,52,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
    queueFrame(0,0,53,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
    assert(visionTrackingActive() && visionTracking.confirmations == 2U);
    queueFrame(0,0,54,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
    assert(!failures && !visionTrackingActive() && visionSession.state == VISION_STATE_ALIGNED);
    /* Exhausting the former ring budget must neither stop nor report alignment. */
    reset(); run(); visionTracking.budgetMm = visionTracking.travelledMm; serviceMotion();
    for(i=2;i<52;++i) { queueFrame(-40,-20,i,VISION_FLAG_TRACKING|VISION_FLAG_VALID); step(20); }
    assert(!failures && visionTrackingActive() && visionSession.state == VISION_STATE_MOVE);
    assert(visionTracking.travelledMm>visionTracking.budgetMm && sentForward+sentRight>0);
    queueFrame(0,0,52,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
    assert(visionTrackingActive() && visionSession.state == VISION_STATE_MOVE);
    queueFrame(0,0,53,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
    assert(visionTrackingActive() && visionTracking.confirmations == 2U);
    queueFrame(0,0,54,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
    assert(!failures && !visionTrackingActive() && visionSession.state == VISION_STATE_ALIGNED);
    /* Second-layer material retains 30 RPM and the two-round settle/limit behavior. */
    for(i = 1; i <= 2; ++i) {
        reset(); cancelVisionTracking(); visionSession.state = VISION_STATE_MOVE;
        visionSession.mode = VISION_MODE_MATERIAL; visionSession.selector = 4;
        visionSession.maxCorrections = 2; visionSession.iteration = (uint8_t)i;
        missionActive = 1; missionPhase = MISSION_TEMP_ALIGN; expectedWheelLimit = 30;
        assert(startVisionTracking(44.44f,22.86f) && visionTracking.limitRpm == 30);
        queueFrame(1,1,1,VISION_FLAG_TRACKING|VISION_FLAG_VALID); step(20);
        assert(!enables && visionSession.state == VISION_STATE_SETTLE);
        assert(vision_session_tick(&visionSession,clockMs+299) == VISION_EVENT_NONE);
        assert(vision_session_tick(&visionSession,clockMs+300) == (i==2?VISION_EVENT_CORRECTION_LIMIT:VISION_EVENT_REQUEST));
    }

    reset(); cancelVisionTracking(); visionSession.state = VISION_STATE_MOVE;
    visionSession.mode = VISION_MODE_MATERIAL; visionSession.selector = 4;
    visionSession.maxCorrections = 2; visionSession.iteration = 1;
    missionActive = 1; missionPhase = MISSION_TEMP_ALIGN; expectedWheelLimit = 30;
    assert(startVisionTracking(44.44f,22.86f));
    queueFrame(-40,-20,1,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
    visionTracking.deadlineMs = clockMs;serviceMotion();
    assert(!failures && !visionTrackingActive() && visionSession.state == VISION_STATE_SETTLE);

    /* Material tracking also retains its session and remaining motion time while held. */
    reset();cancelVisionTracking();visionSession.state=VISION_STATE_MOVE;
    visionSession.mode=VISION_MODE_MATERIAL;visionSession.selector=4;
    visionSession.maxCorrections=2;visionSession.iteration=1;
    missionActive=routeWaiting=1;missionPhase=MISSION_TEMP_ALIGN;expectedWheelLimit=30;
    assert(startVisionTracking(44.44f,22.86f));
    run();for(i=0;i<17;++i) step(20);
    assert(visionTracking.state==VISION_TRACK_HOLD);
    for(i=0;i<100;++i) step(20);
    assert(!failures && visionSession.state==VISION_STATE_MOVE && visionSession.iteration==1);
    queueFrame(-40,0,2,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);step(20);
    assert(!failures && visionTrackingActive() && sentForward>0 && visionSession.state==VISION_STATE_MOVE);
    queueFrame(0,0,3,VISION_FLAG_TRACKING|VISION_FLAG_VALID);step(20);
    assert(!failures && !visionTrackingActive() && visionSession.state==VISION_STATE_SETTLE);

    reset(); cancelVisionTracking(); visionSession.sampleOnly = 1; assert(!visionTrackingEligible());
    visionSession.sampleOnly = 0; rawPickAtStation = 1; assert(!visionTrackingEligible());
    rawPickAtStation = 0; visionSession.requestFlags = VISION_FLAG_MATERIAL_LAYOUT; assert(!visionTrackingEligible());
    visionSession.requestFlags = 0; visionSession.mode = VISION_MODE_MATERIAL; visionSession.selector = 4;
    assert(!visionTrackingEligible()); missionActive = 1; missionPhase = MISSION_TEMP_ALIGN; visionSession.maxCorrections = 2;
    assert(visionTrackingEligible());
    reset(); assert(startVisionTracking(44.44f,22.86f));
    queueFrame(-40,0,1,VISION_FLAG_TRACKING|VISION_FLAG_VALID); receiveVisionTracking();
    assert(pixelSamples == 1);
    queueFrame(-40,0,1,VISION_FLAG_TRACKING|VISION_FLAG_VALID); receiveVisionTracking();
    assert(pixelSamples == 1); /* Duplicate frames must not look like new measurements. */
    queueFrame(-40,0,2,VISION_FLAG_TRACKING|VISION_FLAG_VALID);
    visionRxPacket.token++; receiveVisionTracking(); assert(pixelSamples == 1);
    queueFrame(-40,0,2,VISION_FLAG_TRACKING); receiveVisionTracking();
    assert(pixelSamples == 1 && failures); /* Invalid targets carry no measured error. */
    puts("PASS production vision tracking runtime ACK gate, live speed, stop/fault/watchdog, offsets and correction limits");
    return 0;
}
