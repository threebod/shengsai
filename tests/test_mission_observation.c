#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include "../firmware/route_plan.h"
#include "../firmware/raw_pick_action.h"

#define SERVO_EMPTY_TURRET_SPEED_DPS10 2500U
static const MechanismPose missionRingObservePose =
    {-770, 0, 2750, 1000, 200, 2000, 240, 2200};
static struct {
    struct { uint8_t taskIndex, arc; } legs[1];
    unsigned index, count;
} presetRoute;
static uint8_t missionActive, presetRouteActive, routeActive, routeWaiting;
static uint8_t routeRotating, missionArcActive, motionMode, missionObservationTask;
static uint8_t imuValid, hostEStop, fault, mechanismActionActive, visionActive;
static uint8_t servoChannelMoving[3], gripperMayHoldMaterial;
static int8_t routeHeading;
static uint32_t clockMs, imuStamp;
static float routeForwardScale = ROUTE_FORWARD_SCALE, routeLateralScale = ROUTE_LATERAL_SCALE;
static MechanismState mechanismState;
static unsigned starts, aborts, servoStops;
static uint8_t rejectStart;
static void cancelMissionObservation(void);
static uint8_t motionInterrupted(void) { return fault; }
static uint8_t visionSessionActive(void) { return visionActive; }
static void stopServoMotion(void)
{
    ++servoStops;
    memset(servoChannelMoving, 0, sizeof(servoChannelMoving));
}
static void missionAbort(const char *reason)
{
    assert(reason && *reason);
    ++aborts;
    cancelMissionObservation();
    routeActive = missionActive = 0U;
    mechanismStateInvalidate(&mechanismState);
}
static void startMechanismPose(const MechanismPose *target)
{
    ++starts;
    assert(target->liftDmm == 0U);
    if (!rejectStart) assert(mechanismStateStart(&mechanismState, target, clockMs));
}
#include "../firmware/mission_observation.h"

static RouteLeg leg;
static void reset(uint8_t task)
{
    memset(&presetRoute, 0, sizeof(presetRoute));
    memset(&mechanismState, 0, sizeof(mechanismState));
    memset(&leg, 0, sizeof(leg));
    memset(servoChannelMoving, 0, sizeof(servoChannelMoving));
    missionActive = presetRouteActive = routeActive = imuValid = 1U;
    routeWaiting = routeRotating = missionArcActive = motionMode = missionObservationTask = 0U;
    hostEStop = fault = mechanismActionActive = visionActive = rejectStart = 0U;
    gripperMayHoldMaterial = 1U;
    starts = aborts = servoStops = 0U;
    clockMs = imuStamp = 1000U;
    routeHeading = 1;
    presetRoute.count = 1U;
    presetRoute.legs[0].taskIndex = task;
    mechanismState.valid = 1U;
    mechanismState.current = rawPickTravelPose;
    leg.event = "STATION";
    leg.upMm = 0;
    leg.rightMm = -870;
    leg.heading = leg.departureHeading = routeHeading;
}

static void finishPose(void)
{
    mechanismState.current = mechanismState.target;
    mechanismState.running = 0U;
}

static void eligibility(void)
{
    unsigned i;
    uint8_t tasks[] = {3U,6U,8U,10U,13U,15U};
    uint8_t *blocked[] = {&routeWaiting,&routeRotating,&missionArcActive,&motionMode,
        &mechanismActionActive,&visionActive,&servoChannelMoving[0],&servoChannelMoving[1],
        &servoChannelMoving[2],&mechanismState.running,&presetRoute.legs[0].arc};
    for (i = 0U; i < sizeof(tasks); ++i) {
        reset(tasks[i]);
        serviceMissionObservation(&leg, 5.0f, 100U, 0U);
        assert(starts == 1U && missionObservationTask == tasks[i]);
        assert(mechanismState.target.horizontalDmm ==
               ((tasks[i] == 3U || tasks[i] == 10U) ? 650 : -770));
        assert(mechanismState.target.turretDdeg == 2750U);
        serviceMissionObservation(&leg, 1.0f, 100U, 0U);
        assert(starts == 1U); /* No resubmission while running or after feedback. */
        finishPose();
        serviceMissionObservation(&leg, 1.0f, 100U, 0U);
        assert(starts == 1U);
    }
    for (i = 0U; i < sizeof(blocked) / sizeof(blocked[0]); ++i) {
        reset(3U); *blocked[i] = 1U;
        serviceMissionObservation(&leg, 5.0f, 100U, 0U);
        assert(!starts && !aborts);
    }
    reset(3U); missionActive = 0U; serviceMissionObservation(&leg, 1, 100, 0); assert(!starts);
    reset(3U); presetRouteActive = 0U; serviceMissionObservation(&leg, 1, 100, 0); assert(!starts);
    reset(3U); routeActive = 0U; serviceMissionObservation(&leg, 1, 100, 0); assert(!starts);
    reset(3U); leg.event = 0; serviceMissionObservation(&leg, 1, 100, 0); assert(!starts);
    reset(1U); serviceMissionObservation(&leg, 1, 100, 0); assert(!starts); /* QR */
    reset(19U); serviceMissionObservation(&leg, 1, 100, 0); assert(!starts); /* HOME */
    reset(3U); leg.departureHeading = 2; serviceMissionObservation(&leg, 1, 100, 0); assert(!starts);
    reset(3U); leg.heading = 2; serviceMissionObservation(&leg, 1, 100, 0); assert(!starts);
    reset(3U); mechanismState.current.liftDmm = 1; serviceMissionObservation(&leg, 1, 100, 0); assert(!starts);
    reset(3U); mechanismState.current.horizontalDmm = -450;
    mechanismState.current.turretDdeg = 1350;
    serviceMissionObservation(&leg, 1, 100, 0); assert(!starts);
    for (i = 0U; i < 2U; ++i) {
        reset(i ? 15U : 8U);
        mechanismState.current.horizontalDmm = i ? -470 : -450;
        mechanismState.current.turretDdeg = 1350;
        serviceMissionObservation(&leg, 1, 100, 0); assert(starts == 1U);
        reset(i ? 15U : 8U);
        mechanismState.current.horizontalDmm = i ? -450 : -470;
        mechanismState.current.turretDdeg = 1350;
        serviceMissionObservation(&leg, 1, 100, 0); assert(!starts);
    }
}

static void timing(void)
{
    float leadDistance;
    uint32_t loaded, empty;
    reset(3U);
    loaded = missionObservationLeadMs(&rawPickObservePose);
    gripperMayHoldMaterial = 0U;
    empty = missionObservationLeadMs(&rawPickObservePose);
    assert(loaded > empty && loaded == turretMotionDurationMs(207000U,2200U) + 100U);
    assert(empty == turretMotionDurationMs(207000U,2500U) + 100U);
    leadDistance = missionObservationLeadDistance(empty,100U,routeForwardScale);
    assert(leadDistance > missionObservationLeadDistance(empty,10U,routeForwardScale));
    assert(leadDistance < missionObservationLeadDistance(empty,250U,routeForwardScale));
    serviceMissionObservation(&leg, leadDistance + 1.0f, 100U, 0U); assert(!starts);
    serviceMissionObservation(&leg, leadDistance - 1.0f, 100U, 0U); assert(starts == 1U);
    reset(6U); mechanismState.current.turretDdeg = 2750;
    assert(missionObservationLeadMs(&missionRingObservePose) ==
           mechanismMoveDurationMs(&mechanismState.current,&missionRingObservePose) + 100U);
    assert(missionObservationLeadDistance(100U,250U,1.0f) > 0.0f); /* braking-only */
    reset(3U); gripperMayHoldMaterial = 0U;
    leadDistance = missionObservationLeadDistance(missionObservationLeadMs(&rawPickObservePose),
                                                100U,routeLateralScale);
    serviceMissionObservation(&leg, leadDistance+1.0f,100U,1U); assert(!starts);
    serviceMissionObservation(&leg, leadDistance-1.0f,100U,1U); assert(starts == 1U);
}

static void arrivalAndCancellation(void)
{
    unsigned i;
    reset(3U); assert(finishMissionObservation(4U) == 0);
    serviceMissionObservation(&leg,1,100,0);
    routeWaiting = 1U;
    clockMs += 60000U;
    assert(finishMissionObservation(4U) == 1 && missionObservationTask);
    finishPose(); servoChannelMoving[2] = 1U;
    assert(finishMissionObservation(4U) == 1);
    servoChannelMoving[2] = 0U; routeWaiting = 0U;
    assert(finishMissionObservation(4U) == 1); /* Ready while driving cannot enable vision. */
    routeWaiting = 1U;
    assert(finishMissionObservation(4U) == 2 && !missionObservationTask && starts == 1U);
    assert(finishMissionObservation(4U) == 0);
    for (i = 0U; i < 3U; ++i) {
        reset(6U); serviceMissionObservation(&leg,1,100,0); finishPose(); routeWaiting = 1U;
        if (i == 0U) mechanismState.valid = 0U;
        if (i == 1U) mechanismState.current.horizontalDmm = 0;
        assert(finishMissionObservation(i == 2U ? 9U : 7U) == -1);
        assert(aborts == 1U && !routeActive && !missionObservationTask && servoStops == 1U);
    }
    reset(3U); serviceMissionObservation(&leg,1,100,0); cancelMissionObservation();
    assert(!missionObservationTask && servoStops == 1U);
    cancelMissionObservation(); assert(servoStops == 1U);
    reset(3U); rejectStart = 1U; serviceMissionObservation(&leg,1,100,0);
    assert(aborts == 1U && !routeActive && !missionObservationTask);
    for (i = 0U; i < 5U; ++i) {
        reset(3U);
        if (i == 0U) mechanismState.valid = 0U;
        if (i == 1U) fault = 1U;
        if (i == 2U) imuValid = 0U;
        if (i == 3U) clockMs = imuStamp + 251U;
        if (i == 4U) hostEStop = 1U;
        serviceMissionObservation(&leg,1,100,0);
        assert(!starts && aborts == 1U && !routeActive);
    }
}

static void coarseCentreBeforeArrivalTurn(void)
{
    unsigned round;
    for (round = 0U; round < 2U; ++round) {
        uint8_t task = round ? 13U : 6U;
        float lead;
        reset(task);
        routeHeading = leg.departureHeading = 2;
        leg.heading = -1; leg.upMm = -1680; leg.rightMm = 0;
        lead = missionObservationLeadDistance(missionObservationLeadMs(&missionRingObservePose),
                                               150U,routeForwardScale);
        serviceMissionObservation(&leg,lead+1.0f,150U,0U); assert(!starts);
        serviceMissionObservation(&leg,lead-1.0f,150U,0U);
        assert(starts == 1U && missionObservationTask == task);
        assert(mechanismState.target.horizontalDmm == -770 && mechanismState.target.liftDmm == 0U);
        assert(mechanismState.target.turretDdeg == missionRingObservePose.turretDdeg);
        /* Finished while driving still cannot start station vision. */
        finishPose(); assert(finishMissionObservation((uint8_t)(task+1U)) == 1);
        routeHeading = -1; routeWaiting = 1U;
        assert(finishMissionObservation((uint8_t)(task+1U)) == 2);
    }
    reset(6U); leg.upMm = 1680; leg.rightMm = 0; leg.heading = -1;
    serviceMissionObservation(&leg,1,100,0); assert(!starts);
    reset(6U); leg.heading = -1; /* Side arrivals retain the original heading guard. */
    serviceMissionObservation(&leg,1,100,0); assert(!starts);
    reset(6U); leg.upMm = -1680; leg.rightMm = 0; leg.heading = 0;
    serviceMissionObservation(&leg,1,100,0); assert(!starts);
    reset(6U); routeHeading = leg.departureHeading = 2;
    leg.upMm = -1680; leg.rightMm = 0; leg.heading = -1; fault = 1U;
    serviceMissionObservation(&leg,1,100,0); assert(!starts && aborts == 1U);
}

static void rawReturnUnfoldNearArrival(void)
{
    float lead;
    reset(10U);
    routeHeading = leg.departureHeading = 2;
    leg.upMm = 880; leg.rightMm = 0; leg.heading = 1;
    mechanismState.current = rawPickTravelPose;
    lead = missionObservationLeadDistance(missionObservationLeadMs(&rawPickObservePose),
                                           150U,routeForwardScale);
    serviceMissionObservation(&leg,lead+1.0f,150U,0U);
    assert(!starts && mechanismState.current.horizontalDmm == 0);
    assert(mechanismState.current.turretDdeg == rawPickTravelPose.turretDdeg);
    serviceMissionObservation(&leg,lead-1.0f,150U,0U);
    assert(starts == 1U && missionObservationTask == 10U);
    assert(mechanismState.target.horizontalDmm == rawPickObservePose.horizontalDmm);
    assert(mechanismState.target.turretDdeg == rawPickObservePose.turretDdeg);
    assert(finishMissionObservation(11U) == 1);
    finishPose(); routeWaiting = 1U;
    assert(finishMissionObservation(11U) == 2);
    reset(10U); leg.event = 0;
    serviceMissionObservation(&leg,1,150U,0U); assert(!starts);
    reset(19U); serviceMissionObservation(&leg,1,150U,0U); assert(!starts);
}

int main(void)
{
    eligibility(); timing(); arrivalAndCancellation(); coarseCentreBeforeArrivalTurn();
    rawReturnUnfoldNearArrival();
    puts("PASS mission observation: six stations, dynamic lead, no concurrent actions, feedback arrival and cancellation");
    return 0;
}
