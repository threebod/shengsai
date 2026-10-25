#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../shared/protocol.h"
#include "../firmware/raw_pick_action.h"

static uint8_t emergencyStop, hostEStop, jogCanFault, obstacleTurretPending, obstacleTurretReady;
static uint8_t motionMode, routeActive, mechanismActionActive, visionActive;
static uint8_t servoChannelMoving[3];
static uint32_t currentAngleMdeg[3];
static MechanismState mechanismState;
static uint8_t hostStartPending, hostStartZone, hostCancelled, hostOperationFailed, hostConfirmed;
static uint8_t missionButtonReady, missionActive, missionOffline, armed, hostExecuting, hostError;
static uint16_t hostStartRpm, missionButtonRpm;
static uint8_t hostHeartbeatActive;
static uint32_t clockMs, hostHeartbeatStamp;
#define HOST_HEARTBEAT_TIMEOUT_MS 1000U
static unsigned servoCalls, stops, missionStarts, routeStarts;
static uint8_t instantServo, rejectMission;

static uint8_t visionSessionActive(void) { return visionActive; }
static void invalidateMechanism(const char *reason) { (void)reason; mechanismState.valid = 0U; }
static void servoSetTarget(uint8_t channel, uint16_t angle, uint16_t speed)
{
    assert(channel >= 2U && channel <= 4U);
    assert(angle == (channel == 2U ? rawPickInitial.gripperCloseDeg : channel == 3U ? 26U : 53U));
    assert(speed == (channel == 2U ? rawPickInitial.gripperDps10 : channel == 3U ? rawPickInitial.platformDps10 : 1200U));
    ++servoCalls;
    servoChannelMoving[channel - 2U] = !instantServo;
    if (instantServo) currentAngleMdeg[channel - 2U] = (uint32_t)angle * 1000U;
}
static void stopServoMotion(void)
{
    ++stops;
    obstacleTurretPending = obstacleTurretReady = 0U;
    memset(servoChannelMoving, 0, sizeof(servoChannelMoving));
}
static void startButtonMission(uint8_t zone)
{
    assert(zone == 2U && missionButtonReady && missionButtonRpm == 240U);
    assert(!obstacleTurretPending && mechanismState.current.turretDdeg == 530U);
    assert(currentAngleMdeg[0] == (uint32_t)rawPickInitial.gripperCloseDeg * 1000U &&
           currentAngleMdeg[1] == 26000U);
    ++missionStarts;
    missionActive = !rejectMission;
}
static void processCommand(const char *command)
{
    assert(strcmp(command, "route auto 2 240") == 0);
    assert(armed && hostExecuting && !obstacleTurretPending);
    ++routeStarts;
    routeActive = 1U;
}
#include "../firmware/obstacle_turret.h"

static void reset(void)
{
    emergencyStop = hostEStop = jogCanFault = obstacleTurretPending = obstacleTurretReady = 0U;
    motionMode = routeActive = mechanismActionActive = visionActive = 0U;
    hostStartPending = hostCancelled = hostOperationFailed = 0U;
    hostConfirmed = 1U;
    hostHeartbeatActive = 0U;
    clockMs = hostHeartbeatStamp = 0U;
    missionButtonReady = missionActive = missionOffline = armed = hostExecuting = hostError = 0U;
    servoCalls = stops = missionStarts = routeStarts = 0U;
    instantServo = rejectMission = 0U;
    rawPickInitial.gripperCloseDeg = 155U;
    memset(servoChannelMoving, 0, sizeof(servoChannelMoving));
    memset(currentAngleMdeg, 0, sizeof(currentAngleMdeg));
    memset(&mechanismState, 0, sizeof(mechanismState));
    mechanismState.valid = 1U;
    mechanismState.current.horizontalDmm = -450;
    mechanismState.current.liftDmm = 300U;
    mechanismState.current.turretDdeg = 2600U;
    hostStartZone = 2U;
    hostStartRpm = 240U;
}

static void finishPreparation(void)
{
    currentAngleMdeg[0] = (uint32_t)rawPickInitial.gripperCloseDeg * 1000U;
    currentAngleMdeg[1] = 26000U;
    currentAngleMdeg[2] = 53000U;
    memset(servoChannelMoving, 0, sizeof(servoChannelMoving));
    serviceObstacleTurret();
    assert(obstacleTurretReady && !obstacleTurretPending);
    assert(mechanismState.current.turretDdeg == 530U);
    assert(mechanismState.current.horizontalDmm == -450 && mechanismState.current.liftDmm == 300U);
}

int main(void)
{
    unsigned i;
    uint8_t *blocked[] = {&emergencyStop, &hostEStop, &jogCanFault, &hostStartPending, &motionMode,
        &routeActive, &mechanismState.running, &mechanismActionActive, &visionActive,
        &servoChannelMoving[0], &servoChannelMoving[1], &servoChannelMoving[2]};
    for (i = 0U; i < sizeof(blocked) / sizeof(blocked[0]); ++i) {
        reset();
        *blocked[i] = 1U;
        assert(!prepareObstacleTurret() && servoCalls == 0U);
    }
    reset();
    assert(prepareObstacleTurret());
    assert(!prepareObstacleTurret() && servoCalls == 3U);
    serviceObstacleTurret();
    assert(obstacleTurretPending && !obstacleTurretReady && mechanismState.current.turretDdeg == 2600U);
    finishPreparation();
    assert(mechanismState.valid);

    for (i = 0U; i < 3U; ++i) {
        reset();
        instantServo = 1U;
        assert(prepareObstacleTurret() && servoCalls == 3U);
        servoChannelMoving[i] = 1U;
        hostStartPending = SS_MISSION;
        serviceObstacleTurret();
        serviceObstacleHostStart();
        assert(obstacleTurretPending && !obstacleTurretReady && !missionStarts);
        servoChannelMoving[i] = 0U;
        serviceObstacleTurret();
        serviceObstacleHostStart();
        assert(missionStarts == 1U && missionOffline);
    }

    reset();
    rawPickInitial.gripperCloseDeg = 167U; /* Use the configured angle, not a fixed 155 degrees. */
    instantServo = 1U;
    assert(prepareObstacleTurret());
    serviceObstacleTurret();
    assert(obstacleTurretReady && currentAngleMdeg[0] == 167000U);

    reset();
    mechanismState.valid = hostConfirmed = 0U; /* Physical HMI works offline, without inventing an axis zero. */
    instantServo = 1U;
    assert(prepareObstacleTurret());
    serviceObstacleTurret();
    assert(obstacleTurretReady && !mechanismState.valid);

    reset();
    assert(prepareObstacleTurret());
    jogCanFault = 1U;
    serviceObstacleTurret();
    assert(stops == 1U && !obstacleTurretReady && !mechanismState.valid && hostOperationFailed);

    for (i = 0U; i < 3U; ++i) {
        reset();
        instantServo = 1U;
        assert(prepareObstacleTurret());
        currentAngleMdeg[i] = 0U; /* Stopped short, with the wrong command angle. */
        serviceObstacleTurret();
        assert(stops == 1U && !obstacleTurretReady && !mechanismState.valid && hostOperationFailed);
    }

    reset();
    assert(prepareObstacleTurret());
    hostStartPending = SS_MISSION;
    serviceObstacleHostStart();
    assert(hostStartPending && !missionStarts);
    finishPreparation();
    serviceObstacleHostStart();
    assert(missionStarts == 1U && !hostStartPending && missionOffline);
    serviceObstacleHostStart();
    assert(missionStarts == 1U);

    reset();
    assert(prepareObstacleTurret());
    hostStartPending = SS_ROUTE;
    finishPreparation();
    serviceObstacleHostStart();
    assert(routeStarts == 1U && !hostStartPending);

    for (i = 0U; i < 6U; ++i) {
        reset();
        assert(prepareObstacleTurret());
        hostStartPending = SS_MISSION;
        if (i == 0U) hostCancelled = 1U;
        if (i == 1U) hostEStop = 1U;
        if (i == 2U) hostConfirmed = 0U;
        if (i == 3U) hostOperationFailed = 1U;
        if (i == 4U) jogCanFault = 1U;
        if (i == 5U) { hostHeartbeatActive = 1U; clockMs = 1001U; }
        serviceObstacleHostStart();
        assert(!hostStartPending && !missionStarts && !routeStarts && hostOperationFailed && stops == 1U);
    }
    reset();
    hostStartPending = SS_MISSION;
    serviceObstacleHostStart(); /* A cancelled turret move cannot release the queued mission. */
    assert(hostOperationFailed && !missionStarts);

    reset();
    assert(prepareObstacleTurret());
    hostStartPending = SS_MISSION;
    finishPreparation();
    rejectMission = 1U;
    serviceObstacleHostStart();
    assert(hostOperationFailed && !missionOffline);
    return 0;
}
