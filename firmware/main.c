#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"
#include "gripper_store.h"
#include "ring_store.h"
#include "delay.h"
#include "board.h"
#include "Emm_V5.h"

#include <string.h>
#include "straight_control.h"
#include "route_plan.h"
#include "arc_route_blend.h"
#include "preset_route.h"
#include "map_navigation.h"
#include "mechanism_action.h"
#include "raw_pick_action.h"
#include "mission_actions.h"
#include "task_code.h"
#include "vision_control.h"
#include "vision_config.h"
#include "qr_scan.h"
#include "obstacle_stop.h"
#include "avoidance_run.h"
#include "tjc_display.h"
#include "tjc_obstacles.h"
#include "hwt101.h"
#include "jog_can.h"
#include "axis_feedback.h"
#include "axis6_test.h"

#define COMMAND_BAUD_RATE  115200U

#define MOTOR_MIN_ID       1U
#define MOTOR_MAX_ID       4U
#define AUX_MOTOR_MIN_ID   5U
#define AUX_MOTOR_MAX_ID   6U
#define MOTOR_TEST_ACCEL   50U
#define VISION_MOVE_STATUS_POLL_MS  20U
#define HOST_HEARTBEAT_TIMEOUT_MS  1000U
#define MISSION_BUTTON_DEBOUNCE_MS 30U
#define MISSION_BUTTON_LONG_PRESS_MS 1000U

#define SERVO_MIN_PULSE_US    500U
#define SERVO_PULSE_RANGE_US  2000U
#define SERVO_PERIOD_US       20000U
#define SERVO_DEFAULT_SPEED_DPS10 1800U
#define SERVO_MAX_SPEED_DPS10     1800U
#define SERVO_GRIPPER_MAX_SPEED_DPS10 3600U
#define SERVO_EMPTY_TURRET_SPEED_DPS10 2500U
#define SERVO_UPDATE_HZ       50U
#define SERVO_STEP_MDEG       ((SERVO_DEFAULT_SPEED_DPS10 * 100U) / SERVO_UPDATE_HZ)

typedef enum {
    DIRECTION_FORWARD = 0,
    DIRECTION_BACK,
    DIRECTION_LEFT,
    DIRECTION_RIGHT
} MecanumDirection;

typedef struct {
    uint8_t channel;
    uint16_t angle;
    uint16_t speedDps10;
} ServoCommand;

/* Index 0 is unused; indexes 1..4 are the motor CAN addresses. */
static const uint8_t motorDirections[4][5] = {
    {1U, 1U, 0U, 0U, 1U},
    {0U, 0U, 1U, 1U, 0U},
    {1U, 1U, 1U, 0U, 0U},
    {0U, 0U, 0U, 1U, 1U}
};

static volatile uint8_t emergencyStop;
static uint8_t hostEStop;
static void hostLogChar(char c);
static uint8_t armed;
static uint8_t servoTimerReady;
static volatile uint8_t servoChannelEnabled[3];
static volatile uint8_t servoChannelMoving[3];
static volatile uint8_t completedServoChannels;
static volatile uint32_t currentAngleMdeg[3];
static volatile uint32_t targetAngleMdeg[3];
static volatile uint32_t turretStartAngleMdeg;
static volatile uint32_t turretElapsedMs;
static volatile uint32_t turretDurationMs;
/* A close command may hold material; opening releases it. No load sensor. */
static uint8_t gripperMayHoldMaterial = 1U;
static uint8_t ringPosesSaved;
static volatile uint32_t servoStepMdeg[3] = {
    SERVO_STEP_MDEG, SERVO_STEP_MDEG, SERVO_STEP_MDEG
};
extern volatile uint8_t jogCanFault;
volatile uint32_t clockMs;
static uint8_t hostHeartbeatActive;
static uint32_t hostHeartbeatStamp;
static uint8_t motorInvert[5];
static uint16_t motorTrim[5] = {1000U, 1000U, 1000U, 1000U, 1000U};
static int8_t yawSign = 1;
/* 0 idle, 1 position test, 2 heading hold, 3 aux, 4 vision, 5 move, 6 arc. */
static uint8_t motionMode;
static uint32_t motionStart, motionDuration, lastControl;
static void serviceMotion(void);
/* Route progress is nominal, NOT measured ground position. */
static uint8_t routeActive, routeWaiting, routeIndex, routeStartZone, routeStep;
/* Independent QR-to-RAW arc test: 1 approach, 2 QR wait, 3 arc, 4 exit. */
static uint8_t arcRoutePhase;
/* Formal mission uses only the selected corners; exit retains live speeds. */
static uint8_t missionArcActive, missionArcExit;
static uint32_t arcRouteWaitStart;
static ArcRouteBlend arcBlend;
static float routeX, routeY, routeYaw;
static float routeLegProgress;
static int16_t routeRawLeftCorrection;
static int16_t routeRawDownCorrection;
static uint8_t routeObstaclePosition = ROUTE_OBSTACLE_UP;
static uint16_t routeObstacleRegions;
static PresetRoute presetRoute;
static uint8_t presetRouteActive;
static int16_t routeForward, routeRight;
static uint32_t routeTick, routeLegStart, routeSettle;
static uint8_t routeAuto, routeRotating, routeOnlyTurn, routeTurnInBand;
/* 0 pending, 1 aligning station heading, 2 aligned for this leg. */
static uint8_t routeStationAlignment;
static uint8_t fullRouteRunning;
static uint8_t rawPickRouteActive, rawPickAtStation, rawPickItemIndex;
static uint8_t rawPickMissingReported;
static uint8_t rawPickCurrentColor, rawPickCurrentSlot, rawPickCollectedMask;
static MechanismAction rawPickRouteActions[RAW_PICK_ACTION_COUNT];
static const uint8_t rawPickColors[2][3] = {{4U, 2U, 6U}, {4U, 2U, 6U}};
static MissionTask missionTask;
static uint8_t missionTaskValid;
static char missionQrCode[16];
static uint8_t missionQrValid;
static uint8_t missionOffline;
static uint8_t missionButtonReady, missionButtonRawHigh, missionButtonStableHigh;
static uint8_t missionButtonLongSent;
static uint16_t missionButtonRpm;
static uint32_t missionButtonChangedAt, missionButtonPressedAt;
typedef enum {
    MISSION_IDLE, MISSION_QR_WAIT, MISSION_RAW_PICK,
    MISSION_RING_ALIGN, MISSION_TEMP_ALIGN, MISSION_COARSE_PLACE, MISSION_COARSE_PICK,
    MISSION_TEMP_PLACE, MISSION_RECENTER, MISSION_RAW_EXIT
} MissionPhase;
static uint8_t missionActive, missionActionStarted;
static uint8_t missionObservationTask;
static MissionPhase missionPhase;
static uint16_t missionGrabCount, missionPlaceCount;
static uint32_t missionDeadline;
static float missionOffsetForward, missionOffsetRight;
static MechanismAction missionWorkActions[MISSION_PLACE_COUNT];
static MechanismAction missionPickBackActions[MISSION_PICK_BACK_COUNT];
static const MechanismPose missionRingObservePose =
    {-770, 0, 2600, 1000, 200, 2000, 240, 2200};
static int8_t routeHeading, routeNextHeading;
static float routeBaseYaw;
static uint32_t routeTurnStart, routeTurnStable;
static uint8_t navInitialized, navRunning, navCurrentNode;
static NavPoint navPath[NAV_PATH_CAPACITY];
static uint8_t navPathCount;
static uint32_t navLastReport, routeLastReport;
static float routeDriveRpm, routeTurnRpm, routeCorrection;
static uint32_t headingPidLastReport;
static RouteHeadingPid routeHeadingPid;
static RouteTurnPid routeTurnPid;
static RouteHeadingPidGains routeHeadingGains = {
    ROUTE_HEADING_KP, ROUTE_HEADING_KI, ROUTE_HEADING_KD
};
static uint16_t routeRpm = ROUTE_RPM;
static float routeForwardScale = ROUTE_FORWARD_SCALE;
static float routeLateralScale = ROUTE_LATERAL_SCALE;
static uint16_t routeTurnRpmLimit = (uint16_t)ROUTE_TURN_RPM;
static uint16_t routeLateralRpmLimit = ROUTE_LATERAL_RPM_MAX;
static int16_t routeSent[4];
static uint8_t routeSentValid;
static void sendRouteSpeeds(int16_t forward, int16_t right, int16_t turn);
static void printHeadingPidTrace(float targetYaw, float actualYaw, int16_t turn);
static void serialSendString(const char *text);
static void serialSendUint32(uint32_t value);
static MechanismState mechanismState;
static AxisFeedback mechanismFeedback[2];
static uint8_t mechanismAxes, mechanismPreparing, mechanismPollAxis, mechanismPollKind[2];
static uint32_t mechanismPollAt, mechanismFeedbackTimeout;
static int32_t mechanismBaseline[2];
static uint8_t mechanismBaselineMask, mechanismEnabledAxes, mechanismPositionPriority;
static int32_t mechanismSignedPulses[2];
static const MechanismInitialState *mechanismActionInitial;
static const MechanismAction *mechanismActions;
static uint16_t mechanismActionCount, mechanismActionIndex;
static uint8_t mechanismActionActive, mechanismActionDispatched;
static MechanismAction axis6TestActions[2];
static uint8_t axis6TestActive(void)
{
    return mechanismActionActive && mechanismActions == axis6TestActions;
}
static uint32_t mechanismActionWaitUntil;
static uint8_t rawPickPlatformSlot;
static uint32_t rawPickPlatformReadyAt;
static VisionSession visionSession;
static VisionParser visionParser;
static volatile VisionPacket visionRxPacket;
static volatile uint8_t visionRxReady;
static volatile uint32_t visionRxStamp;
static uint8_t visionTrackingActive(void);
static void cancelVisionTracking(void);
static QrScanParser qrParser;
static ObstacleStopParser obstacleParser;
static TjcObstacleParser tjcObstacleParser;
static volatile uint16_t tjcObstaclePending;
static uint8_t obstacleTurretPending, obstacleTurretReady;
static uint8_t prepareObstacleTurret(void);
static volatile uint8_t obstacleStopPending;
static AvRun avoidance;
static AvParser avoidanceParser;
static volatile AvObservation avoidanceRxPacket;
static volatile uint8_t avoidanceRxReady;
static uint8_t processAvoidanceCommand(const char *command);
static void serviceAvoidance(void);
static volatile uint8_t qrRxReady;
static volatile char qrRxPayload[QR_SCAN_PAYLOAD_MAX + 1U];
static uint8_t qrAtStation, qrResultAtStation;
static uint8_t visionMotionAutomatic;
static uint8_t visionMotionMotorMask, visionMotionQueryId;
static uint8_t visionMotionSeenMask, visionMotionStatus[4];
static uint8_t visionPickActive;
static const VisionCalibration visionPickCalibration = {
    1U, {0, 0, 320, 240}, 160, 120,
    {0.0f, 0.0f, 0.0f, 0.0f}
};

static void serviceVision(void);
static void serviceRawPickRoute(void);
static void serviceMission(void);
static void missionBeginStation(uint8_t nextIndex);
static void missionAbort(const char *reason);
static void cancelMissionObservation(void);
static uint8_t processVisionCommand(const char *command);
static uint8_t visionSessionActive(void);
static void failVision(const char *reason);

static void invalidateMechanism(const char *reason)
{
    mechanismEnabledAxes = 0U;
    if (!mechanismState.valid && !mechanismState.running) return;
    mechanismStateInvalidate(&mechanismState);
    serialSendString("MECH INVALID reason=");
    serialSendString(reason);
    serialSendString("\r\n");
}

static void serialSendChar(char value) { hostLogChar(value); }

static void serialSendString(const char *text)
{
    while (*text != '\0') {
        serialSendChar(*text++);
    }
    while (USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET) {
    }
}







static void serialSendUint(uint16_t value)
{
    serialSendUint32(value);
}

static void serialSendInt(int16_t value)
{
    if (value < 0) {
        serialSendChar('-');
        value = (int16_t)-value;
    }
    serialSendUint((uint16_t)value);
}

static void invalidateNavigation(const char *reason)
{
    if (!navInitialized) return;
    navInitialized = 0U;
    navRunning = 0U;
    navPathCount = 0U;
    serialSendString("NAV INVALID reason=");
    serialSendString(reason);
    serialSendString("\r\n");
}

static void missionButtonInit(void)
{
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);
    missionButtonRawHigh = missionButtonStableHigh =
        GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) != 0U;
    /* A button held during boot must be released before it can start a task. */
    missionButtonLongSent = !missionButtonRawHigh;
    missionButtonChangedAt = missionButtonPressedAt = clockMs;
}

static void serialInit(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC |
                           RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = COMMAND_BAUD_RATE;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &usart);

    nvic.NVIC_IRQChannel = UART5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2U;
    nvic.NVIC_IRQChannelSubPriority = 2U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART5, ENABLE);
}

static void serialSendUint32(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char)('0' + value % 10U);
        value /= 10U;
    } while (value != 0U);

    while (count > 0U) {
        serialSendChar(digits[--count]);
    }
}

static void visionUartInit(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);
    USART_StructInit(&uart);
    uart.USART_BaudRate = 9600U;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &uart);
    nvic.NVIC_IRQChannel = UART4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic.NVIC_IRQChannelSubPriority = 1U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    vision_parser_init(&visionParser);
    visionRxReady = 0U;
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART4, ENABLE);
}

static void visionUartSend(const VisionPacket *packet)
{
    uint8_t frame[VISION_FRAME_SIZE];
    uint8_t index;
    vision_packet_encode(packet, frame);
    for (index = 0U; index < VISION_FRAME_SIZE; ++index) {
        while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET) {
        }
        USART_SendData(UART4, frame[index]);
    }
}

static void qrUartInit(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);
    USART_StructInit(&uart);
    uart.USART_BaudRate = 115200U;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &uart);
    nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic.NVIC_IRQChannelSubPriority = 2U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    qr_scan_init(&qrParser);
    qrRxReady = 0U;
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);
}

static void tjcDisplayInit(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);
    USART_StructInit(&uart);
    uart.USART_BaudRate = 115200U;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART6, &uart);
    nvic.NVIC_IRQChannel = USART6_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART6, ENABLE);
}

void USART6_IRQHandler(void)
{
    uint16_t regions;
    uint8_t byte;
    if (USART_GetITStatus(USART6, USART_IT_RXNE) == RESET) return;
    byte = (uint8_t)USART_ReceiveData(USART6);
    regions = tjc_obstacles_feed(&tjcObstacleParser, byte);
    if (regions != 0U) tjcObstaclePending = regions;
    USART_ClearITPendingBit(USART6, USART_IT_RXNE);
}

static void tjcDisplaySend(const uint8_t *command, uint8_t length)
{
    uint8_t index;
    for (index = 0U; index < length; ++index) {
        while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET) {
        }
        USART_SendData(USART6, command[index]);
    }
}

static void tjcDisplayQr(const char *result)
{
    uint8_t command[TJC_DISPLAY_COMMAND_MAX];
    uint8_t length = tjc_display_qr_command(result, command);
    tjcDisplaySend(command, length);
}

static void tjcDisplayCounts(void)
{
    uint8_t command[TJC_DISPLAY_COMMAND_MAX];
    uint8_t length = tjc_display_count_command(1U, missionGrabCount, command);
    tjcDisplaySend(command, length);
    length = tjc_display_count_command(2U, missionPlaceCount, command);
    tjcDisplaySend(command, length);
}

static void tjcDisplayResetCounts(void)
{
    missionGrabCount = missionPlaceCount = 0U;
    tjcDisplayCounts();
}

static void serviceTjcObstacles(void)
{
    static const uint8_t showTask[] = "page page0\xFF\xFF\xFF";
    uint16_t regions;
    uint8_t i;
    if (tjcObstaclePending == 0U) return;
    __disable_irq();
    regions = tjcObstaclePending;
    tjcObstaclePending = 0U;
    __enable_irq();
    if (regions == 0U) return;
    if (!prepareObstacleTurret()) {
        serialSendString("HMI OBSTACLES rejected: busy or stopped; resend selection\r\n");
        return;
    }
    routeObstacleRegions = regions;
    serialSendString("HMI OBSTACLES regions=");
    for (i = 0U; i < 15U; ++i) {
        if ((routeObstacleRegions & (1U << i)) != 0U) {
            serialSendUint(i + 1U);
            serialSendChar(' ');
        }
    }
    serialSendString("\r\n");
    /* Receipt acknowledgement; both inputs share the same RAM-only selection. */
    tjcDisplaySend(showTask, (uint8_t)(sizeof(showTask) - 1U));
    tjcDisplayCounts();
}

void USART3_IRQHandler(void)
{
    uint8_t byte;
    char result[QR_SCAN_PAYLOAD_MAX + 1U];
    uint8_t index;
    if (USART_GetITStatus(USART3, USART_IT_RXNE) == RESET) return;
    byte = (uint8_t)USART_ReceiveData(USART3);
    {
        AvObservation packet;
        if (av_parser_feed(&avoidanceParser, byte, &packet) && avoidance.active &&
            packet.token == avoidance.token && packet.region == avoidance.region) {
            if (!avoidanceRxReady || !avoidanceRxPacket.seen) {
                avoidanceRxPacket = packet;
                avoidanceRxReady = 1U;
            }
        }
    }
    if (obstacle_stop_feed(&obstacleParser, byte)) {
        obstacleStopPending = 1U;
        emergencyStop = 1U;
    }
    if (qr_scan_feed(&qrParser, byte, result) && !qrRxReady) {
        for (index = 0U; result[index] != '\0'; ++index)
            qrRxPayload[index] = result[index];
        qrRxPayload[index] = '\0';
        qrRxReady = 1U;
    }
    USART_ClearITPendingBit(USART3, USART_IT_RXNE);
}

static void serviceQrScan(void)
{
    char result[QR_SCAN_PAYLOAD_MAX + 1U];
    uint8_t index;
    if (!qrRxReady) return;
    __disable_irq();
    for (index = 0U; index <= QR_SCAN_PAYLOAD_MAX; ++index) {
        result[index] = qrRxPayload[index];
        if (result[index] == '\0') break;
    }
    qrRxReady = 0U;
    __enable_irq();
    serialSendString("QR RESULT value=");
    serialSendString(result);
    serialSendString("\r\n");
    tjcDisplayQr(result);
    if (!missionQrValid && (!missionActive ||
        (!missionTaskValid && routeIndex <= 2U))) {
        MissionTask scanned;
        if (missionTaskParse(result, &scanned)) {
            memcpy(missionQrCode, result, sizeof(missionQrCode));
            missionQrValid = 1U;
            serialSendString("QR TASK cached\r\n");
        } else {
            serialSendString("QR INVALID expected=CCC+PPP+CCC+PPP\r\n");
        }
    }
    if (qrAtStation && ((!missionActive && !arcRoutePhase) || missionQrValid)) qrResultAtStation = 1U;
}

void UART4_IRQHandler(void)
{
    uint8_t byte;
    VisionPacket packet;
    if (USART_GetITStatus(UART4, USART_IT_RXNE) == RESET) return;
    byte = (uint8_t)USART_ReceiveData(UART4);
    if (vision_parser_feed(&visionParser, &byte, 1U, &packet)) {
        visionRxPacket = packet;
        visionRxStamp = clockMs;
        visionRxReady = 1U;
    }
    USART_ClearITPendingBit(UART4, USART_IT_RXNE);
}

static void stopDriveMotors(void)
{
    uint8_t id;

    for (id = MOTOR_MIN_ID; id <= MOTOR_MAX_ID; ++id) {
        Emm_V5_Stop_Now(id, false);
        delay_ms(2U);
    }
}

static void stopAuxMotors(void)
{
    uint8_t id;

    mechanismEnabledAxes = 0U;

    for (id = AUX_MOTOR_MIN_ID; id <= AUX_MOTOR_MAX_ID; ++id) {
        Emm_V5_Stop_Now(id, false);
        delay_ms(2U);
    }
}

static void stopAllMotors(void)
{
    cancelVisionTracking();
    cancelMissionObservation();
    obstacleTurretPending = obstacleTurretReady = 0U;
    if (visionSession.state == VISION_STATE_REQUEST ||
        visionSession.state == VISION_STATE_WAIT ||
        visionSession.state == VISION_STATE_MOVE ||
        visionSession.state == VISION_STATE_SETTLE ||
        visionSession.state == VISION_STATE_PREP ||
        visionSession.state == VISION_STATE_WAIT_ACTIONS ||
        visionSession.state == VISION_STATE_PICK || motionMode == 4U) {
        vision_session_pause(&visionSession);
    }
    visionPickActive = 0U;
    invalidateMechanism("stopped");
    mechanismActionActive = 0U;
    motionMode = 0U;
    routeRawDownCorrection = 0;
    arcRoutePhase = 0U;
    missionArcActive = missionArcExit = 0U;
    routeActive = 0U;
    fullRouteRunning = 0U;
    presetRouteActive = 0U;
    presetRoute.count = presetRoute.index = 0U;
    missionButtonReady = 0U;
    missionOffline = 0U;
    missionQrValid = 0U;
    missionActive = missionActionStarted = 0U;
    missionPhase = MISSION_IDLE;
    missionOffsetForward = missionOffsetRight = 0.0f;
    rawPickRouteActive = rawPickAtStation = rawPickItemIndex = 0U;
    rawPickMissingReported = 0U;
    rawPickCurrentColor = rawPickCurrentSlot = rawPickCollectedMask = 0U;
    routeWaiting = 0U;
    routeRotating = routeOnlyTurn = routeTurnInBand = 0U;
    routeStationAlignment = 0U;
    routeDriveRpm = routeTurnRpm = routeCorrection = 0.0f;
    routeHeadingPidReset(&routeHeadingPid);
    routeSentValid = 0U;
    routeForward = routeRight = 0;
    navInitialized = 0U;
    navRunning = 0U;
    navPathCount = 0U;
    stopDriveMotors();
    stopAuxMotors();
    armed = 0U;
}

static void setAllMotorsEnabled(bool enabled)
{
    uint8_t id;

    for (id = MOTOR_MIN_ID; id <= MOTOR_MAX_ID; ++id) {
        Emm_V5_En_Control(id, enabled, false);
        delay_ms(2U);
    }
}

static uint8_t parseUint(const char **cursor, uint16_t *value)
{
    uint32_t result = 0U;
    uint8_t digitCount = 0U;

    while (**cursor == ' ') {
        (*cursor)++;
    }
    while (**cursor >= '0' && **cursor <= '9') {
        result = result * 10U + (uint32_t)(**cursor - '0');
        if (result > 65535U) {
            return 0U;
        }
        (*cursor)++;
        digitCount++;
    }
    if (digitCount == 0U) {
        return 0U;
    }
    *value = (uint16_t)result;
    return 1U;
}

static uint16_t servoAngleToPulse(uint8_t channel, uint32_t angleMdeg)
{
    uint32_t maximumAngleMdeg = channel == 4U ? 360000U : 270000U;

    return (uint16_t)(SERVO_MIN_PULSE_US +
                      angleMdeg * SERVO_PULSE_RANGE_US / maximumAngleMdeg);
}

static void servoTimerInit(void)
{
    TIM_TimeBaseInitTypeDef timeBase;
    NVIC_InitTypeDef nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructInit(&timeBase);
    timeBase.TIM_Prescaler = 84U - 1U;
    timeBase.TIM_Period = SERVO_PERIOD_US - 1U;
    timeBase.TIM_CounterMode = TIM_CounterMode_Up;
    timeBase.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &timeBase);
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    servoTimerReady = 1U;
}

static void servoGpioEnable(uint8_t channel)
{
    GPIO_InitTypeDef gpio;
    uint16_t pin;
    uint8_t pinSource;

    if (channel == 2U) {
        pin = GPIO_Pin_1;
        pinSource = GPIO_PinSource1;
    } else if (channel == 3U) {
        pin = GPIO_Pin_2;
        pinSource = GPIO_PinSource2;
    } else {
        pin = GPIO_Pin_3;
        pinSource = GPIO_PinSource3;
    }

    GPIO_PinAFConfig(GPIOA, pinSource, GPIO_AF_TIM2);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = pin;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);
}

static void servoSetPulse(uint8_t channel, uint16_t pulse)
{
    TIM_OCInitTypeDef outputCompare;
    uint8_t index = (uint8_t)(channel - 2U);

    if (!servoTimerReady) {
        servoTimerInit();
    }
    if (!servoChannelEnabled[index]) {
        TIM_OCStructInit(&outputCompare);
        outputCompare.TIM_OCMode = TIM_OCMode_PWM1;
        outputCompare.TIM_OutputState = TIM_OutputState_Enable;
        outputCompare.TIM_OCPolarity = TIM_OCPolarity_High;
        outputCompare.TIM_Pulse = pulse;

        if (channel == 2U) {
            TIM_OC2Init(TIM2, &outputCompare);
            TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
        } else if (channel == 3U) {
            TIM_OC3Init(TIM2, &outputCompare);
            TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
        } else {
            TIM_OC4Init(TIM2, &outputCompare);
            TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
        }
        TIM_GenerateEvent(TIM2, TIM_EventSource_Update);
        servoGpioEnable(channel);
        servoChannelEnabled[index] = 1U;
    } else if (channel == 2U) {
        TIM_SetCompare2(TIM2, pulse);
    } else if (channel == 3U) {
        TIM_SetCompare3(TIM2, pulse);
    } else {
        TIM_SetCompare4(TIM2, pulse);
    }
}

static uint8_t servoSetTargetMdeg(uint8_t channel, uint32_t target,
                                  uint16_t speedDps10)
{
    uint8_t index = (uint8_t)(channel - 2U);
    uint32_t step = (uint32_t)speedDps10 * 100U / SERVO_UPDATE_HZ;
    if (channel == 2U) {
        gripperMayHoldMaterial = target < (uint32_t)rawPickInitial.gripperOpenDeg * 1000U;
    }
    if (channel == 4U && !gripperMayHoldMaterial) {
        speedDps10 = SERVO_EMPTY_TURRET_SPEED_DPS10;
        step = (uint32_t)speedDps10 * 100U / SERVO_UPDATE_HZ;
    }
    if (step == 0U) step = 1U;

    if (!servoChannelEnabled[index]) {
        servoSetPulse(channel, servoAngleToPulse(channel, target));
        __disable_irq();
        currentAngleMdeg[index] = target;
        targetAngleMdeg[index] = target;
        servoStepMdeg[index] = step;
        servoChannelMoving[index] = 0U;
        __enable_irq();
        return 0U;
    }

    __disable_irq();
    if (channel == 4U) {
        uint32_t current = currentAngleMdeg[index];
        uint32_t delta = target > current ? target - current : current - target;
        turretStartAngleMdeg = current;
        turretElapsedMs = 0U;
        turretDurationMs = turretMotionDurationMs(delta, speedDps10);
    }
    targetAngleMdeg[index] = target;
    servoStepMdeg[index] = step;
    servoChannelMoving[index] = currentAngleMdeg[index] != target;
    completedServoChannels &= (uint8_t)~(1U << index);
    __enable_irq();
    return servoChannelMoving[index];
}

static uint8_t servoSetTarget(uint8_t channel, uint16_t angle,
                              uint16_t speedDps10)
{
    return servoSetTargetMdeg(channel, (uint32_t)angle * 1000U,
                              speedDps10);
}

static void stopServoMotion(void)
{
    uint8_t index;

    __disable_irq();
    obstacleTurretPending = obstacleTurretReady = 0U;
    for (index = 0U; index < 3U; ++index) {
        targetAngleMdeg[index] = currentAngleMdeg[index];
        servoChannelMoving[index] = 0U;
    }
    completedServoChannels = 0U;
    __enable_irq();
}

static void serviceHostWatchdog(void)
{
    uint8_t hadNavigation, hadFullRoute, hadVision;
    if ((missionOffline && fullRouteRunning) ||
        (missionButtonReady && !motionMode && !routeActive &&
         !mechanismState.running && !mechanismActionActive && !visionSessionActive() &&
         !servoChannelMoving[0] && !servoChannelMoving[1] && !servoChannelMoving[2])) return;
    if (!hostHeartbeatActive ||
        clockMs - hostHeartbeatStamp <= HOST_HEARTBEAT_TIMEOUT_MS) {
        return;
    }

    hostHeartbeatActive = 0U;
    hadNavigation = navInitialized;
    hadFullRoute = fullRouteRunning || missionButtonReady;
    hadVision = visionSessionActive();
    stopServoMotion();
    stopAllMotors();
    if (hadNavigation) serialSendString("NAV INVALID reason=heartbeat_timeout\r\n");
    if (hadFullRoute) serialSendString("ROUTE INVALID reason=heartbeat_timeout\r\n");
    if (hadVision) failVision("HEARTBEAT_TIMEOUT");
    serialSendString("ERR: host heartbeat timeout; stopped\r\n");
}

static void reportCompletedServoMoves(void)
{
    uint8_t completed;
    uint8_t index;

    __disable_irq();
    completed = completedServoChannels;
    completedServoChannels = 0U;
    __enable_irq();

    for (index = 0U; index < 3U; ++index) {
        if ((completed & (uint8_t)(1U << index)) != 0U) {
            serialSendString("DONE servo=");
            serialSendUint((uint16_t)(index + 2U));
            serialSendString(" angle=");
            serialSendUint((uint16_t)(currentAngleMdeg[index] / 1000U));
            serialSendString("\r\n");
        }
    }
}

void TIM2_IRQHandler(void)
{
    uint8_t index;

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == RESET) {
        return;
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    for (index = 0U; index < 3U; ++index) {
        uint32_t current;
        uint32_t target;

        if (!servoChannelEnabled[index] || !servoChannelMoving[index]) {
            continue;
        }
        current = currentAngleMdeg[index];
        target = targetAngleMdeg[index];
        if (index == 2U) {
            turretElapsedMs += 20U;
            current = turretMotionAngleMdeg(turretStartAngleMdeg, target,
                                            turretElapsedMs, turretDurationMs);
        } else if (current < target) {
            current = target - current <= servoStepMdeg[index] ?
                      target : current + servoStepMdeg[index];
        } else {
            current = current - target <= servoStepMdeg[index] ?
                      target : current - servoStepMdeg[index];
        }
        currentAngleMdeg[index] = current;
        servoSetPulse((uint8_t)(index + 2U),
                      servoAngleToPulse((uint8_t)(index + 2U), current));
        if (current == target &&
            (index != 2U || turretElapsedMs >= turretDurationMs)) {
            servoChannelMoving[index] = 0U;
            completedServoChannels |= (uint8_t)(1U << index);
        }
    }
}

static uint8_t motionInterrupted(void)
{
    return emergencyStop != 0U || jogCanFault != 0U;
}

void TIM7_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
        ++clockMs;
    }
}

static void clockInit(void)
{
    TIM_TimeBaseInitTypeDef timer;
    NVIC_InitTypeDef nvic;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 84U - 1U;
    timer.TIM_Period = 1000U - 1U;
    TIM_TimeBaseInit(TIM7, &timer);
    nvic.NVIC_IRQChannel = TIM7_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM7, ENABLE);
}

static uint8_t visionSessionActive(void)
{
    return visionSession.state == VISION_STATE_REQUEST ||
           visionSession.state == VISION_STATE_WAIT ||
           visionSession.state == VISION_STATE_MOVE ||
           visionSession.state == VISION_STATE_SETTLE ||
           visionSession.state == VISION_STATE_PREP ||
           visionSession.state == VISION_STATE_WAIT_ACTIONS ||
           visionSession.state == VISION_STATE_PICK;
}

static const VisionCalibration *visionCalibration(void)
{
    if (visionPickActive) return rawPickAtStation ?
        &visionMaterialCalibration : &visionPickCalibration;
    if (missionActive && missionPhase == MISSION_TEMP_ALIGN && routeIndex == 16U) {
        return &visionRingCalibration[1];
    }
    if (visionSession.mode == VISION_MODE_MATERIAL) {
        return &visionMaterialCalibration;
    }
    if (visionSession.target >= 1U && visionSession.target <= 3U) {
        return &visionRingCalibration[visionSession.target - 1U];
    }
    return (const VisionCalibration *)0;
}

static uint8_t rawPickPendingMask(void)
{
    const uint8_t *colors = missionActive ? missionTask.colors[routeIndex == 11U] :
                                          rawPickColors[routeIndex == 11U];
    uint8_t index, mask = 0U;
    for (index = 0U; index < 3U; ++index) {
        uint8_t bit;
        if (colors[index] < 1U || colors[index] > 6U) return 0U;
        bit = (uint8_t)(1U << (colors[index] - 1U));
        if (mask & bit) return 0U;
        mask |= bit;
    }
    return (uint8_t)(mask & (uint8_t)~rawPickCollectedMask);
}

static void printRawPickState(const char *state, uint8_t color, uint8_t slot)
{
    serialSendString("ROUTE RAWPICK state="); serialSendString(state);
    serialSendString(" color="); serialSendUint(color);
    serialSendString(" slot="); serialSendUint(slot);
    serialSendString("\r\n");
}

static const char *visionStateName(VisionState state)
{
    switch (state) {
    case VISION_STATE_REQUEST: return "REQUEST";
    case VISION_STATE_WAIT: return "WAIT";
    case VISION_STATE_MOVE: return "MOVE";
    case VISION_STATE_SETTLE: return "SETTLE";
    case VISION_STATE_ALIGNED: return "ALIGNED";
    case VISION_STATE_FAILED: return "FAILED";
    case VISION_STATE_PAUSED: return "PAUSED";
    case VISION_STATE_PREP: return "PREP";
    case VISION_STATE_PICK: return "PICK";
    case VISION_STATE_PICK_DONE: return "PICK_DONE";
    case VISION_STATE_WAIT_ACTIONS: return "WAIT_ACTIONS";
    case VISION_STATE_CORRECTION_LIMIT: return "CORRECTION_LIMIT";
    default: return "IDLE";
    }
}

static void serialSendFloat1(float value)
{
    uint16_t integer;
    uint16_t fraction;
    if (value < 0.0f) {
        serialSendChar('-');
        value = -value;
    }
    integer = (uint16_t)value;
    fraction = (uint16_t)((value - integer) * 10.0f + 0.5f);
    if (fraction >= 10U) {
        ++integer;
        fraction = 0U;
    }
    serialSendUint(integer);
    serialSendChar('.');
    serialSendUint(fraction);
}

static void printVisionState(void)
{
    serialSendString("VISION STATE state=");
    serialSendString(visionStateName(visionSession.state));
    serialSendString(" mode=");
    serialSendString(visionSession.mode == VISION_MODE_RING ? "RING" : "MATERIAL");
    serialSendString(" selector=");
    serialSendUint(visionSession.selector);
    serialSendString(" target=");
    serialSendUint(visionSession.target);
    serialSendString(" iteration=");
    serialSendUint(visionSession.iteration);
    serialSendString("\r\n");
}

static void failVision(const char *reason)
{
    uint8_t wasRawPick = rawPickAtStation;
    uint8_t wasMission = missionActive;
    uint8_t slot = wasRawPick ? rawPickCurrentSlot : 0U;
    uint8_t color = wasRawPick ? rawPickCurrentColor : 0U;
    cancelVisionTracking();
    if (visionPickActive || wasMission) {
        stopServoMotion();
        stopAllMotors();
    }
    if (motionMode == 4U) {
        stopDriveMotors();
        motionMode = 0U;
    }
    visionSession.state = VISION_STATE_FAILED;
    visionSession.fault = reason;
    serialSendString("VISION ERROR reason=");
    serialSendString(reason);
    serialSendString("\r\n");
    if (wasRawPick) {
        serialSendString("ROUTE RAWPICK state=ERROR color=");
        serialSendUint(color);
        serialSendString(" slot=");
        serialSendUint(slot);
        serialSendString("\r\n");
    }
    if (wasMission) serialSendString("ROUTE INVALID reason=mission_vision\r\n");
    else if (wasRawPick) serialSendString("ROUTE INVALID reason=rawpick_vision\r\n");
}

static void printVisionMotionStatus(void)
{
    uint8_t id;
    serialSendString("VISION MOVE STATUS elapsed_ms="); serialSendUint32(clockMs - motionStart);
    serialSendString(" limit_ms="); serialSendUint32(motionDuration);
    serialSendString(" motor_mask="); serialSendUint32(visionMotionMotorMask);
    serialSendString(" seen_mask="); serialSendUint32(visionMotionSeenMask);
    for (id = 1U; id <= 4U; ++id) {
        serialSendString(" m"); serialSendUint32(id);
        serialSendString("="); serialSendUint32(visionMotionStatus[id - 1U]);
    }
    serialSendString("\r\n");
}

#include "vision_tracking_runtime.h"

static uint8_t startVisionMotion(float forwardMm, float rightMm,
                                 uint16_t rpm, uint8_t automatic)
{
    float forward = forwardMm / routeForwardScale;
    float right = rightMm / routeLateralScale;
    float wheelMm[4];
    uint32_t maximumPulses = 0U;
    uint8_t id;
    uint8_t moved = 0U;
    if (automatic && visionSession.mode == VISION_MODE_RING &&
        visionSession.iteration >= 2U) rpm = 10U;
    visionMotionMotorMask = visionMotionQueryId = 0U;
    wheelMm[0] = forward - right;
    wheelMm[1] = forward + right;
    wheelMm[2] = forward - right;
    wheelMm[3] = forward + right;
    setAllMotorsEnabled(true);
    if (motionInterrupted()) return 0U;
    for (id = 1U; id <= 4U; ++id) {
        float distance = wheelMm[id - 1U];
        uint8_t direction = motorDirections[DIRECTION_FORWARD][id] ^ motorInvert[id];
        uint32_t pulses;
        if (distance < 0.0f) {
            distance = -distance;
            direction ^= 1U;
        }
        pulses = (uint32_t)(distance * 3200.0f / ROUTE_MM_PER_REV + 0.5f);
        if (pulses == 0U) {
            Emm_V5_Stop_Now(id, false);
            continue;
        }
        if (pulses > maximumPulses) maximumPulses = pulses;
        moved = 1U;
        visionMotionMotorMask |= (uint8_t)(1U << (id - 1U));
        Emm_V5_Pos_Control(id, direction,
                           (uint16_t)((uint32_t)rpm * motorTrim[id] / 1000U),
                           MOTOR_TEST_ACCEL, pulses, false, true);
        delay_ms(2U);
    }
    if (!moved || motionInterrupted()) return 0U;
    Emm_V5_Synchronous_motion(0x00);
    invalidateNavigation("vision_motion");
    motionStart = clockMs;
    if (visionSession.mode == VISION_MODE_RING) {
        lastControl = motionStart;
        visionMotionSeenMask = 0U;
        memset(visionMotionStatus, 0, sizeof(visionMotionStatus));
        jogCanResetDriveStatus();
        __disable_irq();
        can.rxFrameFlag = false;
        __enable_irq();
    }
    /* Both visual modes use timed completion; ring status checks faults only. */
    motionDuration = maximumPulses * 60000U / (3200U * rpm) + 800U;
    motionMode = 4U;
    visionMotionAutomatic = automatic;
    if (missionActive && routeWaiting) {
        missionOffsetForward += forwardMm;
        missionOffsetRight += rightMm;
    }
    return 1U;
}

#include "vision_motion_runtime.h"

static uint8_t parsePair(const char *cursor, uint16_t *a, uint16_t *b)
{
    if (!parseUint(&cursor, a) || *cursor != ' ' || !parseUint(&cursor, b)) return 0U;
    while (*cursor == ' ') ++cursor;
    return *cursor == '\0';
}



static uint8_t parseValueWithOptionalRpm(const char *cursor, uint16_t *value,
                                         uint16_t *rpm)
{
    if (!parseUint(&cursor, value)) return 0U;
    if (*cursor == '\0') {
        *rpm = ROUTE_RPM;
        return 1U;
    }
    if (*cursor != ' ') return 0U;
    ++cursor;
    return parseUint(&cursor, rpm) && *cursor == '\0';
}

static void printMechanismPose(const char *prefix, const MechanismPose *pose)
{
    serialSendString(prefix);
    serialSendString(" h=");
    serialSendInt(pose->horizontalDmm);
    serialSendString(" l=");
    serialSendUint(pose->liftDmm);
    serialSendString(" t=");
    serialSendUint(pose->turretDdeg);
    serialSendString("\r\n");
}

#include "mechanism_pose_runtime.h"

#include "mechanism_runtime.h"
#include "mission_observation.h"

uint8_t mechanismActionStart(const MechanismInitialState *initial,
                             const MechanismAction *actions,
                             uint16_t count)
{
    if (initial == (const MechanismInitialState *)0 ||
        actions == (const MechanismAction *)0 || count == 0U ||
        mechanismActionActive || !mechanismState.valid ||
        mechanismState.running || motionMode ||
        (routeActive && !(routeWaiting &&
                         ((rawPickRouteActive && rawPickAtStation) || missionActive))))
        return 0U;
    mechanismActionInitial = initial;
    mechanismActions = actions;
    mechanismActionCount = count;
    mechanismActionIndex = 0U;
    mechanismActionDispatched = 0U;
    mechanismActionActive = 1U;
    mechanismActionWaitUntil = 0U;
    rawPickPlatformSlot = 0U;
    return 1U;
}

/* Counts describe completed commands, not sensor-confirmed material handling. */
static void missionCountAction(const MechanismAction *action)
{
    if (!missionActive || !mechanismState.valid ||
        action->type != MECHANISM_ACTION_GRIPPER) return;
    if (!action->value &&
        (missionPhase == MISSION_RAW_PICK || missionPhase == MISSION_COARSE_PICK)) {
        ++missionGrabCount;
    } else if (action->value &&
               (missionPhase == MISSION_COARSE_PLACE || missionPhase == MISSION_TEMP_PLACE) &&
               mechanismState.current.liftDmm != 0U &&
               mechanismState.current.turretDdeg != 1200U) {
        /* Exclude opening at the observation pose and transfers on the tray. */
        ++missionPlaceCount;
    } else {
        return;
    }
    tjcDisplayCounts();
}

static void mechanismActionCompleteStep(uint16_t waitMs)
{
    missionCountAction(&mechanismActions[mechanismActionIndex]);
    if (waitMs != 0U) {
        mechanismActionWaitUntil = clockMs + waitMs;
        return;
    }
    ++mechanismActionIndex;
    mechanismActionDispatched = 0U;
}

void mechanismActionService(void)
{
    const MechanismAction *action;
    uint16_t angle;
    uint16_t speedDps10;
    if (!mechanismActionActive) return;
    if (rawPickPlatformSlot && !servoChannelMoving[1] &&
        (int32_t)(clockMs - rawPickPlatformReadyAt) >= 0) {
        servoSetTarget(3U, rawPickInitial.platformDeg[rawPickPlatformSlot - 1U],
                       rawPickInitial.platformDps10);
        rawPickPlatformSlot = 0U;
    }
    if (mechanismActionDispatched && mechanismActionWaitUntil != 0U) {
        if ((int32_t)(clockMs - mechanismActionWaitUntil) < 0) return;
        mechanismActionWaitUntil = 0U;
        ++mechanismActionIndex;
        mechanismActionDispatched = 0U;
    }
    if (mechanismActionIndex >= mechanismActionCount) {
        if (rawPickPlatformSlot) return;
        if (axis6TestActive()) serialSendString("AXIS6 TEST DONE\r\n");
        mechanismActionActive = 0U;
        return;
    }
    action = &mechanismActions[mechanismActionIndex];
    if (!mechanismActionDispatched) {
        /* Tray descent and release wait for platform arrival. Other poses overlap. */
        if (action->type == MECHANISM_ACTION_POSE &&
            (action->pose.horizontalDmm == -500 || action->pose.horizontalDmm == -450 ||
             action->pose.horizontalDmm == -470) &&
            action->pose.turretDdeg == 1200U &&
            action->pose.liftDmm > mechanismState.current.liftDmm &&
            servoChannelMoving[1]) return;
        if (action->type == MECHANISM_ACTION_GRIPPER && action->value &&
            mechanismState.current.turretDdeg == 1200U && servoChannelMoving[1] &&
            !mechanismGripperOpenCommandComplete(mechanismActionInitial,
                servoChannelEnabled[0], servoChannelMoving[0],
                currentAngleMdeg[0], targetAngleMdeg[0])) return;
        if ((action->type == MECHANISM_ACTION_PLATFORM ||
             (action->type == MECHANISM_ACTION_SERVO && action->channel == 3U)) &&
            servoChannelMoving[1]) return;
        if (action->type == MECHANISM_ACTION_POSE) {
            if (mechanismState.running) return;
            /* Reach the lower material horizontally before descending. */
            if (mechanismActionInitial == &rawPickInitial &&
                action->pose.horizontalDmm == rawPickLowerPose.horizontalDmm &&
                action->pose.liftDmm == rawPickLowerPose.liftDmm &&
                action->pose.turretDdeg == rawPickLowerPose.turretDdeg &&
                mechanismState.current.horizontalDmm != action->pose.horizontalDmm &&
                mechanismState.current.liftDmm < action->pose.liftDmm) {
                MechanismPose approach = action->pose;
                approach.liftDmm = mechanismState.current.liftDmm;
                startMechanismPose(&approach);
                return;
            }
            startMechanismPose(&action->pose);
        } else if (action->type == MECHANISM_ACTION_GRIPPER) {
            angle = action->value ? mechanismActionInitial->gripperOpenDeg :
                                    mechanismActionInitial->gripperCloseDeg;
            servoSetTarget(2U, angle, mechanismActionInitial->gripperDps10);
            if (mechanismActionInitial == &rawPickInitial && action->value &&
                mechanismState.current.turretDdeg == 1200U) {
                serialSendString("RAW RELEASE target_deg=");
                serialSendUint(angle);
                serialSendString(" step=");
                serialSendUint(mechanismActionIndex);
                serialSendString("\r\n");
            }
        } else if (action->type == MECHANISM_ACTION_PLATFORM) {
            if (action->value < 1U || action->value > 3U) {
                mechanismActionActive = 0U;
                return;
            }
            if (mechanismActionInitial == &rawPickInitial &&
                mechanismActionIndex > 0U &&
                mechanismActions[mechanismActionIndex - 1U].type == MECHANISM_ACTION_GRIPPER &&
                mechanismActions[mechanismActionIndex - 1U].value) {
                /* Let the released material settle while the arm returns. */
                rawPickPlatformSlot = (uint8_t)action->value;
                rawPickPlatformReadyAt = clockMs + 500U;
            } else {
                servoSetTarget(3U,
                    mechanismActionInitial->platformDeg[action->value - 1U],
                    mechanismActionInitial->platformDps10);
            }
        } else if (action->type == MECHANISM_ACTION_SERVO) {
            if (action->channel < 2U || action->channel > 4U ||
                (action->channel < 4U && action->value > 270U) ||
                (action->channel == 4U && action->value > 360U)) {
                mechanismActionActive = 0U;
                return;
            }
            speedDps10 = action->channel == 2U ?
                mechanismActionInitial->gripperDps10 :
                (action->channel == 3U ?
                    mechanismActionInitial->platformDps10 :
                    mechanismActionInitial->pose.turretDps10);
            servoSetTarget(action->channel, action->value, speedDps10);
        } else if (action->type != MECHANISM_ACTION_WAIT) {
            mechanismActionActive = 0U;
            return;
        }
        mechanismActionDispatched = 1U;
        if (!mechanismActionActive) return;
        if (action->type == MECHANISM_ACTION_WAIT) {
            mechanismActionWaitUntil = clockMs + action->waitMs;
            return;
        }
    }
    if (action->type == MECHANISM_ACTION_POSE && mechanismState.running) return;
    if ((action->type == MECHANISM_ACTION_GRIPPER ||
         action->type == MECHANISM_ACTION_PLATFORM ||
         action->type == MECHANISM_ACTION_SERVO) &&
        servoChannelMoving[action->channel - 2U] &&
        !(action->channel == 3U && action->waitMs == 0U)) return;
    if (mechanismActionInitial == &rawPickInitial &&
        action->type == MECHANISM_ACTION_GRIPPER && action->value &&
        mechanismState.current.turretDdeg == 1200U) {
        serialSendString("RAW RELEASE pwm_deg=");
        serialSendUint((uint16_t)(currentAngleMdeg[0] / 1000U));
        serialSendString(" step=");
        serialSendUint(mechanismActionIndex);
        serialSendString("\r\n");
    }
    mechanismActionCompleteStep(action->waitMs);
}



static uint8_t processAuxMoveCommand(const char *command)
{
    const char *cursor;
    uint16_t motorId, rpm, accel;
    int16_t distanceDmm;
    int16_t maximumDmm;

    if (strncmp(command, "auxmove ", 8U) != 0) return 0U;
    cursor = command + 8U;
    if (!mechanismParseUnsigned(&cursor, &motorId) ||
        !mechanismParseSigned(&cursor, &distanceDmm) ||
        !mechanismParseUnsigned(&cursor, &rpm) ||
        !mechanismParseUnsigned(&cursor, &accel) ||
        !mechanismAtEnd(cursor)) {
        serialSendString("ERR: auxmove 5|6 signed_dmm rpm accel\r\n");
        return 1U;
    }
    if (motorId < AUX_MOTOR_MIN_ID || motorId > AUX_MOTOR_MAX_ID) {
        serialSendString("ERR: auxmove motor 5|6\r\n");
        return 1U;
    }
    maximumDmm = motorId == 5U ? 1500 : 1870;
    if (distanceDmm == 0 || distanceDmm < -maximumDmm ||
        distanceDmm > maximumDmm || rpm < MECHANISM_RPM_MIN ||
        rpm > (motorId == 5U ? MECHANISM_LIFT_RPM_MAX : MECHANISM_RPM_MAX) ||
        accel < MECHANISM_ACCEL_MIN ||
        accel > (motorId == 5U ? MECHANISM_LIFT_ACCEL_MAX : MECHANISM_ACCEL_MAX)) {
        serialSendString("ERR: auxmove distance/rpm/accel out of range\r\n");
        return 1U;
    }
    if (!armed) {
        serialSendString("ERR: send 'arm' first\r\n");
        return 1U;
    }
    armed = 0U;
    {
        MechanismPose target=mechanismState.current;
        int32_t next;
        if(!mechanismState.valid || mechanismState.running || mechanismActionActive) {
            serialSendString("ERR MECH: initialize pose first\r\n");return 1U;
        }
        next=(motorId==5U?(int32_t)target.liftDmm:target.horizontalDmm)+distanceDmm;
        if(motorId==5U) {
            if(next<0 || next>MECHANISM_LIFT_MAX_DMM) {serialSendString("ERR MECH: lift limit\r\n");return 1U;}
            target.liftDmm=(uint16_t)next;target.liftRpm=rpm;target.liftAccel=(uint8_t)accel;
        } else {
            if(next<MECHANISM_HORIZONTAL_MIN_DMM || next>MECHANISM_HORIZONTAL_MAX_DMM) {serialSendString("ERR MECH: horizontal limit\r\n");return 1U;}
            target.horizontalDmm=(int16_t)next;target.horizontalRpm=rpm;target.horizontalAccel=(uint8_t)accel;
        }
        startMechanismPose(&target);
    }
    return 1U;
}



static void sendRouteSpeeds(int16_t forward, int16_t right, int16_t turn)
{
    uint8_t id, direction;
    uint8_t changed = (uint8_t)!routeSentValid;
    int16_t speed, speeds[4];
    for (id = 1U; id <= 4U; ++id) {
        speeds[id - 1U] = routeWheel(id, forward, right, turn, motorTrim[id]);
    }
    if (motionMode == 6U || motionMode == 7U || arcRoutePhase || missionArcExit) {
        int16_t peak = 250;
        for (id = 0U; id < 4U; ++id) {
            int16_t magnitude = speeds[id] < 0 ? -speeds[id] : speeds[id];
            if (magnitude > peak) peak = magnitude;
        }
        if (peak > 250) {
            for (id = 0U; id < 4U; ++id)
                speeds[id] = (int16_t)((int32_t)speeds[id] * 250 / peak);
        }
    }
    for (id = 0U; id < 4U; ++id) {
        if (speeds[id] != routeSent[id]) changed = 1U;
    }
    /* A velocity command persists in the drive. Re-send the synchronized batch
     * only when a wheel target changes, not every iteration at constant speed. */
    if (!changed) return;
    for (id = 1U; id <= 4U; ++id) {
        if (motionInterrupted()) return;
        speed = speeds[id - 1U];
        direction = motorDirections[0][id] ^ motorInvert[id];
        if (speed < 0) { speed = -speed; direction ^= 1U; }
        Emm_V5_Vel_Control(id, direction, (uint16_t)speed, 0U, true);
    }
    if (!motionInterrupted()) Emm_V5_Synchronous_motion(0x00);
    if (!motionInterrupted()) {
        for (id = 0U; id < 4U; ++id) routeSent[id] = speeds[id];
        routeSentValid = 1U;
    }
}



static void printHeadingPidTrace(float targetYaw, float actualYaw, int16_t turn)
{
    serialSendString("PID TRACE target_cdeg=");
    serialSendInt((int16_t)(headingError(targetYaw, 0.0f) * 100.0f));
    serialSendString(" actual_cdeg=");
    serialSendInt((int16_t)(actualYaw * 100.0f));
    serialSendString(" output_rpm=");
    serialSendInt(turn);
    serialSendString("\r\n");
}

static int16_t navFieldYawCdeg(void)
{
    float fieldYaw = 90.0f + headingError(imuYaw, routeBaseYaw) * yawSign;
    while (fieldYaw > 180.0f) fieldYaw -= 360.0f;
    while (fieldYaw <= -180.0f) fieldYaw += 360.0f;
    return (int16_t)(fieldYaw * 100.0f);
}

static void printNavPose(const char *state)
{
    NavPoint target = navPathCount > 0U ? navPath[navPathCount - 1U]
                                        : navPoint(navCurrentNode);
    serialSendString("NAV POS x=");
    serialSendUint((uint16_t)(routeX + 0.5f));
    serialSendString(" y=");
    serialSendUint((uint16_t)(routeY + 0.5f));
    serialSendString(" yaw_cdeg=");
    serialSendInt(navFieldYawCdeg());
    serialSendString(" state=");
    serialSendString(state);
    serialSendString(" target_x=");
    serialSendUint((uint16_t)target.x);
    serialSendString(" target_y=");
    serialSendUint((uint16_t)target.y);
    serialSendString("\r\n");
}

static RoutePoint activeRoutePoint(void)
{
    NavPoint point = navPath[routeIndex];
    RoutePoint result;
    result.x = point.x;
    result.y = point.y;
    result.event = 0;
    result.heading = routeIndex + 1U == navPathCount ?
                     point.arrivalHeading : NAV_HEADING_KEEP;
    return result;
}

static void printArcBlendState(const char *event)
{
    serialSendString(event);
    serialSendString(" turned_cdeg="); serialSendInt(routeRound(arcBlend.turned * 100.0f));
    serialSendString(" error_cdeg="); serialSendInt(routeRound((90.0f - arcBlend.turned) * 100.0f));
    serialSendString(" target_error_cdeg=");
    serialSendInt(routeRound(headingError(arcBlend.startYaw + 90.0f * yawSign * arcBlend.direction,
                                          arcBlend.previousYaw) * yawSign * arcBlend.direction * 100.0f));
    serialSendString(" distance_mm="); serialSendUint32((uint32_t)arcBlend.distance);
    serialSendString(" elapsed_ms="); serialSendUint32(clockMs - arcBlend.start);
    serialSendString("\r\n");
}

static uint8_t missionArcCorner(uint8_t index)
{
    RouteLeg before, after;
    if (presetRouteActive) return presetRoute.legs[presetRoute.index].arc;
    if (!missionActive || navRunning || index + 1U >= ROUTE_COUNT ||
        ARC_BLEND_GEOMETRY_MM * routeForwardScale / ARC_BLEND_RADIUS_MM >= 0.98f)
        return 0U;
    if (index != 2U && index != 4U && index != 5U && index != 7U && index != 9U &&
        index != 11U && index != 12U && index != 14U) return 0U;
    /* The center lane's pickup and 80mm coarse approach keep pivot turns. */
    if (routeObstaclePosition >= ROUTE_OBSTACLE_LEFT &&
        (index == 4U || index == 5U || index == 11U || index == 12U)) return 0U;
    before = routeObstacleLeg(index, routeStartZone, routeObstaclePosition);
    after = routeObstacleLeg((uint8_t)(index + 1U), routeStartZone, routeObstaclePosition);
    if (index == 4U || index == 11U) {
        before.rightMm += routeRawLeftCorrection;
        if (before.rightMm >= -250.5f) return 0U;
    }
    return (uint8_t)(routeAbs(before.upMm ? before.upMm : before.rightMm) > 250.5f &&
                     routeAbs(after.upMm ? after.upMm : after.rightMm) > 250.5f);
}

static void beginArcBlend(void)
{
    RouteLeg leg = presetRouteActive ? presetRoute.legs[presetRoute.index].drive :
                   routeObstacleLeg(routeIndex, routeStartZone, routeObstaclePosition);
    int16_t forward, right;
    int8_t change;
    arcBlend.exitHeading = (presetRouteActive || missionActive) ? leg.heading : 1;
    change = arcBlend.exitHeading - routeHeading;
    if (change > 2) change -= 4;
    if (change < -2) change += 4;
    arcBlend.direction = change > 0 ? 1 : -1;
    arcBlend.startYaw = routeYaw;
    routeBody(leg.upMm, leg.rightMm, routeHeading, &forward, &right);
    arcBlend.travel = forward < 0 ? -1 : 1;
    arcBlend.distance = 0.0f;
    arcBlend.forward = routeDriveRpm;
    arcBlend.turn = routeCorrection;
    __disable_irq();
    arcBlend.previousYaw = imuYaw; arcBlend.sample = imuStamp;
    __enable_irq();
    arcBlend.turned = arcBlend.rateDps = 0.0f;
    arcBlend.tick = arcBlend.start = arcBlend.report = clockMs;
    routeHeadingPidReset(&routeHeadingPid);
    routeActive = (presetRouteActive || missionActive) ? 1U : 0U;
    motionMode = 7U;
    if (presetRouteActive || missionActive) {
        missionArcActive = 1U;
        missionArcExit = 0U;
        serialSendString("ROUTE ARC index="); serialSendUint((uint16_t)(routeIndex + 1U));
        serialSendString(" wheel_rpm_limit=250 profile=BLEND\r\n");
    } else {
        arcRoutePhase = 3U;
        serialSendString("ARCTEST ARC wheel_rpm_limit=250 profile=BLEND\r\n");
    }
    printArcBlendState("ARC RUN");
}

static RouteLeg activeDistanceLeg(void)
{
    RouteLeg leg = routeObstacleLeg(routeIndex, routeStartZone, routeObstaclePosition);
    if (presetRouteActive) return presetActiveLeg(&presetRoute);
    if (arcRoutePhase) {
        if (routeIndex == 2U) { leg.upMm -= 249; leg.heading = 4; }
        if (routeIndex == 3U) leg.rightMm += 249;
    }
    if (routeIndex == 4U || routeIndex == 11U) leg.rightMm += routeRawLeftCorrection;
    if ((routeObstaclePosition == ROUTE_OBSTACLE_LEFT ||
         routeObstaclePosition == ROUTE_OBSTACLE_RIGHT) &&
        (routeIndex == 5U || routeIndex == 12U)) leg.upMm -= routeRawDownCorrection;
    if (missionArcCorner(routeIndex)) {
        if (leg.upMm) leg.upMm += leg.upMm > 0 ? -249 : 249;
        else leg.rightMm += leg.rightMm > 0 ? -249 : 249;
        leg.heading = 4;
    }
    if (routeIndex && missionArcCorner((uint8_t)(routeIndex - 1U))) {
        if (leg.upMm) leg.upMm += leg.upMm > 0 ? -249 : 249;
        else leg.rightMm += leg.rightMm > 0 ? -249 : 249;
    }
    return leg;
}

static void printFullRoutePose(const char *state)
{
    RouteLeg leg = activeDistanceLeg();
    uint16_t distance = (uint16_t)routeAbs(leg.upMm ? leg.upMm : leg.rightMm);
    const char *direction = leg.upMm > 0 ? "UP" : leg.upMm < 0 ? "DOWN" :
                            leg.rightMm > 0 ? "RIGHT" : "LEFT";
    serialSendString("ROUTE DIST index=");
    serialSendUint((uint16_t)(routeIndex + 1U));
    serialSendString(" direction="); serialSendString(direction);
    serialSendString(" distance_mm="); serialSendUint(distance);
    serialSendString(" progress_mm=");
    serialSendUint((uint16_t)(routeLegProgress + 0.5f));
    serialSendString(" yaw_cdeg="); serialSendInt(navFieldYawCdeg());
    serialSendString(" state="); serialSendString(state);
    serialSendString(" stage="); serialSendString(leg.event ? leg.event : "TRANSIT");
    serialSendString("\r\n");
}

static void printFullRouteStage(const RoutePoint *point)
{
    serialSendString("ROUTE STAGE index=");
    serialSendUint((uint16_t)(routeIndex + 1U));
    serialSendString(" name=");
    serialSendString(point->event ? point->event : "TRANSIT");
    serialSendString("\r\n");
}

static void finishFullRoute(void)
{
    uint8_t wasPreset = presetRouteActive;
    stopAllMotors();
    serialSendString(wasPreset ? "ROUTE DONE mode=PRESET_REGIONS\r\n" : "ROUTE DONE mode=DISTANCE\r\n");
    serialSendString("ROUTE END: verify actual home position\r\n");
}

static void finishNavigation(void)
{
    NavPoint target = navPath[navPathCount - 1U];
    stopDriveMotors();
    routeActive = 0U;
    routeWaiting = 0U;
    routeRotating = routeOnlyTurn = routeTurnInBand = 0U;
    routeHeadingPidReset(&routeHeadingPid);
    routeForward = routeRight = 0;
    navRunning = 0U;
    navCurrentNode = navNodeAt(target.x, target.y);
    routeX = target.x;
    routeY = target.y;
    printNavPose("IDLE");
    serialSendString("NAV DONE x=");
    serialSendUint((uint16_t)target.x);
    serialSendString(" y=");
    serialSendUint((uint16_t)target.y);
    serialSendString(" yaw_cdeg=");
    serialSendInt(navFieldYawCdeg());
    serialSendString("\r\n");
}

static void serviceRoute(void)
{
    uint32_t now, dt, age, sampleMs;
    RoutePoint p;
    RouteLeg leg;
    float yaw, error, dx, dy, remaining, turnEntry, turnExit, correctionAccel, brakeRemaining;
    int16_t speed, turn, bodyForward, bodyRight;
    float pidTurn;
    uint16_t cruiseRpm;
    uint8_t lateral, bodyLateral;
    if (!routeActive || missionArcActive || motionInterrupted()) return;
    now = clockMs;
    if (routeWaiting) { routeTick = now; return; }
    dt = now - routeTick;
    if (dt < 20U) return;
    __disable_irq();
    yaw = imuYaw; sampleMs = imuStamp; age = clockMs - sampleMs;
    __enable_irq();
    error = headingError(routeYaw, yaw);
    if (!imuValid || age > 250U || (!routeRotating && routeAbs(error) > 20.0f) || dt > 150U ||
        now - routeLegStart > 45000U) {
        uint8_t wasNavigation = navRunning;
        uint8_t wasFullRoute = fullRouteRunning;
        stopAllMotors();
        if (wasNavigation) serialSendString("NAV INVALID reason=route_fault\r\n");
        if (wasFullRoute) serialSendString("ROUTE INVALID reason=route_fault\r\n");
        serialSendString(wasNavigation ?
            "ERR NAV: IMU/deviation/control delay/leg timeout; position invalid\r\n" :
            "ERR ROUTE: IMU/deviation/control delay/leg timeout; aborted\r\n");
        return;
    }
    routeTick = now;
    if (routeRotating) {
        if (navRunning && now - navLastReport >= 200U) {
            navLastReport = now;
            printNavPose("TURN");
        }
        if (fullRouteRunning && now - routeLastReport >= 200U) {
            routeLastReport = now;
            printFullRoutePose("TURN");
        }
        if (now - routeTurnStart > 18000U) {
            uint8_t wasNavigation = navRunning;
            uint8_t wasFullRoute = fullRouteRunning;
            stopAllMotors();
            if (wasNavigation) serialSendString("NAV INVALID reason=turn_timeout\r\n");
            if (wasFullRoute) serialSendString("ROUTE INVALID reason=turn_timeout\r\n");
            serialSendString(wasNavigation ?
                "ERR NAV: turn timeout; position invalid\r\n" :
                "ERR TURN: timeout; aborted\r\n");
            return;
        }
        /* Station work requires 1deg; ordinary turns retain their hysteresis. */
        turnEntry = routeStationAlignment == 1U ? 1.0f : 2.0f;
        turnExit = routeStationAlignment == 1U ? 1.0f : 3.5f;
        if (!routeTurnInBand && routeAbs(error) <= turnEntry) routeTurnInBand = 1U;
        if (routeTurnInBand && routeAbs(error) > turnExit) routeTurnInBand = 0U;
        pidTurn = routeTurnPidStep(&routeTurnPid, error, sampleMs,
                                   routeTurnRpmLimit, yawSign);
        routeTurnRpm = routeTurnRamp(routeTurnRpm,
                                    routeTurnInBand ? 0.0f : pidTurn, dt);
        sendRouteSpeeds(0, 0, routeRound(routeTurnRpm));
        if (routeTurnInBand && routeRound(routeTurnRpm) == 0) {
            if (routeTurnInBand == 1U) { routeTurnStable = now; routeTurnInBand = 2U; }
            if (now - routeTurnStable >= 200U &&
                (routeStationAlignment != 1U ||
                 (int32_t)(sampleMs - routeTurnStable) >= 200)) {
                routeRotating = routeTurnInBand = 0U;
                routeHeading = routeNextHeading;
                routeLegStart = now;
                routeHeadingPidReset(&routeHeadingPid);
                if (routeStationAlignment == 1U) {
                    routeStationAlignment = 2U;
                    serialSendString("ROUTE station heading aligned (1deg / 200ms)\r\n");
                }
                if (routeOnlyTurn) {
                    stopDriveMotors();
                    routeActive = routeOnlyTurn = 0U;
                    routeSentValid = 0U;
                    serialSendString("TURN DONE (2deg entry / 3.5deg hysteresis)\r\n");
                }
            }
        }
        return;
    }
    /* Task arrival depends only on this leg's commanded travel estimate. */
    if (navRunning) {
        routeX += routeEstimateScaled(routeRight, dt,
            (routeHeading == 0 || routeHeading == 2) ?
            routeLateralScale : routeForwardScale);
        routeY += routeEstimateScaled(routeForward, dt,
            (routeHeading == 1 || routeHeading == -1) ?
            routeLateralScale : routeForwardScale);
        if (now - navLastReport >= 200U) {
            navLastReport = now;
            printNavPose("RUN");
        }
        p = activeRoutePoint();
        dx = p.x - routeX; dy = p.y - routeY;
    } else {
        leg = activeDistanceLeg();
        routeLegProgress += routeAbs(routeEstimateScaled(
            leg.upMm ? routeForward : routeRight, dt,
            ((leg.upMm && (routeHeading == 1 || routeHeading == -1)) ||
             (leg.rightMm && (routeHeading == 0 || routeHeading == 2))) ?
            routeLateralScale : routeForwardScale));
        remaining = routeAbs(leg.upMm ? leg.upMm : leg.rightMm) - routeLegProgress;
        if (remaining < 0.0f) remaining = 0.0f;
        p.x = p.y = 0; p.event = leg.event; p.heading = leg.heading;
        dx = leg.rightMm ? (leg.rightMm > 0 ? remaining : -remaining) : 0.0f;
        dy = leg.upMm ? (leg.upMm > 0 ? remaining : -remaining) : 0.0f;
        if (fullRouteRunning && now - routeLastReport >= 200U) {
            routeLastReport = now;
            printFullRoutePose("RUN");
        }
        if (leg.departureHeading != 4 && routeHeading != leg.departureHeading &&
            remaining > 1.5f) {
            sendRouteSpeeds(0, 0, 0);
            if (motionInterrupted()) return;
            routeForward = routeRight = 0;
            routeDriveRpm = routeTurnRpm = routeCorrection = 0.0f;
            routeHeadingPidReset(&routeHeadingPid);
            routeNextHeading = leg.departureHeading;
            routeYaw = routeBaseYaw + leg.departureHeading * 90.0f * yawSign;
            routeRotating = 1U; routeTurnInBand = 0U;
            routeTurnPidReset(&routeTurnPid);
            routeTurnStart = routeLegStart = clockMs;
            serialSendString("ROUTE turning before distance leg\r\n");
            return;
        }
    }
    if (!routeAuto && now - routeSettle < 300U) return;
    /* At the endpoint don't reverse just because one sampled step crossed it. */
    if (navRunning && ((routeRight > 0 && dx < 0) || (routeRight < 0 && dx > 0))) {
        routeX = p.x; dx = 0;
    }
    if (navRunning && ((routeForward > 0 && dy < 0) || (routeForward < 0 && dy > 0))) {
        routeY = p.y; dy = 0;
    }
    if (routeAbs(dx) <= 1.5f && routeAbs(dy) <= 1.5f) {
        if ((arcRoutePhase && routeIndex == 2U) || missionArcCorner(routeIndex)) {
            beginArcBlend();
            return;
        }
        sendRouteSpeeds(0, 0, 0);
        if (motionInterrupted()) return;
        routeForward = routeRight = 0;
        routeDriveRpm = routeTurnRpm = routeCorrection = 0.0f;
        routeHeadingPidReset(&routeHeadingPid);
        if (navRunning) { routeX = p.x; routeY = p.y; }
        if ((p.heading != 4 && p.heading != routeHeading) ||
            (p.event && routeStationAlignment == 0U)) {
            routeStationAlignment = p.event ? 1U : 0U;
            routeNextHeading = p.heading == 4 ? routeHeading : p.heading;
            routeYaw = routeBaseYaw + routeNextHeading * 90.0f * yawSign;
            routeRotating = 1U; routeTurnInBand = 0U;
            routeTurnPidReset(&routeTurnPid);
            routeTurnStart = routeLegStart = clockMs;
            serialSendString("ROUTE turning to station heading\r\n");
            return;
        }
        if (navRunning) {
            navCurrentNode = navNodeAt(navPath[routeIndex].x,
                                       navPath[routeIndex].y);
            if (routeIndex + 1U == navPathCount) {
                finishNavigation();
                return;
            }
            ++routeIndex;
            routeStationAlignment = 0U;
            routeSettle = routeLegStart = clockMs;
            return;
        }
        serialSendString("ROUTE distance leg complete: ");
        serialSendUint((uint16_t)(routeIndex + 1U));
        if (p.event) { serialSendChar(' '); serialSendString(p.event); }
        serialSendString("\r\n");
        if (fullRouteRunning && p.event) printFullRouteStage(&p);
        missionArcExit = 0U;
        if (presetRouteActive) {
            uint8_t next = presetAdvance(&presetRoute, &routeIndex);
            routeStationAlignment = 0U;
            routeLegProgress = 0.0f;
            routeSettle = routeLegStart = clockMs;
            if (!next) { finishFullRoute(); return; }
            routeWaiting = (uint8_t)(next == 2U && missionActive);
            if (routeWaiting) missionBeginStation(routeIndex);
            else routeIndex = presetRoute.legs[presetRoute.index].taskIndex;
            return;
        }
        if (arcRoutePhase && routeIndex == 3U) {
            stopDriveMotors();
            routeActive = routeWaiting = fullRouteRunning = arcRoutePhase = 0U;
            routeSentValid = 0U;
            serialSendString("ARCTEST DONE station=RAW_1 no_pickup\r\n");
            serialSendString("ROUTE DONE mode=DISTANCE\r\n");
            return;
        }
        if (routeIndex + 1U == ROUTE_COUNT) {
            if (fullRouteRunning) finishFullRoute();
            else {
                stopAllMotors();
                serialSendString("ROUTE END: verify actual home position\r\n");
            }
            return;
        }
        ++routeIndex;
        routeStationAlignment = 0U;
        routeLegProgress = 0.0f;
        routeWaiting = (uint8_t)((!routeAuto && (routeStep || p.event != 0)) ||
                                 (rawPickRouteActive && routeIndex == 4U) ||
                                 (missionActive && p.event != 0));
        routeSettle = routeLegStart = clockMs;
        if (arcRoutePhase && routeIndex == 2U) {
            routeWaiting = 1U;
            arcRoutePhase = 2U;
            arcRouteWaitStart = clockMs;
            missionQrValid = qrResultAtStation = 0U;
            qrRxReady = 0U;
            serialSendString("ARCTEST WAIT_QR timeout_ms=45000\r\n");
        }
        if (missionActive) missionBeginStation(routeIndex);
        if (rawPickRouteActive && routeIndex == 4U) {
            rawPickAtStation = 1U;
            rawPickItemIndex = 0U;
            rawPickMissingReported = 0U;
            rawPickCurrentColor = rawPickCurrentSlot = rawPickCollectedMask = 0U;
            printRawPickState("WAIT_MATERIAL", rawPickColors[0][0], 1U);
        }
        if (routeWaiting && !missionActive)
            serialSendString("WAIT: verify position / finish station work; send route next\r\n");
        return;
    }
    /* Map-axis translation is transformed to the current body heading. */
    lateral = routeAbs(dx) > 1.5f;
    remaining = lateral ? routeAbs(dx) : routeAbs(dy);
    bodyLateral = (routeHeading == 1 || routeHeading == -1) ? !lateral : lateral;
    cruiseRpm = bodyLateral && routeRpm > routeLateralRpmLimit ?
        routeLateralRpmLimit : routeRpm;
    /* Leave either start zone slowly before resuming the requested route speed. */
    if (!navRunning && routeIndex == 0U && cruiseRpm > ROUTE_START_EXIT_RPM)
        cruiseRpm = ROUTE_START_EXIT_RPM;
    if (arcRoutePhase || missionArcCorner(routeIndex) || missionArcExit) {
        uint8_t id;
        uint16_t trim = 1000U;
        for (id = 1U; id <= 4U; ++id) if (motorTrim[id] > trim) trim = motorTrim[id];
        if (cruiseRpm > ARC_BLEND_WHEEL_RPM * 1000U / trim - ROUTE_HEADING_MAX_RPM)
            cruiseRpm = (uint16_t)(ARC_BLEND_WHEEL_RPM * 1000U / trim - ROUTE_HEADING_MAX_RPM);
    }
    if (!navRunning) serviceMissionObservation(&leg, remaining, cruiseRpm, bodyLateral);
    if (!routeActive || motionInterrupted()) return;
    brakeRemaining = remaining;
    if (arcRoutePhase == 4U || missionArcExit ||
        (arcRoutePhase == 1U && routeIndex == 2U) || missionArcCorner(routeIndex)) {
        /* Reserve one control interval for braking before arc entry or a station. */
        brakeRemaining -= routeAbs(routeEstimateScaled(routeRound(routeDriveRpm), dt, routeForwardScale));
        if (brakeRemaining < 0.0f) brakeRemaining = 0.0f;
    }
    speed = routeSpeed(remaining, now - routeLegStart, cruiseRpm);
    if ((arcRoutePhase == 1U && routeIndex == 2U) || missionArcCorner(routeIndex)) {
        float t = now - routeLegStart < 700U ? (now - routeLegStart) / 700.0f : 1.0f;
        float edgeRpm = cruiseRpm < ARC_BLEND_EDGE_RPM ? cruiseRpm : ARC_BLEND_EDGE_RPM;
        float cap = sqrtf(edgeRpm * edgeRpm +
            2.0f * ROUTE_BRAKE_RPM_S * brakeRemaining /
            (ROUTE_MM_PER_REV / 60.0f * routeForwardScale));
        float desired = cruiseRpm * t * t * (3.0f - 2.0f * t);
        speed = routeRound(desired < cap ? desired : cap);
    }
    correctionAccel = arcRoutePhase == 4U || missionArcExit ? 50.0f : 90.0f;
    if ((arcRoutePhase == 4U || missionArcExit) && !missionArcCorner(routeIndex)) {
        float cap = sqrtf(2.0f * ROUTE_BRAKE_RPM_S * brakeRemaining /
                         (ROUTE_MM_PER_REV / 60.0f * routeForwardScale));
        if (speed > cap) speed = routeRound(cap);
    }
    routeDriveRpm = routeDriveSlew(routeDriveRpm, speed, dt);
    speed = routeRound(routeDriveRpm);
    routeRight = lateral ? (dx > 0 ? speed : -speed) : 0;
    routeForward = lateral ? 0 : (dy > 0 ? speed : -speed);
    pidTurn = routeHeadingPidStep(&routeHeadingPid, &routeHeadingGains, error,
                                  yaw, sampleMs, dt, yawSign);
    /* A slow arc exit still needs yaw damping. Scaling by the requested
     * 250 RPM cruise would almost disable it at the live 10..36 RPM speed. */
    if (arcRoutePhase == 4U || missionArcExit) {
        float limit = routeDriveRpm > 1.0f ? routeDriveRpm - 1.0f : 0.0f;
        if (pidTurn > limit) pidTurn = limit;
        if (pidTurn < -limit) pidTurn = -limit;
    } else pidTurn *= (float)speed / cruiseRpm;
    routeCorrection = routeSlew(routeCorrection, pidTurn, correctionAccel, dt);
    turn = routeRound(routeCorrection);
    routeBody(routeForward, routeRight, routeHeading, &bodyForward, &bodyRight);
    sendRouteSpeeds(bodyForward, bodyRight, turn);
    if (!motionInterrupted() && now - headingPidLastReport >= 200U) {
        headingPidLastReport = now;
        printHeadingPidTrace(routeYaw, yaw, turn);
    }
}

static uint8_t processRouteCommand(const char *command)
{
    uint8_t start, missionRun;
    uint16_t degrees, rpm;
    const char *cursor;
    if (strncmp(command,"route auto ",11)==0) {
        cursor=command+11;missionRun=0;
    } else if(strncmp(command,"route mission ",14)==0) {
        cursor=command+14;missionRun=1;
    } else return 0;
    if (!parseValueWithOptionalRpm(cursor, &degrees, &rpm) ||
        degrees < 1U || degrees > 2U || rpm < 10U || rpm > 250U) {
        serialSendString("ERR: route auto|mission 1|2 [10..250rpm]\r\n");
        return 1U;
    }
    if (motionMode || routeActive || mechanismState.running ||
        mechanismActionActive || visionSessionActive()) {
        serialSendString(missionRun ? "ERR ROUTE: mission busy; stop first\r\n" :
                                      "ERR: busy; stop first\r\n");
        return 1U;
    }
    if (!armed) {
        serialSendString(missionRun ? "ERR ROUTE: mission needs arm\r\n" :
                                      "ERR: send 'arm' first\r\n");
        return 1U;
    }
    if (missionRun && !mechanismState.valid) {
        serialSendString("ERR ROUTE: rawpick mechanism not initialized; check mech status\r\n");
        return 1U;
    }
    if (missionRun && (mechanismState.current.horizontalDmm != 0 ||
                   mechanismState.current.liftDmm != 0U ||
                   (mechanismState.current.turretDdeg != 530U &&
                    mechanismState.current.turretDdeg != 2600U))) {
        serialSendString("ERR ROUTE: rawpick needs pose h=0 l=0 t=530 or 2600; check mech status\r\n");
        return 1U;
    }
    if (missionRun && (servoChannelMoving[0] || servoChannelMoving[1] ||
                   servoChannelMoving[2])) {
        serialSendString("ERR ROUTE: rawpick servo still moving; wait for SERVO DONE\r\n");
        return 1U;
    }
    if (missionRun && !vision_calibration_valid(&visionMaterialCalibration)) {
        serialSendString("ERR ROUTE: rawpick material calibration invalid; apply vision scale material\r\n");
        return 1U;
    }
    if (missionRun && !vision_calibration_valid(&visionRingCalibration[1])) {
        serialSendString("ERR ROUTE: mission ring 2 calibration invalid; apply vision scale ring 2\r\n");
        return 1U;
    }
    if (missionRun && (!servoChannelEnabled[0] || !servoChannelEnabled[1] ||
                       (currentAngleMdeg[0] != (uint32_t)rawPickInitial.gripperOpenDeg * 1000U &&
                        currentAngleMdeg[0] != (uint32_t)rawPickInitial.gripperCloseDeg * 1000U) ||
                       currentAngleMdeg[1] != 26000U)) {
        serialSendString("ERR ROUTE: mission needs configured open/close gripper angle and platform=26\r\n");
        return 1U;
    }
    armed = 0U;
    if (!imuValid || clockMs - imuStamp > 250U || motionInterrupted()) {
        serialSendString(missionRun ?
            "ERR ROUTE: mission needs fresh IMU and healthy CAN\r\n" :
            "ERR: fresh IMU and healthy CAN required\r\n");
        return 1U;
    }
    if (!presetBuild(&presetRoute, (uint8_t)degrees, routeObstacleRegions)) {
        serialSendString("ERR ROUTE: NO_PATH; select two regions with clear station approaches\r\n");
        return 1U;
    }
    if (ARC_BLEND_GEOMETRY_MM * routeForwardScale / ARC_BLEND_RADIUS_MM >= 0.98f) {
        unsigned i;
        for (i = 0U; i < presetRoute.count; ++i) {
            presetRoute.legs[i].arc = 0U;
            presetRoute.legs[i].drive.heading = presetRoute.legs[i].to.heading;
        }
    }
    missionButtonReady = 0U;
    invalidateNavigation("manual_motion");
    start = (uint8_t)degrees;
    routeRpm = rpm;
    routeAuto = fullRouteRunning = 1U;
    presetRouteActive = 1U;
    missionActive = missionRun;
    if (missionRun) {
        missionTaskValid = 0U;
        tjcDisplayResetCounts();
    }
    missionPhase = MISSION_IDLE;
    missionActionStarted = 0U;
    missionOffsetForward = missionOffsetRight = 0.0f;
    rawPickRouteActive = 0U;
    rawPickAtStation = rawPickItemIndex = rawPickMissingReported = 0U;
    rawPickCurrentColor = rawPickCurrentSlot = rawPickCollectedMask = 0U;
    routeHeading = routeNextHeading = 0;
    routeDriveRpm = routeTurnRpm = routeCorrection = 0.0f;
    routeHeadingPidReset(&routeHeadingPid);
    routeSentValid = 0U;
    routeRotating = routeOnlyTurn = routeTurnInBand = 0U;
    routeStationAlignment = 0U;
    routeStartZone = start; routeStep = 0U; routeIndex = 0U;
    routeRawLeftCorrection = 0;
    routeRawDownCorrection = 0;
    routeLegProgress = 0.0f;
    routeBaseYaw = routeYaw = imuYaw;
    routeForward = routeRight = 0;
    setAllMotorsEnabled(true);
    if (motionInterrupted()) {
        uint8_t wasFullRoute = fullRouteRunning;
        stopAllMotors();
        if (wasFullRoute) serialSendString("ROUTE INVALID reason=can_fault\r\n");
        return 1U;
    }
    routeTick = routeSettle = routeLegStart = clockMs;
    routeLastReport = clockMs - 200U;
    routeWaiting = 0U; routeActive = 1U;
    if (missionRun) vision_session_init(&visionSession);
    serialSendString("ROUTE START mode=PRESET_REGIONS; nose UP; nominal pose only\r\n");
    serialSendString("ROUTE PLAN regions_mask="); serialSendUint(routeObstacleRegions);
    serialSendString(" legs="); serialSendUint(presetRoute.count); serialSendString("\r\n");
    if (fullRouteRunning) printFullRoutePose("RUN");
    return 1U;
}

static void serviceArcRoute(void)
{
    if (!arcRoutePhase && !missionArcActive) return;
    if (motionInterrupted() || !imuValid || clockMs - imuStamp > 250U ||
        (arcRoutePhase && (!hostHeartbeatActive ||
         clockMs - hostHeartbeatStamp > HOST_HEARTBEAT_TIMEOUT_MS))) {
        stopAllMotors();
        serialSendString("ERR ROUTE ARC: IMU/heartbeat/CAN fault; stopped\r\n");
        return;
    }
    if (arcRoutePhase == 2U) {
        if (clockMs - arcRouteWaitStart >= 45000U) {
            stopAllMotors();
            serialSendString("ERR ARCTEST: QR_TIMEOUT; stopped\r\n");
        } else if (qrResultAtStation && missionQrValid && clockMs - arcRouteWaitStart >= 1000U) {
            routeWaiting = 0U;
            arcRoutePhase = 1U;
            routeTick = routeSettle = routeLegStart = clockMs;
            serialSendString("ARCTEST QR_OK continuing\r\n");
        }
    } else if (arcRoutePhase == 3U || missionArcActive) {
        uint32_t now = clockMs, dt = now - arcBlend.tick, stamp;
        float yaw, heading, curvature, error, exitError, forward, turn, deltaForward, deltaTurn, step, sum;
        float deltaYaw, turnLimit;
        uint8_t id;
        uint16_t trim = 1000U;
        __disable_irq();
        yaw = imuYaw; stamp = imuStamp;
        __enable_irq();
        if (dt > 150U || now - arcBlend.start >= 45000U || motionMode != 7U) {
            stopAllMotors();
            serialSendString("ERR ROUTE ARC: control delay/arc timeout; stopped\r\n");
            return;
        }
        if (dt < 20U) return;
        arcBlend.tick = now;
        if (stamp != arcBlend.sample) {
            deltaYaw = headingError(yaw, arcBlend.previousYaw) * yawSign * arcBlend.direction;
            arcBlend.rateDps = deltaYaw * 1000.0f / (stamp - arcBlend.sample);
            arcBlend.turned += deltaYaw;
            arcBlend.previousYaw = yaw; arcBlend.sample = stamp;
        }
        /* Exit protection uses the same route heading as exit completion.
         * Relative rotation also contains any heading error at arc entry. */
        exitError = headingError(arcBlend.startYaw + 90.0f * yawSign * arcBlend.direction, yaw) *
                    yawSign * arcBlend.direction;
        if (arcBlend.turned < -3.5f || exitError < -3.5f) {
            stopAllMotors();
            serialSendString("ERR ARC: reason=");
            printArcBlendState(arcBlend.turned < 0.0f ? "WRONG_DIRECTION" : "OVERSHOOT");
            return;
        }
        arcBlend.distance += routeAbs(routeEstimateScaled(routeRound(arcBlend.forward), dt,
                                                          routeForwardScale));
        arcBlendShape(arcBlend.distance, &heading, &curvature);
        error = headingError(arcBlend.startYaw + heading * yawSign * arcBlend.direction, yaw);
        if (routeAbs(error) > 20.0f) {
            stopAllMotors();
            serialSendString("ERR ROUTE ARC: curve heading deviation; stopped\r\n");
            return;
        }
        if (arcBlend.distance > ARC_BLEND_LENGTH_MM + 30.0f &&
            routeAbs(exitError) > 3.5f) {
            stopAllMotors();
            serialSendString("ERR ARC: reason="); printArcBlendState("EXIT_HEADING");
            return;
        }
        for (id = 1U; id <= 4U; ++id) if (motorTrim[id] > trim) trim = motorTrim[id];
        turnLimit = arcBlendTurnLimit(exitError, arcBlend.rateDps, clockMs - stamp, trim);
        /* A delayed IMU may still be converging at the geometric tangent.
         * Keep moving at the edge speed for at most 30mm, never pivot in place. */
        if (arcBlend.distance >= ARC_BLEND_LENGTH_MM &&
            routeAbs(exitError) <= 3.5f) {
            RouteLeg exit;
            /* Retain translation, but do not carry yaw that predicts crossing
             * the target into the straight-line controller. */
            if (arcBlend.turn * arcBlend.direction > turnLimit)
                arcBlend.turn = arcBlend.direction * turnLimit;
            sendRouteSpeeds(arcBlend.travel * routeRound(arcBlend.forward), 0,
                            routeRound(arcBlend.turn));
            if (motionInterrupted()) {
                stopAllMotors();
                serialSendString("ERR ROUTE ARC: CAN fault at exit; stopped\r\n");
                return;
            }
            printArcBlendState("ARC DONE");
            motionMode = 0U;
            routeHeading = routeNextHeading = arcBlend.exitHeading;
            routeYaw = routeBaseYaw + routeHeading * 90.0f * yawSign;
            if (presetRouteActive) {
                ++presetRoute.index;
                routeIndex = presetRoute.legs[presetRoute.index].taskIndex;
            } else ++routeIndex;
            routeLegProgress = arcBlend.distance - ARC_BLEND_LENGTH_MM;
            routeDriveRpm = arcBlend.forward;
            routeCorrection = arcBlend.turn;
            routeTurnRpm = 0.0f;
            exit = presetRouteActive ? presetActiveLeg(&presetRoute) :
                   routeObstacleLeg(routeIndex, routeStartZone, routeObstaclePosition);
            routeForward = exit.upMm ? (exit.upMm > 0 ? 1 : -1) * routeRound(arcBlend.forward) : 0;
            routeRight = exit.rightMm ? (exit.rightMm > 0 ? 1 : -1) * routeRound(arcBlend.forward) : 0;
            routeHeadingPidReset(&routeHeadingPid);
            routeHeadingPid.initialized = 1U;
            routeHeadingPid.previousYaw = yaw;
            routeHeadingPid.previousSampleMs = stamp;
            routeHeadingPid.filteredRate = arcBlend.rateDps * yawSign * arcBlend.direction;
            routeStationAlignment = routeWaiting = 0U;
            routeTick = routeSettle = now;
            /* Carry the live wheel targets into the exit, without smooth-start reset. */
            routeLegStart = now - 700U;
            routeActive = 1U;
            if (missionArcActive) {
                missionArcActive = 0U;
                missionArcExit = 1U;
                serialSendString("ROUTE ARC EXIT continuous=1\r\n");
            } else {
                arcRoutePhase = 4U;
                serialSendString("ARCTEST EXIT distance_mm=641 continuous=1\r\n");
            }
            return;
        }
        forward = arcBlendSpeed(arcBlend.distance, routeForwardScale,
                                (float)ARC_BLEND_WHEEL_RPM * 1000U / trim);
        /* A lagging heading needs time to converge before the 30mm exit
         * allowance runs out. Slow translation in the final curvature ramp
         * instead of advancing at a fixed edge speed while removing yaw. */
        if (arcBlend.distance >= ARC_BLEND_LENGTH_MM - ARC_BLEND_RAMP_MM &&
            exitError > 90.0f - heading + 3.5f) {
            forward *= (90.0f - heading + 3.5f) / exitError;
            if (arcBlend.distance > ARC_BLEND_LENGTH_MM)
                forward *= (ARC_BLEND_LENGTH_MM + 30.0f - arcBlend.distance) / 30.0f;
            if (forward < 10.0f) forward = 10.0f;
        }
        if (missionArcActive && forward > routeRpm) forward = routeRpm;
        /* The reference turns too: differentiate the tracking error, otherwise
         * the straight-line PID's yaw damping fights the planned rotation. */
        turn = arcBlend.direction * forward * ARC_BLEND_GEOMETRY_MM * routeForwardScale * curvature +
            routeHeadingPidStep(&routeHeadingPid, &routeHeadingGains, error,
                                -error, stamp, dt, yawSign);
        if (turn * arcBlend.direction > turnLimit) turn = arcBlend.direction * turnLimit;
        if (turn > forward - 1.0f) turn = forward - 1.0f;
        if (turn < 1.0f - forward) turn = 1.0f - forward;
        deltaForward = forward - arcBlend.forward;
        deltaTurn = turn - arcBlend.turn;
        /* Retain the original braking budget whenever translation or yaw slows,
         * including a yaw reversal. Only acceleration uses the higher rate. */
        step = (deltaForward < 0.0f || routeAbs(turn) < routeAbs(arcBlend.turn) ||
                turn * arcBlend.turn < 0.0f ? ARC_BLEND_BRAKE_RPM_S : ARC_BLEND_ACCEL_RPM_S) * dt / trim;
        sum = routeAbs(deltaForward) + routeAbs(deltaTurn);
        if (sum > step) { deltaForward *= step / sum; deltaTurn *= step / sum; }
        arcBlend.forward += deltaForward; arcBlend.turn += deltaTurn;
        /* Once at/past the target, safety takes precedence over yaw slew.
         * Forward motion continues and the opposite correction is retained. */
        if (exitError <= 0.0f && arcBlend.turn * arcBlend.direction > 0.0f)
            arcBlend.turn = 0.0f;
        sendRouteSpeeds(arcBlend.travel * routeRound(arcBlend.forward), 0, routeRound(arcBlend.turn));
        if (motionInterrupted()) {
            stopAllMotors();
            serialSendString("ERR ROUTE ARC: CAN fault; stopped\r\n");
            return;
        }
        if (now - arcBlend.report >= 200U) {
            arcBlend.report = now; printArcBlendState("ARC POS");
        }
    }
}

static void serviceMaterialLayout(void)
{
    VisionPacket packet;
    VisionEvent event;
    VisionState before;
    uint8_t wasPositioned;
    uint8_t index;
    if (!mechanismState.valid) { failVision("PICK_PREP_FAILED"); return; }
    if (!motionMode && !mechanismState.running &&
        !servoChannelMoving[0] && !servoChannelMoving[1] && !servoChannelMoving[2] &&
        visionSession.layoutPhase == VISION_LAYOUT_TRACK &&
        (visionSession.state == VISION_STATE_WAIT || visionSession.state == VISION_STATE_SETTLE)) {
        if (mechanismState.current.horizontalDmm != rawPickObservePose.horizontalDmm ||
            mechanismState.current.liftDmm != rawPickObservePose.liftDmm ||
            mechanismState.current.turretDdeg != rawPickObservePose.turretDdeg ||
            currentAngleMdeg[0] != (uint32_t)rawPickInitial.gripperOpenDeg * 1000U) {
            failVision("PICK_PREP_FAILED"); return;
        }
        if (visionSession.state != VISION_STATE_SETTLE)
            vision_session_move_complete(&visionSession, clockMs);
        visionSession.layoutPhase = VISION_LAYOUT_CONFIRM;
        vision_session_make_request(&visionSession, &visionMaterialCalibration, &packet, clockMs);
        visionUartSend(&packet);
        printVisionState();
    }
    event = vision_session_tick(&visionSession, clockMs);
    if (event == VISION_EVENT_FAILED) { failVision(visionSession.fault); return; }
    if (visionSession.state == VISION_STATE_REQUEST) {
        vision_session_make_request(&visionSession, &visionMaterialCalibration, &packet, clockMs);
        visionUartSend(&packet);
        printVisionState();
    }
    if (!visionRxReady) return;
    __disable_irq();
    packet = visionRxPacket;
    visionRxReady = 0U;
    __enable_irq();
    before = visionSession.state;
    /* Results still refresh communication during motion, but only the parked
     * phase may produce correction/ready events. */
    if (motionMode || mechanismState.running || servoChannelMoving[0] ||
        servoChannelMoving[1] || servoChannelMoving[2]) {
        if (packet.type == VISION_MESSAGE_RESULT && packet.token == visionSession.token &&
            packet.mode == VISION_MODE_MATERIAL && packet.selector == 0U && packet.target == 0U &&
            (packet.flags & VISION_FLAG_MATERIAL_LAYOUT)) {
            visionSession.requestAt = clockMs;
            visionSession.misses = 0U;
        }
        return;
    }
    wasPositioned = visionSession.layoutPositioned;
    event = vision_layout_observe(&visionSession, &visionMaterialCalibration, &packet, clockMs);
    if (!wasPositioned && visionSession.layoutPositioned) {
        vision_session_make_request(&visionSession, &visionMaterialCalibration, &packet, clockMs);
        visionUartSend(&packet); /* Fresh token and fresh window for three-slot colors. */
        printRawPickState("WAIT_MATERIAL", rawPickCurrentColor, rawPickCurrentSlot);
    }
    if (event == VISION_EVENT_FAILED) { failVision(visionSession.fault); return; }
    if (event == VISION_EVENT_MOVE) {
        if (!startVisionMotion(visionSession.forwardMm, visionSession.rightMm, 30U, 1U)) {
            failVision("MOTION_START_FAILED"); return;
        }
        vision_session_make_request(&visionSession, &visionMaterialCalibration, &packet, clockMs);
        visionUartSend(&packet);
        printVisionState();
    } else if (event == VISION_EVENT_ALIGNED) {
        const uint8_t *colors = missionActive ? missionTask.colors[routeIndex == 11U] :
                                              rawPickColors[routeIndex == 11U];
        static const char *positions[] = {"NONE", "BOTTOM", "LEFT_UP", "RIGHT_UP"};
        stopDriveMotors();
        printVisionState();
        for (index = 0U; index < 3U; ++index) {
            if (rawPickCollectedMask & (1U << (colors[index] - 1U))) continue;
            serialSendString("ROUTE RAWPICK TARGET color="); serialSendUint(colors[index]);
            serialSendString(" position=");
            serialSendString(positions[vision_layout_pick_position(&visionSession, colors[index])]);
            serialSendString(" slot="); serialSendUint((uint16_t)(index + 1U));
            serialSendString(" inferred="); serialSendUint(visionSession.layoutInferred);
            serialSendString("\r\n");
        }
        if (!rawPickActionsForPosition(rawPickRouteActions, rawPickCurrentSlot,
                vision_layout_pick_position(&visionSession, rawPickCurrentColor))) {
            printRawPickState("WAIT_ACTIONS", rawPickCurrentColor, rawPickCurrentSlot);
            return;
        }
        if (!mechanismActionStart(&rawPickInitial, rawPickRouteActions, RAW_PICK_ACTION_COUNT)) {
            failVision("PICK_START_FAILED"); return;
        }
        visionSession.layoutPhase = VISION_LAYOUT_END;
        vision_session_make_request(&visionSession, &visionMaterialCalibration, &packet, clockMs);
        visionUartSend(&packet);
        visionSession.state = VISION_STATE_PICK;
        printVisionState();
        printRawPickState("PICK", rawPickCurrentColor, rawPickCurrentSlot);
    } else if (before == VISION_STATE_WAIT_ACTIONS && visionSession.state != before) {
        printVisionState();
        printRawPickState("WAIT_MATERIAL", rawPickCurrentColor, rawPickCurrentSlot);
    }
}

static void serviceRawPickRoute(void)
{
    int8_t prepared;
    if (!rawPickAtStation || !routeWaiting) return;
    if (!imuValid || clockMs - imuStamp > 250U || motionInterrupted()) {
        visionPickActive = 1U;
        failVision("IMU_OR_CAN_FAULT");
        return;
    }
    if (rawPickItemIndex >= 3U) return;
    if (visionPickActive || visionSessionActive()) return;
    if (visionSession.state != VISION_STATE_IDLE &&
        visionSession.state != VISION_STATE_PICK_DONE) return;
    prepared = finishMissionObservation(routeIndex);
    if (prepared < 0 || prepared == 1) return;
    if (!mechanismState.valid || mechanismState.running ||
        (mechanismState.current.horizontalDmm != 0 && prepared != 2) ||
        mechanismState.current.liftDmm != 0U ||
        (mechanismState.current.turretDdeg != 2600U &&
         !(rawPickItemIndex == 0U &&
           mechanismState.current.turretDdeg == 530U))) {
        visionPickActive = 1U;
        failVision("PICK_REQUIRES_INITIAL_POSE");
        return;
    }
    rawPickCurrentColor = missionActive ? missionTask.colors[routeIndex == 11U][rawPickItemIndex] :
                                         rawPickColors[routeIndex == 11U][rawPickItemIndex];
    rawPickCurrentSlot = (uint8_t)(rawPickItemIndex + 1U);
    if (!vision_session_start(&visionSession, VISION_MODE_MATERIAL,
                              rawPickItemIndex ? rawPickCurrentColor : 0U,
                              0U, &visionMaterialCalibration, clockMs)) {
        visionPickActive = 1U;
        failVision("PICK_START_REJECTED");
        return;
    }
    visionPickActive = 1U;
    visionSession.requestFlags = VISION_FLAG_MATERIAL_LAYOUT;
    visionSession.materialMask = rawPickPendingMask();
    if (!visionSession.materialMask) { failVision("PICK_REQUIRES_THREE_COLORS"); return; }
    visionSession.state = VISION_STATE_PREP;
    printRawPickState("PREP", rawPickCurrentColor, rawPickCurrentSlot);
    servoSetTarget(2U, rawPickInitial.gripperOpenDeg, rawPickInitial.gripperDps10);
    if (prepared != 2) {
        startMechanismPose(&rawPickObservePose);
        if (!mechanismState.running) failVision("PICK_PREP_FAILED");
    }
}

static void serviceVision(void)
{
    const VisionCalibration *calibration;
    VisionPacket packet;
    VisionEvent event;
    int16_t du;
    int16_t dv;
    flushVisionTrackingEnd();
    if (visionTrackingActive()) {
        receiveVisionTracking();
        return;
    }
    if (!visionSessionActive()) {
        if ((visionSession.requestFlags & VISION_FLAG_MATERIAL_LAYOUT) &&
            visionSession.layoutPhase != VISION_LAYOUT_END) {
            VisionState stopped = visionSession.state;
            visionSession.layoutPhase = VISION_LAYOUT_END;
            vision_session_make_request(&visionSession, &visionMaterialCalibration, &packet, clockMs);
            visionSession.state = stopped;
            visionUartSend(&packet);
        }
        visionRxReady = 0U;
        return;
    }
    if (!imuValid || clockMs - imuStamp > 250U || motionInterrupted()) {
        failVision("IMU_OR_CAN_FAULT");
        return;
    }
    if (visionSession.state == VISION_STATE_PREP) {
        uint8_t color = visionSession.selector;
        if (!mechanismState.valid) {
            failVision("PICK_PREP_FAILED");
        } else if (!mechanismState.running && !servoChannelMoving[0]) {
            if (mechanismState.current.horizontalDmm != rawPickObservePose.horizontalDmm ||
                mechanismState.current.liftDmm != 0U ||
                mechanismState.current.turretDdeg != 2600U ||
                !vision_session_start(&visionSession, VISION_MODE_MATERIAL,
                                      color, 0U, visionCalibration(),
                                      clockMs)) {
                failVision("PICK_PREP_FAILED");
            } else {
                if (rawPickAtStation) {
                    visionSession.requestFlags = VISION_FLAG_MATERIAL_LAYOUT;
                    visionSession.materialMask = rawPickPendingMask();
                    visionSession.waitForTarget = 1U;
                    visionSession.layoutPositioned = rawPickItemIndex != 0U;
                }
                printVisionState();
                if (rawPickAtStation) {
                    printRawPickState("DETECT", color,
                                      color == 0U ? 0U : rawPickCurrentSlot);
                }
            }
        }
        return;
    }
    if (visionSession.state == VISION_STATE_PICK) {
        if (!mechanismState.valid) {
            failVision("PICK_ACTION_FAILED");
        } else if (!mechanismActionActive) {
            if (rawPickAtStation && rawPickItemIndex == 2U) {
                if (mechanismState.running) return;
                if (mechanismState.current.turretDdeg != 530U) {
                    startMechanismPose(&rawPickTravelPose);
                    if (!mechanismState.running) failVision("PICK_RETURN_FAILED");
                    return;
                }
            }
            visionPickActive = 0U;
            visionSession.state = VISION_STATE_PICK_DONE;
            printVisionState();
            serialSendString("VISION PICK DONE color=");
            serialSendUint(rawPickAtStation ? rawPickCurrentColor : visionSession.selector);
            serialSendString("\r\n");
            if (rawPickAtStation) {
                serialSendString("ROUTE RAWPICK state=ITEM_DONE color=");
                serialSendUint(rawPickCurrentColor);
                serialSendString(" slot=");
                serialSendUint(rawPickCurrentSlot);
                serialSendString("\r\n");
                ++rawPickItemIndex;
                rawPickCollectedMask |= (uint8_t)(1U << (rawPickCurrentColor - 1U));
                rawPickMissingReported = 0U;
                if (rawPickItemIndex == 3U) {
                    printRawPickState("DONE", rawPickCurrentColor, rawPickCurrentSlot);
                    if (!missionActive)
                        serialSendString("WAIT: verify three materials; send route next\r\n");
                } else {
                    printRawPickState("WAIT_MATERIAL",
                        missionActive ? missionTask.colors[routeIndex == 11U][rawPickItemIndex] :
                                        rawPickColors[routeIndex == 11U][rawPickItemIndex],
                        (uint8_t)(rawPickItemIndex + 1U));
                }
            }
        }
        return;
    }
    if (visionSession.requestFlags & VISION_FLAG_MATERIAL_LAYOUT) {
        serviceMaterialLayout();
        return;
    }
    calibration = visionCalibration();
    event = vision_session_tick(&visionSession, clockMs);
    if (event == VISION_EVENT_FAILED) {
        failVision(visionSession.fault);
        return;
    }
    if (event == VISION_EVENT_CORRECTION_LIMIT) {
        stopDriveMotors();
        serialSendString("VISION CORRECTION LIMIT iterations=");
        serialSendUint(visionSession.iteration);
        serialSendString("\r\n");
        printVisionState();
        return;
    }
    if (visionSession.state == VISION_STATE_REQUEST) {
        vision_session_make_request(&visionSession, calibration, &packet, clockMs);
        visionUartSend(&packet);
        printVisionState();
    }
    if (!visionRxReady) return;
    __disable_irq();
    packet = visionRxPacket;
    visionRxReady = 0U;
    __enable_irq();
    if (rawPickAtStation &&
        vision_session_wait_for_material(&visionSession, &packet)) {
        if (!rawPickMissingReported) {
            printRawPickState("WAIT_MATERIAL", visionSession.selector,
                              visionSession.selector == 0U ? 0U : rawPickCurrentSlot);
            rawPickMissingReported = 1U;
        }
        return;
    }
    event = vision_session_observe(&visionSession, calibration, &packet, clockMs);
    if (event == VISION_EVENT_NONE) return;
    du = (int16_t)(packet.value[0] - calibration->anchorU);
    dv = (int16_t)(packet.value[1] - calibration->anchorV);
    if (event == VISION_EVENT_SAMPLED) {
        serialSendString("VISION MEASURE id="); serialSendUint(visionSession.sampleId);
        if (visionSession.mode == VISION_MODE_MATERIAL) {
            serialSendString(" material="); serialSendUint(visionSession.selector);
        } else serialSendString(" ring=2");
        serialSendString(" token="); serialSendUint32(packet.token);
        serialSendString(" du="); serialSendInt(du);
        serialSendString(" dv="); serialSendInt(dv);
        serialSendString(" quality="); serialSendUint((uint16_t)packet.value[2]);
        serialSendString("\r\n");
        printVisionState();
        return;
    }
    serialSendString("VISION SAMPLE token=");
    serialSendUint32(packet.token);
    serialSendString(" du="); serialSendInt(du);
    serialSendString(" dv="); serialSendInt(dv);
    serialSendString(" forward_mm="); serialSendFloat1(visionSession.forwardMm);
    serialSendString(" right_mm="); serialSendFloat1(visionSession.rightMm);
    serialSendString(" quality="); serialSendUint((uint16_t)packet.value[2]);
    serialSendString(" iteration="); serialSendUint(visionSession.iteration);
    serialSendString("\r\n");
    if (event == VISION_EVENT_MOVE) {
        if (visionTrackingEligible()) {
            if (!startVisionTracking(visionSession.forwardMm,visionSession.rightMm))
                failVision("TRACKING_START_FAILED");
            else printVisionState();
            return;
        }
        if (!imuValid || clockMs - imuStamp > 250U ||
            !startVisionMotion(visionSession.forwardMm, visionSession.rightMm,
                               30U, 1U)) {
            failVision("MOTION_START_FAILED");
            return;
        }
        printVisionState();
    } else if (event == VISION_EVENT_ALIGNED) {
        if (visionPickActive) {
            const MechanismAction *actions = rawPickActions;
            if (mechanismState.running) {
                visionSession.state = VISION_STATE_REQUEST;
                visionSession.confirmations = 0U;
                return;
            }
            if (!mechanismState.valid ||
                mechanismState.current.horizontalDmm != rawPickObservePose.horizontalDmm ||
                mechanismState.current.liftDmm != rawPickObservePose.liftDmm ||
                mechanismState.current.turretDdeg != rawPickObservePose.turretDdeg) {
                failVision("PICK_PREP_FAILED");
                return;
            }
            if (!mechanismActionStart(&rawPickInitial, actions,
                                      RAW_PICK_ACTION_COUNT)) {
                failVision("PICK_START_FAILED");
            } else {
                visionSession.state = VISION_STATE_PICK;
                printVisionState();
                if (rawPickAtStation) {
                    serialSendString("ROUTE RAWPICK state=PICK color=");
                    serialSendUint(rawPickCurrentColor);
                    serialSendString(" slot=");
                    serialSendUint(rawPickCurrentSlot);
                    serialSendString("\r\n");
                }
            }
        } else {
            stopDriveMotors();
            serialSendString("VISION DONE forward_mm=");
            serialSendFloat1(visionSession.forwardMm);
            serialSendString(" right_mm=");
            serialSendFloat1(visionSession.rightMm);
            serialSendString(" iterations=");
            serialSendUint(visionSession.iteration);
            serialSendString("\r\n");
        }
    } else if (event == VISION_EVENT_FAILED) {
        failVision(visionSession.fault);
    }
}

static void missionReport(const char *phase)
{
    serialSendString("ROUTE MISSION phase=");
    serialSendString(phase);
    serialSendString(" station=");
    if (routeIndex == 2U) serialSendString("QR");
    else if (routeIndex == 4U) serialSendString("RAW_1");
    else if (routeIndex == 7U) serialSendString("COARSE_1");
    else if (routeIndex == 9U) serialSendString("TEMP_1");
    else if (routeIndex == 11U) serialSendString("RAW_2");
    else if (routeIndex == 14U) serialSendString("COARSE_2");
    else if (routeIndex == 16U) serialSendString("TEMP_2");
    else serialSendString("TRANSIT");
    serialSendString("\r\n");
}

static void missionAbort(const char *reason)
{
    if (!missionActive) return;
    stopServoMotion();
    stopAllMotors();
    serialSendString("ROUTE INVALID reason=mission_");
    serialSendString(reason);
    serialSendString("\r\n");
}

static void missionContinue(void)
{
    rawPickAtStation = 0U;
    visionPickActive = 0U;
    vision_session_init(&visionSession);
    missionPhase = MISSION_IDLE;
    missionActionStarted = 0U;
    missionOffsetForward = missionOffsetRight = 0.0f;
    routeWaiting = 0U;
    if (presetRouteActive) routeIndex = presetRoute.legs[presetRoute.index].taskIndex;
    routeTick = routeSettle = routeLegStart = clockMs;
    missionReport("CONTINUE");
}

static void missionBeginStation(uint8_t nextIndex)
{
    missionActionStarted = 0U;
    missionOffsetForward = missionOffsetRight = 0.0f;
    vision_session_init(&visionSession);
    if (nextIndex == 2U) {
        /* Preserve early results from the legacy single-shot QR entry as well. */
        missionTaskValid = 0U;
        missionPhase = MISSION_QR_WAIT;
        missionDeadline = clockMs + 1000U;
        missionReport("QR_WAIT");
    } else if (nextIndex == 4U || nextIndex == 11U) {
        routeRawLeftCorrection = 0;
        routeRawDownCorrection = 0;
        missionPhase = MISSION_RAW_PICK;
        rawPickAtStation = 1U;
        rawPickItemIndex = rawPickMissingReported = 0U;
        rawPickCurrentColor = rawPickCurrentSlot = rawPickCollectedMask = 0U;
        missionReport("RAW_PICK");
    } else if (nextIndex == 7U || nextIndex == 14U) {
        missionPhase = MISSION_RING_ALIGN;
        missionReport("RING_ALIGN");
    } else if (nextIndex == 9U || nextIndex == 16U) {
        uint8_t rings[3];
        if (!missionTaskStorageRings(&missionTask, nextIndex == 16U, rings)) {
            missionAbort("stack_colors");
            return;
        }
        missionPhase = MISSION_TEMP_ALIGN;
        missionBuildTaskPlace(missionWorkActions, rings,
                             nextIndex == 16U ? 2U : 1U, 0U);
        missionReport("TEMP_ALIGN");
    }
}

static void serviceMission(void)
{
    float forward, right;
    if (!missionActive || !routeActive || !routeWaiting) return;
    if (!imuValid || clockMs - imuStamp > 250U || motionInterrupted()) {
        missionAbort("imu_or_can");
        return;
    }
    if (missionPhase == MISSION_QR_WAIT) {
        if (!missionTaskValid && missionQrValid) {
            missionTaskValid = missionTaskParse(missionQrCode, &missionTask);
            missionQrValid = 0U;
            serialSendString("TASK SET code=");
            serialSendString(missionQrCode);
            serialSendString("\r\n");
        }
        if (missionTaskValid && (int32_t)(clockMs - missionDeadline) >= 0)
            missionContinue();
        return;
    }
    if (missionPhase == MISSION_RAW_PICK) {
        if (rawPickItemIndex == 3U && !visionPickActive &&
            !mechanismActionActive) {
            /* Preserve the alignment offset so pickup can return to its baseline. */
            missionPhase = MISSION_RECENTER;
            missionReport("RECENTER");
        }
        return;
    }
    if (missionPhase == MISSION_RING_ALIGN ||
        missionPhase == MISSION_TEMP_ALIGN) {
        if (!missionActionStarted) {
            int8_t prepared = finishMissionObservation(routeIndex);
            if (prepared < 0 || prepared == 1) return;
            if (missionPhase == MISSION_TEMP_ALIGN && mechanismState.running) return;
            if (!mechanismState.valid || mechanismState.running ||
                mechanismState.current.liftDmm != 0U ||
                (mechanismState.current.turretDdeg != 2600U &&
                 mechanismState.current.turretDdeg != 530U &&
                 !(missionPhase == MISSION_TEMP_ALIGN &&
                   mechanismState.current.turretDdeg == 1200U &&
                   mechanismState.current.horizontalDmm ==
                       (routeIndex == 16U ? -470 : -450)))) {
                missionAbort("ring_prep_pose");
                return;
            }
            if (prepared != 2) {
                startMechanismPose(&missionRingObservePose);
                if (!mechanismState.running) {
                    missionAbort("ring_prep_start");
                    return;
                }
            }
            missionActionStarted = 1U;
            missionReport(missionPhase == MISSION_TEMP_ALIGN && routeIndex == 16U ?
                          "MATERIAL_PREP" : "RING_PREP");
        } else if (missionActionStarted == 1U && !mechanismState.running) {
            uint8_t materialAlign = missionPhase == MISSION_TEMP_ALIGN && routeIndex == 16U;
            uint8_t item = 0U;
            /* Stack over the first layer's middle-ring material directly. */
            while (item < 2U && missionTask.rings[0][item] != 2U) ++item;
            if (!mechanismState.valid ||
                mechanismState.current.horizontalDmm != missionRingObservePose.horizontalDmm ||
                mechanismState.current.liftDmm != 0U ||
                mechanismState.current.turretDdeg != 2600U) {
                missionAbort("ring_prep_failed");
                return;
            }
            if (!vision_session_start(&visionSession,
                                      materialAlign ? VISION_MODE_MATERIAL : VISION_MODE_RING,
                                      materialAlign ? missionTask.colors[0][item] : 0U,
                                      2U, &visionRingCalibration[1], clockMs)) {
                missionAbort("ring_start");
                return;
            }
            visionSession.waitForTarget = 1U;
            visionSession.maxCorrections = 2U;
            if (materialAlign)
                visionSession.materialMask = (uint8_t)(1U << (visionSession.selector - 1U));
            missionActionStarted = 2U;
            printVisionState();
        } else if (missionActionStarted == 2U &&
                   (visionSession.state == VISION_STATE_ALIGNED ||
                    visionSession.state == VISION_STATE_CORRECTION_LIMIT)) {
            /* Continue the next distance leg from the locally aligned station. */
            missionOffsetForward = missionOffsetRight = 0.0f;
            missionPhase = missionPhase == MISSION_TEMP_ALIGN ?
                           MISSION_TEMP_PLACE : MISSION_COARSE_PLACE;
            missionActionStarted = 0U;
            missionReport(missionPhase == MISSION_TEMP_PLACE ?
                          "TEMP_PLACE" : "COARSE_PLACE");
        }
        return;
    }
    if (missionPhase == MISSION_COARSE_PLACE ||
        missionPhase == MISSION_COARSE_PICK ||
        missionPhase == MISSION_TEMP_PLACE) {
        if (!missionActionStarted) {
            const MechanismAction *actions = missionWorkActions;
            const MechanismInitialState *initial = &missionInitial;
            uint16_t count = MISSION_PLACE_COUNT;
            if (missionPhase == MISSION_COARSE_PLACE) {
                missionBuildTaskPlace(missionWorkActions,
                                      missionTask.rings[routeIndex == 14U], 1U, 1U);
                initial = &missionCoarseInitial;
                count = MISSION_COARSE_PLACE_COUNT;
            } else if (missionPhase == MISSION_COARSE_PICK) {
                missionBuildTaskPickBack(missionPickBackActions,
                                         missionTask.rings[routeIndex == 14U],
                                         routeIndex == 14U ? -470 : -450);
                actions = missionPickBackActions;
                initial = &missionCoarseInitial;
                count = MISSION_PICK_BACK_COUNT;
            }
            if (!mechanismActionStart(initial, actions, count)) {
                missionAbort("action_start");
                return;
            }
            missionActionStarted = 1U;
        } else if (!mechanismActionActive) {
            if (!mechanismState.valid ||
                mechanismActionIndex != mechanismActionCount) {
                missionAbort("action_failed");
                return;
            }
            if (missionPhase == MISSION_COARSE_PLACE) {
                missionPhase = MISSION_COARSE_PICK;
                missionActionStarted = 0U;
                missionReport("COARSE_PICK");
            } else {
                if (missionPhase == MISSION_COARSE_PICK &&
                    (mechanismState.current.horizontalDmm !=
                         (routeIndex == 14U ? -470 : -450) ||
                     mechanismState.current.liftDmm != 0U ||
                     mechanismState.current.turretDdeg != 1200U)) {
                    missionAbort("recovery_release_pose");
                    return;
                }
                if (missionPhase == MISSION_COARSE_PICK && routeIndex == 14U) {
                    VisionPacket packet;
                    memset(&packet, 0, sizeof(packet));
                    packet.type = VISION_MESSAGE_REQUEST;
                    packet.mode = VISION_MODE_MATERIAL;
                    packet.target = 2U;
                    packet.flags = VISION_FLAG_STACK_PREPARE;
                    packet.value[2] = 320;
                    packet.value[3] = 240;
                    packet.value[4] = 160;
                    packet.value[5] = 120;
                    visionUartSend(&packet);
                    serialSendString("VISION CAMERA PREP resolution=320x240 target=STACK\r\n");
                }
                if (missionPhase == MISSION_COARSE_PICK) {
                    missionContinue();
                    return;
                }
                missionPhase = MISSION_RECENTER;
                missionReport("RECENTER");
            }
        }
        return;
    }
    if (missionPhase == MISSION_RAW_EXIT) {
        if (motionMode) return;
        missionContinue();
        return;
    }
    if (missionPhase != MISSION_RECENTER) return;
    if (motionMode || mechanismState.running || mechanismActionActive) return;
    if (!mechanismState.valid || mechanismState.current.horizontalDmm != 0 ||
        mechanismState.current.liftDmm != 0U ||
        (mechanismState.current.turretDdeg != 2600U &&
          !((routeIndex == 4U || routeIndex == 7U ||
             routeIndex == 11U || routeIndex == 14U) &&
           mechanismState.current.turretDdeg == 530U))) {
        missionAbort("unsafe_recenter_pose");
        return;
    }
    if (presetRouteActive && (routeIndex == 4U || routeIndex == 11U)) {
        if (routeAbs(missionOffsetForward) <= 1.0f && routeAbs(missionOffsetRight) <= 1.0f) {
            missionContinue();
            return;
        }
        if (!presetRecenterClear(&presetRoute,missionOffsetForward,missionOffsetRight)) {
            missionAbort("raw_return_blocked");
            return;
        }
        if (!startVisionMotion(-missionOffsetForward,-missionOffsetRight,15U,0U)) {
            missionAbort("raw_exit_failed");
            return;
        }
        missionPhase = MISSION_RAW_EXIT;
        missionReport("RAW_EXIT_RETURN");
        return;
    }
    if (!presetRouteActive && (routeIndex == 4U || routeIndex == 11U)) {
        if (routeObstaclePosition == ROUTE_OBSTACLE_LEFT ||
            routeObstaclePosition == ROUTE_OBSTACLE_RIGHT) {
            /* Facing left: body forward is map left, body right is map up.
             * Restore map X now; carry map Y into the following down leg. */
            routeRawLeftCorrection = 0;
            routeRawDownCorrection = routeRound(missionOffsetRight);
            if (routeAbs(missionOffsetForward) <= 1.0f) {
                missionContinue();
                return;
            }
            if (!startVisionMotion(-missionOffsetForward, 0.0f, 15U, 0U)) {
                missionAbort("raw_exit_failed");
                return;
            }
            missionPhase = MISSION_RAW_EXIT;
            missionReport("RAW_EXIT_HORIZONTAL");
            return;
        }
        /* Both pickups face left: body right is map up; body forward is map left.
         * Restore map Y now and merge map X compensation into the left leg. */
        routeRawLeftCorrection = routeRound(missionOffsetForward);
        if (routeAbs(missionOffsetRight) <= 1.0f) {
            missionContinue();
            return;
        }
        if (!startVisionMotion(0.0f, -missionOffsetRight, 15U, 0U)) {
            missionAbort("raw_exit_failed");
            return;
        }
        missionPhase = MISSION_RAW_EXIT;
        missionReport("RAW_EXIT_VERTICAL");
        return;
    }
    if (routeAbs(missionOffsetForward) <= 1.0f &&
        routeAbs(missionOffsetRight) <= 1.0f) {
        missionContinue();
        return;
    }
    forward = -missionOffsetForward;
    right = -missionOffsetRight;
    if (forward > 20.0f) forward = 20.0f;
    if (forward < -20.0f) forward = -20.0f;
    if (right > 20.0f) right = 20.0f;
    if (right < -20.0f) right = -20.0f;
    if (!startVisionMotion(forward, right, 15U, 0U))
        missionAbort("recenter_failed");
}

static uint8_t processVisionCommand(const char *command)
{
    const char *cursor;
    const VisionCalibration *calibration;
    uint16_t value, rpm, sampleId;
    int16_t forward, right;
    if (visionSessionActive() && strncmp(command, "vision ", 7U) == 0) {
        failVision("COMMAND_CONFLICT");
        return 1U;
    }
    if (strncmp(command, "vision sample ring ", 19U) == 0) {
        cursor = command + 19U;
        if (!mechanismParseUnsigned(&cursor, &value) || value != 2U ||
            !mechanismParseUnsigned(&cursor, &sampleId) || sampleId == 0U ||
            !mechanismAtEnd(cursor)) {
            serialSendString("VISION ERROR reason=SAMPLE_RANGE\r\n");
            return 1U;
        }
        if (motionMode || routeActive || mechanismState.running ||
            mechanismActionActive || servoChannelMoving[0] || servoChannelMoving[1] ||
            servoChannelMoving[2] || !imuValid || clockMs - imuStamp > 250U ||
            motionInterrupted()) {
            serialSendString("VISION ERROR reason=SAMPLE_REQUIRES_IDLE_FRESH_IMU\r\n");
            return 1U;
        }
        if (!vision_session_start_sample(&visionSession, &visionRingCalibration[1],
                                          sampleId, clockMs)) {
            serialSendString("VISION ERROR reason=SAMPLE_REJECTED\r\n");
            return 1U;
        }
        printVisionState();
        return 1U;
    }
    if (strncmp(command, "vision jog ", 11U) == 0) {
        cursor = command + 11U;
        if (!mechanismParseSigned(&cursor, &forward) ||
            !mechanismParseSigned(&cursor, &right) ||
            !mechanismParseUnsigned(&cursor, &rpm) || !mechanismAtEnd(cursor) ||
            (forward == 0 && right == 0) || forward < -20 || forward > 20 ||
            right < -20 || right > 20 || rpm < 10U || rpm > 30U) {
            serialSendString("VISION ERROR reason=JOG_RANGE\r\n");
            return 1U;
        }
        if ((visionSession.state != VISION_STATE_PAUSED &&
             !(visionSession.state == VISION_STATE_IDLE && visionSession.sampleOnly)) ||
            motionMode || routeActive ||
            mechanismState.running || mechanismActionActive || !armed ||
            !imuValid || clockMs - imuStamp > 250U) {
            serialSendString("VISION ERROR reason=JOG_REQUIRES_PAUSED_ARMED_IDLE_FRESH_IMU\r\n");
            return 1U;
        }
        armed = 0U;
        if (!startVisionMotion((float)forward, (float)right, rpm, 0U)) {
            failVision("JOG_START_FAILED");
        } else {
            serialSendString("VISION STATE state=MOVE mode=MANUAL selector=0 target=0 iteration=0\r\n");
        }
        return 1U;
    }
    if (strncmp(command, "vision align ring ", 18U) != 0) return 0U;
    cursor = command + 18U;
    if (!mechanismParseUnsigned(&cursor, &value) || !mechanismAtEnd(cursor) ||
        value < 1U || value > 3U) {
        serialSendString("VISION ERROR reason=RING_RANGE\r\n");
        return 1U;
    }
    calibration = &visionRingCalibration[value - 1U];
    if (!vision_calibration_valid(calibration)) {
        serialSendString("VISION ERROR reason=CALIBRATION_INVALID\r\n");
        return 1U;
    }
    if (motionMode || routeActive || mechanismState.running ||
        mechanismActionActive || visionSessionActive() || !armed ||
        !imuValid || clockMs - imuStamp > 250U || motionInterrupted()) {
        serialSendString("VISION ERROR reason=ALIGN_REQUIRES_ARMED_IDLE_FRESH_IMU\r\n");
        return 1U;
    }
    if (!vision_session_start(&visionSession, VISION_MODE_RING, 0U, (uint8_t)value,
                              calibration, clockMs)) {
        serialSendString("VISION ERROR reason=START_REJECTED\r\n");
        return 1U;
    }
    visionSession.maxCorrections = 2U;
    armed = 0U;
    invalidateNavigation("vision_motion");
    printVisionState();
    return 1U;
}

static void prepareOfflineMission(void)
{
    MechanismPose zero = rawPickInitial.pose;
    zero.horizontalDmm = 0;
    zero.liftDmm = 0U;
    zero.turretDdeg = 530U;
    /* The user confirms the axes are physically at zero on power-on. */
    mechanismStateInitialize(&mechanismState, &zero);
    visionMaterialCalibration.calibrated = 1U;
    visionRingCalibration[1].calibrated = 1U;
    missionButtonRpm = ROUTE_RPM;
    missionButtonReady = 1U;
}

static void startButtonMission(uint8_t zone)
{
    char command[24] = "route mission 1 ";
    char digits[3];
    uint8_t count = 0U, length = 16U;
    uint16_t value = missionButtonRpm;
    if (obstacleTurretPending || motionMode || routeActive || mechanismState.running ||
        mechanismActionActive || visionSessionActive()) {
        serialSendString("BUTTON IGNORED busy\r\n");
        return;
    }
    if (!missionButtonReady || emergencyStop || hostEStop || jogCanFault) {
        serialSendString("BUTTON ERROR prepare task first\r\n");
        return;
    }
    if (!ss_obstacle_regions_valid(routeObstacleRegions)) {
        serialSendString("BUTTON ERROR select two obstacle regions first\r\n");
        return;
    }
    if (!servoChannelEnabled[0]) {
        servoSetTarget(2U, rawPickInitial.gripperCloseDeg, rawPickInitial.gripperDps10);
        gripperMayHoldMaterial = 0U;
    }
    missionButtonReady = 0U;
    command[14] = (char)('0' + zone);
    do {
        digits[count++] = (char)('0' + value % 10U);
        value /= 10U;
    } while (value != 0U);
    while (count > 0U) command[length++] = digits[--count];
    command[length] = '\0';
    /* The physical press authorizes only this start; all route checks remain. */
    missionOffline = 1U;
    armed = 1U;
    processRouteCommand(command);
    armed = 0U;
    if (!routeActive || !missionActive) {
        missionOffline = 0U;
        serialSendString("ROUTE INVALID reason=button_start_rejected\r\n");
    }
}

static void serviceMissionButton(void)
{
    uint8_t high = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) != 0U;
    if (high != missionButtonRawHigh) {
        missionButtonRawHigh = high;
        missionButtonChangedAt = clockMs;
    }
    if (high != missionButtonStableHigh &&
        clockMs - missionButtonChangedAt >= MISSION_BUTTON_DEBOUNCE_MS) {
        missionButtonStableHigh = high;
        if (!high) {
            missionButtonPressedAt = missionButtonChangedAt;
            missionButtonLongSent = 0U;
        } else if (!missionButtonLongSent) {
            missionButtonLongSent = 1U;
            startButtonMission(missionButtonChangedAt - missionButtonPressedAt >=
                               MISSION_BUTTON_LONG_PRESS_MS ? 2U : 1U);
        }
    }
    if (!high && !missionButtonStableHigh && !missionButtonLongSent &&
        clockMs - missionButtonPressedAt >= MISSION_BUTTON_LONG_PRESS_MS) {
        missionButtonLongSent = 1U;
        startButtonMission(2U);
    }
}

static void applyGripperAngles(uint16_t open, uint16_t close)
{
    rawPickInitial.gripperOpenDeg = missionInitial.gripperOpenDeg =
        missionCoarseInitial.gripperOpenDeg = open;
    rawPickInitial.gripperCloseDeg = missionInitial.gripperCloseDeg =
        missionCoarseInitial.gripperCloseDeg = close;
}

static uint8_t processRingCommand(const char *command)
{
    RingPose poses[3];
    uint8_t i;
    const char *cursor;
    if (strcmp(command, "ring poses") != 0) {
        if (strncmp(command, "ring poses ", 11U) != 0) return 0U;
        cursor = command + 10U;
        for (i = 0U; i < 3U; ++i)
            if (*cursor != ' ' || !mechanismParseSigned(&cursor, &poses[i].horizontalDmm) ||
                *cursor != ' ' || !mechanismParseUnsigned(&cursor, &poses[i].turretDdeg)) break;
        if (i != 3U || !mechanismAtEnd(cursor) || !ringPosesValid(poses)) {
            serialSendString("ERR RING: use ring poses h1 t1 h2 t2 h3 t3; horizontal -1220..850, turret 0..3600\r\n");
            return 1U;
        }
        if (motionMode || routeActive || mechanismState.running || mechanismActionActive ||
            visionSessionActive() || servoChannelMoving[0] || servoChannelMoving[1] ||
            servoChannelMoving[2]) {
            serialSendString("ERR RING: busy; stop first\r\n");
            return 1U;
        }
        if (!ringPosesSave(poses)) {
            serialSendString("ERR RING: save failed\r\n");
            return 1U;
        }
        memcpy(missionRingPoses, poses, sizeof(poses));
        ringPosesSaved = 1U;
    }
    serialSendString("RING POSES");
    for (i = 0U; i < 3U; ++i) {
        serialSendChar(' ');
        serialSendInt(missionRingPoses[i].horizontalDmm);
        serialSendChar(' ');
        serialSendUint(missionRingPoses[i].turretDdeg);
    }
    serialSendString(" saved=");
    serialSendUint(ringPosesSaved);
    serialSendString("\r\n");
    return 1U;
}

/* Private dispatch for a small typed binary command set; never accepts wire text. */
static void processCommand(const char *command)
{
    if(emergencyStop || hostEStop) return;
    if(processRingCommand(command)) return;
    if(processVisionCommand(command)) return;
    if(processAuxMoveCommand(command)) return;
    if(processRouteCommand(command)) return;
    serialSendString("ERR: unsupported internal command\r\n");
}

#include "avoidance_firmware.h"
#include "qr_camera_control.h"

#include "host_control.h"
void UART5_IRQHandler(void)
{
    uint16_t next;uint8_t b;
    if(USART_GetITStatus(UART5,USART_IT_RXNE)==RESET) return;
    b=(uint8_t)USART_ReceiveData(UART5);next=(uint16_t)((hostHead+1)%sizeof(hostRx));
    if(next==hostTail) hostOverflow=1;
    else {hostRx[hostHead]=b;hostHead=next;}
    USART_ClearITPendingBit(UART5,USART_IT_RXNE);
}

int main(void)
{
    delay_init(168U);
    board_init();
    clockInit();
    missionButtonInit();
    serialInit();
    hwt101Init(115200U);
    visionUartInit();
    qrUartInit();
    tjcDisplayInit();
    tjcDisplayResetCounts();

    mechanismStateReset(&mechanismState);
    vision_session_init(&visionSession);
    stopAllMotors();
    /* Temporarily leave all servo PWM outputs disabled on power-on. */
    prepareOfflineMission();
    {
        uint16_t open = rawPickInitial.gripperOpenDeg, close = rawPickInitial.gripperCloseDeg;
        (void)gripperAnglesLoad(&open, &close);
        applyGripperAngles(open, close);
    }
    ringPosesSaved = ringPosesLoad(missionRingPoses);
    /* Servo PWM starts only when an explicit motion command needs its channel. */
    gripperMayHoldMaterial = 0U;
    serialSendString("BOOT axes=0 servo_pwm=off; PC9 short=zone1 long=zone2; task from QR\r\n");
    serialSendString("\r\nYYB mecanum/servo jog ready; motors 1..6 stopped; disarmed.\r\n");
    hostScaleInit();

    for (;;) {
        if (jogCanFault) {
            uint8_t hadNavigation = navInitialized;
            uint8_t hadFullRoute = fullRouteRunning || missionButtonReady;
            uint8_t hadVision = visionSessionActive() || motionMode == 4U;
            if (visionPickActive) stopServoMotion();
            stopAllMotors();
            jogCanFault = 0U;
            if (hadNavigation) serialSendString("NAV INVALID reason=can_fault\r\n");
            if (hadFullRoute) serialSendString("ROUTE INVALID reason=can_fault\r\n");
            if (hadVision) failVision("CAN_FAULT");
            serialSendString("ERR: CAN transmit failed; stop attempted, check power/bus\r\n");
        }
        if (emergencyStop) {
            hostEStop=1;hostConfirmed=0;
            uint8_t hadNavigation = navInitialized;
            uint8_t hadFullRoute = fullRouteRunning || missionButtonReady;
            uint8_t hadVision = visionSessionActive() || motionMode == 4U;
            stopServoMotion();
            stopAllMotors();
            __disable_irq();
            emergencyStop = 0U;
            if (obstacleStopPending) {
                obstacleStopPending = 0U;
                __enable_irq();
                serialSendString("OBSTACLE STOP; black blob detected; disarmed\r\n");
            } else {
                __enable_irq();
            }
            if (hadNavigation) serialSendString("NAV INVALID reason=emergency_stop\r\n");
            if (hadFullRoute) serialSendString("ROUTE INVALID reason=emergency_stop\r\n");
            if (hadVision) failVision("EMERGENCY_STOP");
            serialSendString("EMERGENCY STOP; motors 1..6 stopped; disarmed\r\n");
        }

        serviceObstacleTurret();
        hostService();
        serviceHostWatchdog();
        serviceMissionButton();
        serviceMotion();
        serviceVision();
        serviceQrScan();
        serviceTjcObstacles();
        serviceMechanism();
        mechanismActionService();
        serviceArcRoute();
        serviceRoute();
        serviceAvoidance();
        serviceRawPickRoute();
        serviceMission();
        serviceQrCamera();
        reportCompletedServoMoves();
    }
}
