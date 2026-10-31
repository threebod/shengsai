#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

#include <stdint.h>
#include <string.h>

#define VISION_FRAME_SIZE 30U
#define VISION_VALUE_COUNT 8U
#define VISION_ALIGN_TOLERANCE_PX 1
#define VISION_RAW_ALIGN_RPM 50U
#define VISION_LAYOUT_ACK_MS 1000U

#define VISION_MESSAGE_REQUEST 1U
#define VISION_MESSAGE_RESULT 2U
#define VISION_MODE_MATERIAL 1U
#define VISION_MODE_RING 2U
#define VISION_FLAG_VALID 0x01U
#define VISION_FLAG_STABLE 0x02U
#define VISION_FLAG_RING_NO_DIGIT 0x04U /* Request only; selector is the fallback color. */
#define VISION_FLAG_MATERIAL_LAYOUT 0x08U
#define VISION_FLAG_COLOR_INFERRED 0x10U /* Result only. */
#define VISION_FLAG_STACK_PREPARE 0x20U /* Request only; prepare stack material or numbered ring 2, without detecting. */
#define VISION_FLAG_LAYOUT_REFERENCE 0x40U /* Result only; actual center is a rotation reference. */
#define VISION_FLAG_TRACKING 0x80U
#define VISION_TRACK_START 1
#define VISION_TRACK_END (-1)
#define VISION_LAYOUT_TRACK 0U
#define VISION_LAYOUT_CONFIRM 1U
#define VISION_LAYOUT_END 2U
#define VISION_LAYOUT_LOCATE 3U
#define VISION_PICK_POSITION_NONE 0U
#define VISION_PICK_POSITION_BOTTOM 1U
#define VISION_PICK_POSITION_LEFT_UP 2U
#define VISION_PICK_POSITION_RIGHT_UP 3U

typedef struct {
    uint8_t type;
    uint32_t token;
    uint8_t mode;
    uint8_t selector;
    uint8_t target;
    uint8_t flags;
    int16_t value[VISION_VALUE_COUNT];
} VisionPacket;

typedef struct {
    uint8_t bytes[VISION_FRAME_SIZE];
    uint8_t length;
} VisionParser;

typedef struct {
    uint8_t calibrated;
    int16_t roi[4];
    int16_t anchorU;
    int16_t anchorV;
    float matrix[4];
} VisionCalibration;

typedef enum {
    VISION_STATE_IDLE = 0,
    VISION_STATE_REQUEST,
    VISION_STATE_WAIT,
    VISION_STATE_MOVE,
    VISION_STATE_SETTLE,
    VISION_STATE_ALIGNED,
    VISION_STATE_FAILED,
    VISION_STATE_PAUSED,
    VISION_STATE_PREP,
    VISION_STATE_PICK,
    VISION_STATE_PICK_DONE,
    VISION_STATE_WAIT_ACTIONS,
    VISION_STATE_CORRECTION_LIMIT
} VisionState;

typedef enum {
    VISION_EVENT_NONE = 0,
    VISION_EVENT_REQUEST,
    VISION_EVENT_MOVE,
    VISION_EVENT_ALIGNED,
    VISION_EVENT_FAILED,
    VISION_EVENT_SAMPLED,
    VISION_EVENT_CORRECTION_LIMIT
} VisionEvent;

typedef struct {
    VisionState state;
    uint8_t mode;
    uint8_t selector;
    uint8_t target;
    uint8_t misses;
    uint8_t confirmations;
    uint8_t iteration;
    uint8_t maxCorrections; /* Zero retains the normal convergence limit. */
    uint8_t waitForTarget;
    uint8_t materialMask;
    uint8_t pickupMode; /* 0 normal alignment, 1 any-color pickup alignment, 2 specified-color pickup. */
    uint8_t observedColor;
    uint8_t requestFlags;
    uint8_t layoutPhase;
    uint8_t layoutColors[3]; /* Bottom, left upper, right upper. */
    uint8_t layoutInferred;
    uint8_t layoutPositioned; /* After first alignment, a lone blob cannot identify a grasp slot. */
    uint8_t layoutFrameReceived;
    uint8_t sampleOnly;
    uint16_t sampleId;
    uint32_t token;
    uint32_t requestAt;
    uint32_t settleUntil;
    float forwardMm;
    float rightMm;
    float cumulativeMm;
    const char *fault;
} VisionSession;

static uint16_t vision_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;
    for (index = 0U; index < length; ++index) {
        crc ^= (uint16_t)data[index] << 8;
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void vision_write_u32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
    out[2] = (uint8_t)(value >> 16);
    out[3] = (uint8_t)(value >> 24);
}

static uint32_t vision_read_u32(const uint8_t *in)
{
    return (uint32_t)in[0] | ((uint32_t)in[1] << 8) |
           ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
}

static void vision_packet_encode(const VisionPacket *packet,
                                 uint8_t frame[VISION_FRAME_SIZE])
{
    uint8_t index;
    uint16_t crc;
    memset(frame, 0, VISION_FRAME_SIZE);
    frame[0] = 0xA5U;
    frame[1] = 0x5AU;
    frame[2] = 1U;
    frame[3] = packet->type;
    vision_write_u32(frame + 4, packet->token);
    frame[8] = packet->mode;
    frame[9] = packet->selector;
    frame[10] = packet->target;
    frame[11] = packet->flags;
    for (index = 0U; index < VISION_VALUE_COUNT; ++index) {
        frame[12U + index * 2U] = (uint8_t)packet->value[index];
        frame[13U + index * 2U] = (uint8_t)((uint16_t)packet->value[index] >> 8);
    }
    crc = vision_crc16(frame, 28U);
    frame[28] = (uint8_t)crc;
    frame[29] = (uint8_t)(crc >> 8);
}

static void vision_parser_init(VisionParser *parser)
{
    parser->length = 0U;
}

static uint8_t vision_parser_feed(VisionParser *parser, const uint8_t *data,
                                  uint16_t length, VisionPacket *packet)
{
    uint16_t offset;
    uint8_t index;
    for (offset = 0U; offset < length; ++offset) {
        uint8_t byte = data[offset];
        if (parser->length == 0U && byte != 0xA5U) continue;
        if (parser->length == 1U && byte != 0x5AU) {
            parser->length = byte == 0xA5U ? 1U : 0U;
            continue;
        }
        parser->bytes[parser->length++] = byte;
        if (parser->length != VISION_FRAME_SIZE) continue;
        parser->length = 0U;
        if (parser->bytes[2] != 1U ||
            vision_crc16(parser->bytes, 28U) !=
                ((uint16_t)parser->bytes[28] | ((uint16_t)parser->bytes[29] << 8))) {
            continue;
        }
        packet->type = parser->bytes[3];
        packet->token = vision_read_u32(parser->bytes + 4);
        packet->mode = parser->bytes[8];
        packet->selector = parser->bytes[9];
        packet->target = parser->bytes[10];
        packet->flags = parser->bytes[11];
        for (index = 0U; index < VISION_VALUE_COUNT; ++index) {
            packet->value[index] = (int16_t)((uint16_t)parser->bytes[12U + index * 2U] |
                                  ((uint16_t)parser->bytes[13U + index * 2U] << 8));
        }
        return 1U;
    }
    return 0U;
}

static uint8_t vision_calibration_valid(const VisionCalibration *calibration)
{
    return calibration != (const VisionCalibration *)0 && calibration->calibrated &&
           calibration->roi[2] > 0 && calibration->roi[3] > 0;
}

static void vision_map_error(const VisionCalibration *calibration,
                             int16_t du, int16_t dv,
                             float *forward, float *right)
{
    *forward = calibration->matrix[0] * du + calibration->matrix[1] * dv;
    *right = calibration->matrix[2] * du + calibration->matrix[3] * dv;
}

static void vision_limit_step(float *forward, float *right, float maximum)
{
    float largest = *forward < 0.0f ? -*forward : *forward;
    float lateral = *right < 0.0f ? -*right : *right;
    float scale;
    if (lateral > largest) largest = lateral;
    if (largest <= maximum || largest == 0.0f) return;
    scale = maximum / largest;
    *forward *= scale;
    *right *= scale;
}

static void vision_session_init(VisionSession *session)
{
    memset(session, 0, sizeof(*session));
    session->fault = "NONE";
}

static uint8_t vision_session_start(VisionSession *session, uint8_t mode,
                                    uint8_t selector, uint8_t target,
                                    const VisionCalibration *calibration,
                                    uint32_t now)
{
    uint32_t token = session->token;
    (void)now;
    if (!vision_calibration_valid(calibration) ||
        (mode != VISION_MODE_MATERIAL && mode != VISION_MODE_RING) ||
        (mode == VISION_MODE_MATERIAL && selector > 6U) ||
        (mode == VISION_MODE_RING && (target < 1U || target > 3U))) return 0U;
    vision_session_init(session);
    session->token = token;
    session->mode = mode;
    session->selector = selector;
    session->target = target;
    session->waitForTarget = mode == VISION_MODE_RING;
    session->state = VISION_STATE_REQUEST;
    return 1U;
}

static void vision_session_make_request(VisionSession *session,
                                        const VisionCalibration *calibration,
                                        VisionPacket *packet, uint32_t now)
{
    memset(packet, 0, sizeof(*packet));
    packet->type = VISION_MESSAGE_REQUEST;
    packet->token = ++session->token;
    packet->mode = session->mode;
    packet->selector = session->selector;
    packet->target = session->target;
    packet->flags = session->requestFlags;
    packet->value[0] = calibration->roi[0];
    packet->value[1] = calibration->roi[1];
    packet->value[2] = calibration->roi[2];
    packet->value[3] = calibration->roi[3];
    packet->value[4] = calibration->anchorU;
    packet->value[5] = calibration->anchorV;
    if (session->mode == VISION_MODE_MATERIAL)
        packet->value[6] = session->materialMask;
    if (session->requestFlags & VISION_FLAG_MATERIAL_LAYOUT) {
        session->layoutFrameReceived = 0U;
        packet->value[4] = 160;
        packet->value[5] = 205;
        packet->value[7] = session->layoutPhase;
        if (session->layoutPhase == VISION_LAYOUT_CONFIRM && !session->layoutPositioned)
            packet->value[7] = VISION_LAYOUT_LOCATE;
    }
    session->requestAt = now;
    if (!(session->requestFlags & VISION_FLAG_MATERIAL_LAYOUT) ||
        (session->state != VISION_STATE_MOVE && session->state != VISION_STATE_SETTLE &&
         session->state != VISION_STATE_WAIT_ACTIONS)) session->state = VISION_STATE_WAIT;
}

/* No action table is available yet. This returns a position only for a fresh,
 * parked layout; the caller must also verify chassis/servo/axis completion. */
static uint8_t vision_layout_pick_position(const VisionSession *session, uint8_t color)
{
    uint8_t index;
    if (session->state != VISION_STATE_WAIT_ACTIONS) return VISION_PICK_POSITION_NONE;
    for (index = 0U; index < 3U; ++index)
        if (session->layoutColors[index] == color) return (uint8_t)(index + 1U);
    return VISION_PICK_POSITION_NONE;
}

static VisionEvent vision_layout_observe(VisionSession *session,
                                         const VisionCalibration *calibration,
                                         const VisionPacket *packet, uint32_t now)
{
    uint8_t index, mask = 0U, single, targetSlot = 0U, referenceSlot;
    uint8_t wasReady = session->state == VISION_STATE_WAIT_ACTIONS;
    int16_t du, dv;
    if (packet->type != VISION_MESSAGE_RESULT || packet->token != session->token ||
        packet->mode != session->mode || packet->selector != session->selector ||
        packet->target != session->target) return VISION_EVENT_NONE;
    if (!(packet->flags & VISION_FLAG_MATERIAL_LAYOUT)) {
        session->state = VISION_STATE_FAILED;
        session->fault = "CAMERA_LAYOUT_UNSUPPORTED";
        return VISION_EVENT_FAILED;
    }
    session->requestAt = now;
    session->layoutFrameReceived = 1U;
    session->misses = 0U;
    if (session->state == VISION_STATE_MOVE || session->state == VISION_STATE_SETTLE ||
        session->layoutPhase != VISION_LAYOUT_CONFIRM) return VISION_EVENT_NONE;
    for (index = 0U; index < 3U; ++index) {
        int16_t color = packet->value[5U + index];
        uint8_t bit;
        if (session->selector && color == 0) continue;
        if (color < 1 || color > 6) break;
        bit = (uint8_t)(1U << (color - 1));
        if (mask & bit) break;
        mask |= bit;
        if (color == session->selector) targetSlot = (uint8_t)(index + 1U);
    }
    single = !session->selector && index == 1U && packet->value[6] == 0 && packet->value[7] == 0 &&
             mask != 0U && (mask & session->materialMask) == mask &&
             !(packet->flags & VISION_FLAG_COLOR_INFERRED);
    referenceSlot = packet->value[1] >= 120 ? 1U : packet->value[0] < 160 ? 2U :
                    packet->value[0] > 160 ? 3U : 0U;
    if (((index != 3U || mask != session->materialMask) &&
         !(single && !session->layoutPositioned)) ||
        (packet->flags & (VISION_FLAG_VALID | VISION_FLAG_STABLE)) !=
            (VISION_FLAG_VALID | VISION_FLAG_STABLE) ||
        packet->value[0] < 0 || packet->value[0] >= 320 ||
        packet->value[1] < 0 || packet->value[1] >= 240 ||
        (!session->layoutPositioned && session->iteration && packet->value[1] < 120) ||
        ((packet->flags & VISION_FLAG_LAYOUT_REFERENCE) &&
         (!session->selector || !(packet->flags & VISION_FLAG_COLOR_INFERRED) ||
          !referenceSlot || !packet->value[4U + referenceSlot])) ||
        (session->selector && (!targetSlot ||
         (!(packet->flags & VISION_FLAG_LAYOUT_REFERENCE) && targetSlot != referenceSlot)))) {
        memset(session->layoutColors, 0, sizeof(session->layoutColors));
        session->layoutInferred = 0U;
        session->state = VISION_STATE_WAIT;
        return VISION_EVENT_NONE;
    }
    du = (int16_t)(packet->value[0] - 160);
    dv = (int16_t)(packet->value[1] - 205);
    vision_map_error(calibration, du, dv, &session->forwardMm, &session->rightMm);
    if (session->layoutPositioned || session->selector ||
        (du >= -10 && du <= 10 && dv >= -10 && dv <= 10)) {
        if (!session->layoutPositioned) {
            session->layoutPositioned = 1U;
            /* The locating round only positions the chassis. Even a complete
             * layout needs a fresh color-confirmation request before grasping. */
            memset(session->layoutColors, 0, sizeof(session->layoutColors));
            session->layoutInferred = 0U;
            session->state = VISION_STATE_WAIT;
            return VISION_EVENT_NONE;
        }
        for (index = 0U; index < 3U; ++index) {
            if (session->layoutColors[index] != (uint8_t)packet->value[5U + index]) wasReady = 0U;
            session->layoutColors[index] = (uint8_t)packet->value[5U + index];
        }
        session->layoutInferred = (packet->flags & VISION_FLAG_COLOR_INFERRED) != 0U;
        session->state = VISION_STATE_WAIT_ACTIONS;
        return wasReady ? VISION_EVENT_NONE : VISION_EVENT_ALIGNED;
    }
    memset(session->layoutColors, 0, sizeof(session->layoutColors));
    if (++session->iteration > 12U) {
        session->state = VISION_STATE_FAILED;
        session->fault = "NOT_CONVERGED";
        return VISION_EVENT_FAILED;
    }
    session->state = VISION_STATE_MOVE;
    session->layoutPhase = VISION_LAYOUT_TRACK;
    return VISION_EVENT_MOVE;
}

static uint8_t vision_session_start_material_sample(VisionSession *session,
                                                    const VisionCalibration *calibration,
                                                    uint8_t color, uint16_t sampleId,
                                                    uint32_t now)
{
    VisionCalibration sampling;
    if (calibration == 0 || calibration->roi[2] <= 0 || calibration->roi[3] <= 0 ||
        color < 1U || color > 6U || sampleId == 0U) return 0U;
    sampling = *calibration;
    sampling.calibrated = 1U;
    if (!vision_session_start(session, VISION_MODE_MATERIAL, color, 0U,
                              &sampling, now)) return 0U;
    session->sampleOnly = 1U;
    session->sampleId = sampleId;
    return 1U;
}

/* Calibration sampling never uses the existing pixel-to-movement matrix. */
static uint8_t vision_session_start_sample(VisionSession *session,
                                           const VisionCalibration *calibration,
                                           uint16_t sampleId, uint32_t now)
{
    VisionCalibration sampling;
    if (calibration == 0 || calibration->roi[2] <= 0 ||
        calibration->roi[3] <= 0 || sampleId == 0U) return 0U;
    sampling = *calibration;
    sampling.calibrated = 1U;
    if (!vision_session_start(session, VISION_MODE_RING, 0U, 2U,
                              &sampling, now)) return 0U;
    session->sampleOnly = 1U;
    session->sampleId = sampleId;
    return 1U;
}

static VisionEvent vision_session_miss(VisionSession *session,
                                       const char *fault)
{
    if (++session->misses >= 3U) {
        session->state = VISION_STATE_FAILED;
        session->fault = fault;
        return VISION_EVENT_FAILED;
    }
    session->state = VISION_STATE_REQUEST;
    return VISION_EVENT_REQUEST;
}

static uint8_t vision_material_allowed(const VisionSession *session,
                                       const VisionPacket *packet)
{
    int16_t color = packet->value[5];
    return color >= 1 && color <= 6 &&
           (session->selector == 0U || color == session->selector) &&
           (session->materialMask == 0U ||
            (session->materialMask & (1U << (color - 1))) != 0U);
}

static uint8_t vision_quality_valid(const VisionPacket *packet)
{
    return packet->mode == VISION_MODE_MATERIAL || packet->value[2] >= 60;
}

static uint8_t vision_session_wait_for_material(VisionSession *session,
                                                const VisionPacket *packet)
{
    if (session->state != VISION_STATE_WAIT ||
        packet->type != VISION_MESSAGE_RESULT || packet->token != session->token ||
        packet->mode != VISION_MODE_MATERIAL || packet->mode != session->mode ||
        packet->selector != session->selector || packet->target != session->target ||
        ((packet->flags & (VISION_FLAG_VALID | VISION_FLAG_STABLE)) ==
             (VISION_FLAG_VALID | VISION_FLAG_STABLE) &&
         vision_quality_valid(packet) &&
         vision_material_allowed(session, packet)))
        return 0U;
    session->state = VISION_STATE_REQUEST;
    session->misses = session->confirmations = 0U;
    return 1U;
}

static VisionEvent vision_session_observe(VisionSession *session,
                                          const VisionCalibration *calibration,
                                          const VisionPacket *packet,
                                          uint32_t now)
{
    float forward;
    float right;
    float absForward;
    float absRight;
    uint8_t aligned;
    int16_t du, dv;
    (void)now;
    if (session->state != VISION_STATE_WAIT ||
        packet->type != VISION_MESSAGE_RESULT || packet->token != session->token ||
        packet->mode != session->mode || packet->selector != session->selector ||
        packet->target != session->target) return VISION_EVENT_NONE;
    if (session->sampleOnly) {
        if ((packet->flags & (VISION_FLAG_VALID | VISION_FLAG_STABLE)) !=
                (VISION_FLAG_VALID | VISION_FLAG_STABLE) ||
            !vision_quality_valid(packet) ||
            (session->mode == VISION_MODE_MATERIAL &&
             !vision_material_allowed(session, packet))) {
            return vision_session_miss(session, "SAMPLE_UNCONFIRMED");
        }
        session->state = VISION_STATE_IDLE;
        return VISION_EVENT_SAMPLED;
    }
    if ((packet->flags & (VISION_FLAG_VALID | VISION_FLAG_STABLE)) !=
            (VISION_FLAG_VALID | VISION_FLAG_STABLE) ||
        !vision_quality_valid(packet) ||
        (session->mode == VISION_MODE_MATERIAL &&
         !vision_material_allowed(session, packet))) {
        session->confirmations = 0U;
        if (session->waitForTarget) {
            session->misses = 0U;
            session->state = VISION_STATE_REQUEST;
            return VISION_EVENT_REQUEST;
        }
        return vision_session_miss(session, "UNCONFIRMED");
    }
    session->misses = 0U;
    if (session->mode == VISION_MODE_MATERIAL && session->materialMask != 0U) {
        if (session->observedColor != (uint8_t)packet->value[5])
            session->confirmations = 0U;
        session->observedColor = (uint8_t)packet->value[5];
    }
    du = (int16_t)(packet->value[0] - calibration->anchorU);
    dv = (int16_t)(packet->value[1] - calibration->anchorV);
    vision_map_error(calibration, du, dv, &forward, &right);
    if (session->mode == VISION_MODE_MATERIAL && session->pickupMode != 0U) {
        /* Pickup depth is applied to the horizontal axis at grasp time. */
        right = 0.0f;
    }
    session->forwardMm = forward;
    session->rightMm = right;
    absForward = forward < 0.0f ? -forward : forward;
    absRight = right < 0.0f ? -right : right;
    aligned = (uint8_t)(absForward <= 3.0f && absRight <= 3.0f);
    if (session->mode == VISION_MODE_RING || session->target != 0U) {
        aligned = (uint8_t)(du >= 0 && du <= VISION_ALIGN_TOLERANCE_PX &&
                            dv >= 0 && dv <= VISION_ALIGN_TOLERANCE_PX);
    }
    if (session->mode == VISION_MODE_MATERIAL && session->pickupMode != 0U) {
        aligned = (uint8_t)(du >= -17 && du <= 17);
    }
    if (aligned) {
        if (++session->confirmations >= (session->pickupMode != 0U ? 1U : 2U)) {
            session->state = VISION_STATE_ALIGNED;
            return VISION_EVENT_ALIGNED;
        }
        session->state = VISION_STATE_REQUEST;
        return VISION_EVENT_REQUEST;
    }
    session->confirmations = 0U;
    if (++session->iteration > 12U) {
        session->state = VISION_STATE_FAILED;
        session->fault = "NOT_CONVERGED";
        return VISION_EVENT_FAILED;
    }
    session->cumulativeMm += absForward + absRight;
    session->state = VISION_STATE_MOVE;
    return VISION_EVENT_MOVE;
}

static void vision_session_move_complete(VisionSession *session, uint32_t now)
{
    session->state = VISION_STATE_SETTLE;
    session->settleUntil = now +
        ((session->requestFlags & VISION_FLAG_MATERIAL_LAYOUT) ? 0U : 300U);
}

static VisionEvent vision_session_tick(VisionSession *session, uint32_t now)
{
    if (session->requestFlags & VISION_FLAG_MATERIAL_LAYOUT) {
        if (session->state != VISION_STATE_REQUEST && session->state != VISION_STATE_PREP &&
            session->state != VISION_STATE_FAILED && session->state != VISION_STATE_PAUSED &&
            now - session->requestAt >= (session->layoutFrameReceived ? 1800U : VISION_LAYOUT_ACK_MS)) {
            memset(session->layoutColors, 0, sizeof(session->layoutColors));
            return vision_session_miss(session, "CAMERA_TIMEOUT");
        }
        if (session->state == VISION_STATE_SETTLE &&
            (int32_t)(now - session->settleUntil) >= 0) session->state = VISION_STATE_WAIT;
        return VISION_EVENT_NONE;
    }
    if (session->state == VISION_STATE_WAIT && now - session->requestAt >= 1800U) {
        return vision_session_miss(session, "CAMERA_TIMEOUT");
    }
    if (session->state == VISION_STATE_SETTLE &&
        (int32_t)(now - session->settleUntil) >= 0) {
        if (session->maxCorrections && session->iteration >= session->maxCorrections) {
            session->state = VISION_STATE_CORRECTION_LIMIT;
            return VISION_EVENT_CORRECTION_LIMIT;
        }
        session->state = VISION_STATE_REQUEST;
        return VISION_EVENT_REQUEST;
    }
    return VISION_EVENT_NONE;
}

static void vision_session_pause(VisionSession *session)
{
    session->state = VISION_STATE_PAUSED;
}

#endif
