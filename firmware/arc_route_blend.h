#ifndef ARC_ROUTE_BLEND_H
#define ARC_ROUTE_BLEND_H

/* A 90deg bend with linear curvature ramps at both ends. Numerically
 * integrated displacement is 249mm on each axis, matching the old trim.
 * These are nominal geometry / software limits, not measured dynamics. */
#define ARC_BLEND_RADIUS_MM 225.0f
#define ARC_BLEND_RAMP_MM 47.19275f
#define ARC_BLEND_LENGTH_MM (1.570796327f * ARC_BLEND_RADIUS_MM + ARC_BLEND_RAMP_MM)
#define ARC_BLEND_GEOMETRY_MM 207.5f
#define ARC_BLEND_EDGE_RPM 36.0f
#define ARC_BLEND_WHEEL_RPM 250U
#define ARC_BLEND_ACCEL_RPM_S 220.0f
#define ARC_BLEND_BRAKE_RPM_S 180.0f

typedef struct {
    float distance, forward, turn, previousYaw, turned;
    float startYaw, rateDps;
    int8_t direction, travel, exitHeading;
    uint32_t tick, start, sample, report;
} ArcRouteBlend;

static float arcBlendTurnLimit(float remaining, float rateDps, uint32_t ageMs,
                               uint16_t trim)
{
    float degreesPerRpm = ROUTE_MM_PER_REV / 60.0f / ARC_BLEND_GEOMETRY_MM * 57.29577951f;
    /* Look ahead across the heading PID's 100ms derivative filter plus
     * sample age; this is a software margin, not a measured stopping time. */
    if (rateDps > 0.0f) remaining -= rateDps * (0.1f + ageMs / 1000.0f);
    if (remaining <= 0.0f) return 0.0f;
    /* Half the wheel braking budget remains available to reduce yaw
     * while forward speed changes. sqrt(2 * (accel/2) * remaining). */
    return sqrtf(ARC_BLEND_BRAKE_RPM_S * 1000.0f / trim * remaining / degreesPerRpm);
}

static void arcBlendShape(float distance, float *degrees, float *curvature)
{
    float tail = ARC_BLEND_LENGTH_MM - distance;
    if (distance < 0.0f) distance = 0.0f;
    if (tail < 0.0f) tail = 0.0f;
    if (distance < ARC_BLEND_RAMP_MM) {
        *degrees = distance * distance / (2.0f * ARC_BLEND_RADIUS_MM * ARC_BLEND_RAMP_MM);
        *curvature = distance / (ARC_BLEND_RADIUS_MM * ARC_BLEND_RAMP_MM);
    } else if (tail < ARC_BLEND_RAMP_MM) {
        *degrees = 1.570796327f - tail * tail /
                   (2.0f * ARC_BLEND_RADIUS_MM * ARC_BLEND_RAMP_MM);
        *curvature = tail / (ARC_BLEND_RADIUS_MM * ARC_BLEND_RAMP_MM);
    } else {
        *degrees = (distance - ARC_BLEND_RAMP_MM * 0.5f) / ARC_BLEND_RADIUS_MM;
        *curvature = 1.0f / ARC_BLEND_RADIUS_MM;
    }
    *degrees *= 57.29577951f;
}

static float arcBlendSpeed(float distance, float scale, float wheelLimit)
{
    float tail = ARC_BLEND_LENGTH_MM - ARC_BLEND_RAMP_MM - distance;
    float ratio = ARC_BLEND_GEOMETRY_MM * scale / ARC_BLEND_RADIUS_MM;
    float deceleration = ARC_BLEND_BRAKE_RPM_S * wheelLimit / ARC_BLEND_WHEEL_RPM - 20.0f;
    float rpm, limit = (wheelLimit - ROUTE_HEADING_MAX_RPM) / (1.0f + ratio);
    if (distance < ARC_BLEND_RAMP_MM || tail < 0.0f) return ARC_BLEND_EDGE_RPM;
    /* Reserve 20 RPM/s for the IMU correction; outer wheels bound braking. */
    rpm = sqrtf(ARC_BLEND_EDGE_RPM * ARC_BLEND_EDGE_RPM +
        2.0f * deceleration / (1.0f + ratio) * tail /
        (ROUTE_MM_PER_REV / 60.0f * scale));
    if (rpm > limit) rpm = limit;
    return rpm;
}

#endif
