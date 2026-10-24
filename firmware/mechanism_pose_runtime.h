static void enableMechanismAxes(uint8_t axes)
{
    uint8_t i, bit;
    for (i = 0U; i < 2U; ++i) {
        bit = (uint8_t)(1U << i);
        if (!(axes & bit) || (mechanismEnabledAxes & bit)) continue;
        Emm_V5_En_Control((uint8_t)(5U + i), true, false);
        delay_ms(2U);
        if (jogCanFault) return;
        mechanismEnabledAxes |= bit;
    }
}

static void startMechanismPose(const MechanismPose *target)
{
    int16_t horizontalDelta = (int16_t)(target->horizontalDmm -
                                        mechanismState.current.horizontalDmm);
    int16_t liftDelta = (int16_t)((int32_t)target->liftDmm -
                                  (int32_t)mechanismState.current.liftDmm);
    uint32_t horizontalPulses = mechanismHorizontalPulses(horizontalDelta);
    uint32_t liftPulses = mechanismLiftPulses(liftDelta);

    if (motionInterrupted()) { stopServoMotion(); stopAllMotors(); return; }
    if (!mechanismStateStart(&mechanismState, target, clockMs)) {
        serialSendString("ERR MECH: invalid or busy pose\r\n");
        return;
    }
    /* Skip only a fully completed command pose; disabled PWM still needs dispatch. */
    if (!horizontalPulses && !liftPulses && servoChannelEnabled[2] &&
        !servoChannelMoving[2] &&
        currentAngleMdeg[2] == (uint32_t)target->turretDdeg * 100U &&
        targetAngleMdeg[2] == (uint32_t)target->turretDdeg * 100U) {
        mechanismState.current = *target;
        mechanismState.running = 0U;
        return;
    }
    if (!gripperMayHoldMaterial) {
        MechanismPose axisTarget = *target;
        uint32_t delta = mechanismAbsoluteDelta((int16_t)(target->turretDdeg -
            mechanismState.start.turretDdeg)) * 100U;
        uint32_t turretMs = turretMotionDurationMs(delta, SERVO_EMPTY_TURRET_SPEED_DPS10);
        axisTarget.turretDdeg = mechanismState.start.turretDdeg;
        mechanismState.durationMs = mechanismMoveDurationMs(&mechanismState.start, &axisTarget);
        if (turretMs > mechanismState.durationMs) mechanismState.durationMs = turretMs;
        mechanismState.deadlineMs = clockMs + mechanismState.durationMs;
    }
    mechanismAxes=0;mechanismBaselineMask=0;
    mechanismSignedPulses[0]=liftDelta>0?(int32_t)liftPulses:-(int32_t)liftPulses;
    mechanismSignedPulses[1]=horizontalDelta>0?(int32_t)horizontalPulses:-(int32_t)horizontalPulses;
    if(liftPulses) mechanismAxes|=1;
    if(horizontalPulses) mechanismAxes|=2;
    mechanismPreparing=1;mechanismPollAxis=0;mechanismPositionPriority=0;
    mechanismPollKind[0]=mechanismPollKind[1]=0;
    mechanismPollAt=clockMs-MECHANISM_FEEDBACK_POLL_MS;mechanismFeedbackTimeout=clockMs+3000;
    enableMechanismAxes(axis6TestActive() ? 2U : 3U);
    if (motionInterrupted()) { stopServoMotion(); stopAllMotors(); return; }
    jogCanClearReply(5,0x33);jogCanClearReply(6,0x33);
}
static void dispatchMechanismPose(void)
{
    const MechanismPose *target=&mechanismState.target;
    int16_t horizontalDelta=(int16_t)(target->horizontalDmm-mechanismState.start.horizontalDmm);
    int16_t liftDelta=(int16_t)((int32_t)target->liftDmm-mechanismState.start.liftDmm);
    uint8_t i;
    for(i=0;i<2;++i) {
        axisBegin(&mechanismFeedback[i],mechanismBaseline[i],mechanismSignedPulses[i]);
        jogCanClearReply((uint8_t)(5+i),0x33);jogCanClearReply((uint8_t)(5+i),0x3a);
        jogCanClearReply((uint8_t)(5+i),0x36);
    }
    mechanismPreparing=0;mechanismPollAxis=0;
    mechanismPollKind[0]=mechanismPollKind[1]=0;
    mechanismState.startMs=clockMs;
    mechanismFeedbackTimeout=clockMs+mechanismState.durationMs*2+3000;

    if (motionInterrupted()) { stopServoMotion(); stopAllMotors(); return; }
    if (mechanismHorizontalPulses(horizontalDelta) != 0U) {
        Emm_V5_Pos_Control(6U, horizontalDelta > 0 ? 0U : 1U,
                           target->horizontalRpm, target->horizontalAccel,
                           mechanismHorizontalPulses(horizontalDelta), false, true);
        delay_ms(2U); /* Allow the drive to cache the command before synchronous start. */
    }
    if (motionInterrupted()) { stopServoMotion(); stopAllMotors(); return; }
    if (mechanismLiftPulses(liftDelta) != 0U) {
        Emm_V5_Pos_Control(5U, liftDelta > 0 ? 0U : 1U,
                           target->liftRpm, target->liftAccel,
                           mechanismLiftPulses(liftDelta), false, true);
        delay_ms(2U);
    }
    if (!axis6TestActive())
        servoSetTargetMdeg(4U, (uint32_t)target->turretDdeg * 100U,
                           target->turretDps10);
    if (motionInterrupted()) { stopServoMotion(); stopAllMotors(); return; }
    Emm_V5_Synchronous_motion(axis6TestActive() ? 6U : 0x00);
    if (motionInterrupted()) { stopServoMotion(); stopAllMotors(); return; }
    printMechanismPose("MECH RUN", target);
}
