static void serviceMotion(void)
{
    uint32_t elapsed;
    uint8_t id,bit,status;
    if(motionMode!=4U || emergencyStop) return;
    elapsed=clockMs-motionStart;

    if (motionInterrupted()) {
        failVision("MOTION_FAULT");
        return;
    }
    if (visionSession.mode == VISION_MODE_RING) {
        for (id = 1U; id <= 4U; ++id) {
            bit = (uint8_t)(1U << (id - 1U));
            if (!(visionMotionMotorMask & bit) || !jogCanTakeDriveStatus(id, &status)) continue;
            visionMotionSeenMask |= bit;
            visionMotionStatus[id - 1U] = status;
            if (!(status & 0x01U) || (status & 0x0CU)) {
                printVisionMotionStatus();
                failVision("MOTOR_STATUS_FAULT");
                return;
            }
        }
    }
    if (elapsed >= motionDuration) {
        stopDriveMotors();
        motionMode = 0U;
        serialSendString("VISION MOVE DONE elapsed_ms=");
        serialSendUint32(elapsed);
        serialSendString("\r\n");
        if (visionMotionAutomatic) {
            vision_session_move_complete(&visionSession, clockMs);
            printVisionState();
        } else {
            vision_session_pause(&visionSession);
            serialSendString("VISION PAUSED\r\n");
        }
        return;
    }
    if (visionSession.mode == VISION_MODE_RING &&
        clockMs - lastControl >= VISION_MOVE_STATUS_POLL_MS) {
        lastControl = clockMs;
        do {
            visionMotionQueryId = (uint8_t)(visionMotionQueryId % 4U + 1U);
        } while (!(visionMotionMotorMask & (1U << (visionMotionQueryId - 1U))));
        jogCanExpectDriveStatus(visionMotionQueryId);
        Emm_V5_Read_Sys_Params(visionMotionQueryId, S_FLAG);
    }
}
