static void serviceMechanism(void)
{
    uint8_t i, pending=0; int32_t value;
    if(!mechanismState.running) return;
    if(motionInterrupted()) { stopServoMotion();stopAllMotors();return; }
    if((int32_t)(clockMs-mechanismFeedbackTimeout)>=0) {
        serialSendString("ERR MECH: feedback timeout; coordinates invalid\r\n");
        stopServoMotion();stopAllMotors();
        for(i=0;i<2;++i) if(mechanismAxes&(1U<<i)) {
            serialSendString("AXIS TIMEOUT motor=");serialSendUint((uint16_t)(5+i));
            serialSendString(" preparing=");serialSendUint(mechanismPreparing);
            serialSendString(" baseline_ok=");serialSendUint((mechanismBaselineMask>>i)&1U);
            serialSendString(" target_ok=");serialSendUint(mechanismFeedback[i].targetVerified);
            serialSendString(" reached=");serialSendUint(mechanismFeedback[i].reached);
            serialSendString(" actual_after_arrival=");serialSendUint(mechanismFeedback[i].actualAfterArrival);
            serialSendString(" elapsed_ms=");serialSendUint32(clockMs-mechanismState.startMs);
            serialSendString("\r\n");
        }
        return;
    }
    for(i=0;i<2;++i) {
        uint8_t id=(uint8_t)(i+5),bit=(uint8_t)(1U<<i);
        if(!(mechanismAxes&bit)) continue;
        if(mechanismPreparing) {
            if(jogCanTakeReply(id,0x33,&value)) { mechanismBaseline[i]=value;mechanismBaselineMask|=bit; }
            continue;
        }
        if(jogCanTakeReply(id,0x33,&value)) {
            axisTarget(&mechanismFeedback[i],value);
            /* Ignore status received before target verification. */
            jogCanClearReply(id,0x3a);
        }
        if(jogCanTakeReply(id,0x36,&value)) {
            mechanismFeedback[i].actual=value;
            if(mechanismFeedback[i].reached) mechanismFeedback[i].actualAfterArrival=1;
        }
        if(jogCanTakeReply(id,0x3a,&value) && axisStatus(&mechanismFeedback[i],(uint8_t)value)<0) {
            serialSendString("ERR MECH: motor disabled/stalled; coordinates invalid\r\n");
            stopServoMotion();stopAllMotors();return;
        }
        if(!mechanismFeedback[i].reached || !mechanismFeedback[i].actualAfterArrival) pending|=bit;
    }
    if(mechanismPreparing && (mechanismBaselineMask&mechanismAxes)==mechanismAxes) {
        dispatchMechanismPose();return;
    }
    if(!mechanismPreparing && !pending && !servoChannelMoving[2]) {
        mechanismState.current=mechanismState.target;mechanismState.running=0;
        printMechanismPose("MECH DONE",&mechanismState.current);
        for(i=0;i<2;++i) if(mechanismAxes&(1U<<i)) {
            serialSendString("AXIS DONE motor=");serialSendUint((uint16_t)(5+i));
            serialSendString(" expected_enc=");serialSendUint32((uint32_t)mechanismFeedback[i].expected);
            serialSendString(" actual_enc=");serialSendUint32((uint32_t)mechanismFeedback[i].actual);
            serialSendString(" elapsed_ms=");serialSendUint32(clockMs-mechanismState.startMs);serialSendString("\r\n");
        }
        return;
    }
    if(clockMs-mechanismPollAt>=MECHANISM_FEEDBACK_POLL_MS) {
        uint8_t id;SysParams_t param;
        mechanismPollAt=clockMs;
        for(i=0;i<2;++i) {
            mechanismPollAxis^=1;
            if(mechanismAxes&(1U<<mechanismPollAxis)) break;
        }
        id=(uint8_t)(5+mechanismPollAxis);
        if(mechanismPreparing || !mechanismFeedback[mechanismPollAxis].targetVerified) param=S_TPOS;
        else if(mechanismFeedback[mechanismPollAxis].reached &&
                !mechanismFeedback[mechanismPollAxis].actualAfterArrival) {
            /* Read position first after arrival; keep checking faults if its reply is lost. */
            mechanismPollKind[mechanismPollAxis]^=1;
            param=mechanismPollKind[mechanismPollAxis]?S_CPOS:S_FLAG;
        }
        else param=S_FLAG;
        if(mechanismAxes) Emm_V5_Read_Sys_Params(id,param);
    }
}
