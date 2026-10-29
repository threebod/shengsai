#ifndef SHENGSAI_HOST_SERVICE_H
#define SHENGSAI_HOST_SERVICE_H

#define HOST_RX_SERVICE_BYTES 64U
#define HOST_RX_SERVICE_FRAMES 2U
static SsFrame hostPendingReply;
static uint8_t hostReplyPending;
static uint8_t hostOperationActive,hostOperationSequence,hostOperationCommand;

static uint8_t hostCompleteOperation(uint8_t active,uint8_t sequence,uint8_t command)
{
    SsFrame frame;
    if(!active || hostBusy() || hostReplyPending) return active;
    frame.flags=SS_EVENT;frame.sequence=sequence;frame.command=command;frame.length=1;
    frame.data[0]=(hostEStop||hostOperationFailed||hostCancelled||jogCanFault)?4:0;
    return hostSend(&frame)?0U:1U;
}

static void hostService(void)
{
    SsFrame q,r;unsigned i,bytes=0U,frames=0U;
    if(hostOverflow) {
        hostOperationFailed=1;
        __disable_irq();hostTail=hostHead;hostOverflow=0;__enable_irq();hostParser.count=0;
        stopServoMotion();stopAllMotors();hostDriveActive=0;
        serialSendString("ERR HOST: RX queue overflow; stopped\r\n");
    }
    if(hostDriveActive && (!motionMode || clockMs-hostDriveAt>300)) {
        hostOperationFailed=1;
        stopServoMotion();stopAllMotors();hostDriveActive=0;
    }
    if(hostReplyPending && hostSend(&hostPendingReply)) hostReplyPending=0U;
    hostOperationActive=hostCompleteOperation(hostOperationActive,hostOperationSequence,hostOperationCommand);
    while(hostTail!=hostHead && bytes<HOST_RX_SERVICE_BYTES && frames<HOST_RX_SERVICE_FRAMES &&
          !hostReplyPending && !(hostOperationActive && !hostBusy()) && hostTxFree(&hostTx)>=SS_MAX_FRAME) {
        uint8_t b=hostRx[hostTail];hostTail=(uint16_t)((hostTail+1)%sizeof(hostRx));++bytes;
        if(!ss_feed(&hostParser,b,&q) || q.flags!=SS_REQUEST) continue;
        ++frames;
        for(i=0;i<16;++i) if(q.command!=SS_STOP && q.command!=SS_ESTOP && hostCache[i].valid &&
            clockMs-hostCache[i].at<=10000 && hostCache[i].request.sequence==q.sequence &&
            hostCache[i].request.command==q.command && hostCache[i].request.length==q.length &&
            !memcmp(hostCache[i].request.data,q.data,q.length)) break;
        if(i<16) {
            if(!hostSend(&hostCache[i].response)) {hostPendingReply=hostCache[i].response;hostReplyPending=1U;}
            continue;
        }
        hostExecute(&q,&r);if(r.data[0]) r.flags|=SS_ERROR;
        if(q.command!=SS_HEARTBEAT && q.command!=SS_STATUS && q.command!=SS_STOP && q.command!=SS_ESTOP) {
            hostCache[hostCacheIndex].request=q;hostCache[hostCacheIndex].response=r;
            hostCache[hostCacheIndex].valid=1;hostCache[hostCacheIndex].at=clockMs;
            hostCacheIndex=(uint8_t)((hostCacheIndex+1)%16);
        }
        if(!hostSend(&r)) {hostPendingReply=r;hostReplyPending=1U;}
        if(!r.data[0] && q.command>=SS_SERVO && q.command<=SS_ALIGN && hostBusy()) {
            hostOperationFailed=0;hostOperationActive=1;hostOperationSequence=q.sequence;hostOperationCommand=q.command;
        }
    }
    serviceObstacleHostStart();
    hostOperationActive=hostCompleteOperation(hostOperationActive,hostOperationSequence,hostOperationCommand);
    if(clockMs-hostStateAt>=200) {
        r.flags=SS_EVENT;r.sequence=0;r.command=SS_STATE;r.data[0]=0;hostSnapshot(&r);
        if(hostSendOptional(&r)) hostStateAt=clockMs;
    }
}
#endif
