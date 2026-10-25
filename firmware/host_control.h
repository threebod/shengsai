#ifndef SHENGSAI_HOST_CONTROL_H
#define SHENGSAI_HOST_CONTROL_H
#include "../shared/protocol.h"
#include "scale_store.h"

static SsParser hostParser;
static volatile uint8_t hostRx[512];
static volatile uint16_t hostHead,hostTail;
static volatile uint8_t hostOverflow;
static uint8_t hostConfirmed,hostError,hostLogLength,hostExecuting;
static uint8_t hostLog[128];
static uint8_t hostDriveActive;
static uint8_t hostCancelled;
static uint8_t hostOperationFailed;
static uint8_t hostStartPending,hostStartZone;
static uint16_t hostStartRpm;
static uint32_t hostDriveAt,hostStateAt,hostSession;
static uint16_t scaleForward=VISION_RING2_FORWARD_MILLI,scaleRight=VISION_RING2_RIGHT_MILLI;
static uint8_t scaleSaved;
typedef struct { uint8_t valid; uint32_t at; SsFrame request,response; } HostCached;
static HostCached hostCache[16];static uint8_t hostCacheIndex;

#include "obstacle_turret.h"

static void hostSend(const SsFrame *f) {
    uint8_t bytes[SS_MAX_FRAME];unsigned i,n=ss_encode(f,bytes);
    for(i=0;i<n;++i) { while(USART_GetFlagStatus(UART5,USART_FLAG_TXE)==RESET) {} USART_SendData(UART5,bytes[i]); }
}
static void hostLogChar(char c) {
    SsFrame f;
    if(c=='\r') return;
    if(c=='\n' || hostLogLength==sizeof(hostLog)) {
        if(hostLogLength) {
            if((hostLogLength>=3 && memcmp(hostLog,"ERR",3)==0) ||
               (hostLogLength>=12 && memcmp(hostLog,"VISION ERROR",12)==0) ||
               (hostLogLength>=12 && memcmp(hostLog,"BUTTON ERROR",12)==0)) { hostError=1;hostOperationFailed=1; }
            f.flags=SS_EVENT;f.command=SS_LOG;f.sequence=0;f.length=hostLogLength;
            taskLogAppend(&taskLog,hostLog,hostLogLength);
            memcpy(f.data,hostLog,hostLogLength);hostSend(&f);hostLogLength=0;
        }
    }
    if(c!='\n') hostLog[hostLogLength++]=(uint8_t)c;
}
static uint8_t hostBusy(void) {
    return hostStartPending || obstacleTurretPending || motionMode || routeActive || mechanismState.running || mechanismActionActive ||
           visionSessionActive() || servoChannelMoving[0] || servoChannelMoving[1] || servoChannelMoving[2];
}
static void hostApplyScale(void) {
    visionRingCalibration[1].matrix[0]=-scaleForward/1000.0f;
    visionRingCalibration[1].matrix[3]=-scaleRight/1000.0f;
    visionRingCalibration[1].calibrated=1;
}
static void hostScaleInit(void) { scaleSaved=scaleLoad(&scaleForward,&scaleRight);hostApplyScale(); }
static void hostSnapshot(SsFrame *r) {
    unsigned i;r->length=21;r->data[1]=hostBusy();r->data[2]=hostEStop;
    r->data[3]=mechanismState.valid;r->data[4]=missionPhase;r->data[5]=routeIndex;
    r->data[6]=missionButtonReady;r->data[7]=fullRouteRunning;r->data[8]=hostConfirmed;
    ss_put16(r->data+9,(uint16_t)mechanismState.current.horizontalDmm);
    ss_put16(r->data+11,mechanismState.current.liftDmm);
    for(i=0;i<3;++i) ss_put16(r->data+13+i*2,(uint16_t)(currentAngleMdeg[i]/100));
    r->data[19]=mechanismState.running;r->data[20]=mechanismActionActive;
}
static void hostExecute(const SsFrame *q,SsFrame *r) {
    char command[80];uint16_t a,b;unsigned i;uint8_t movement;
    r->flags=SS_RESPONSE;r->sequence=q->sequence;r->command=q->command;r->length=1;r->data[0]=0;
    movement=q->command>=SS_SERVO && q->command<=SS_ALIGN;
    if(q->command==SS_HELLO) {
        if(q->length!=4) {r->data[0]=1;return;}
        if(hostSession!=ss_get32(q->data)) {
            if(axis6TestActive()) {
                hostCancelled=hostOperationFailed=1;stopServoMotion();stopAllMotors();
            }
            memset(hostCache,0,sizeof(hostCache));hostConfirmed=0;hostSession=ss_get32(q->data);
        }
        r->length=4;r->data[1]=1;r->data[2]=SS_ROUTE_REVISION;r->data[3]=1;return;
    }
    if(q->command==SS_STATUS && !q->length) {hostSnapshot(r);return;}
    if(q->command==SS_GET_LOG) {taskLogRead(&taskLog,q,r);return;}
    if(q->command==SS_HEARTBEAT && !q->length) {hostHeartbeatActive=1;hostHeartbeatStamp=clockMs;return;}
    if((q->command==SS_STOP || q->command==SS_ESTOP) && !q->length) {
        hostCancelled=hostOperationFailed=1;
        stopServoMotion();stopAllMotors();hostDriveActive=0;
        if(q->command==SS_ESTOP) {hostEStop=1;hostConfirmed=0;}
        return;
    }
    if(q->command==SS_CLEAR && q->length==1 && q->data[0]==1 && !hostBusy()) {
        MechanismPose zero=rawPickInitial.pose;
        zero.horizontalDmm=0;zero.liftDmm=0;zero.turretDdeg=(uint16_t)(currentAngleMdeg[2]/100);
        mechanismStateInitialize(&mechanismState,&zero);
        hostEStop=0;hostConfirmed=1;return;
    }
    if(hostEStop && movement) {r->data[0]=3;return;}
    if(movement && !hostConfirmed) {r->data[0]=6;return;}
    if(q->command==SS_GET_RINGS && !q->length) {
        r->length=14;r->data[1]=ringPosesSaved;
        for(i=0;i<3;++i) {ss_put16(r->data+2+i*4,(uint16_t)missionRingPoses[i].horizontalDmm);ss_put16(r->data+4+i*4,missionRingPoses[i].turretDdeg);}return;
    }
    if(q->command==SS_GET_SCALE && !q->length) {
        r->length=6;r->data[1]=scaleSaved;ss_put16(r->data+2,scaleForward);ss_put16(r->data+4,scaleRight);return;
    }
    if(hostBusy() && !(q->command==SS_DRIVE && hostDriveActive) &&
       !(q->command==SS_SERVO && !hostStartPending && !obstacleTurretPending &&
         !motionMode && !routeActive && !mechanismState.running &&
         !mechanismActionActive && !visionSessionActive())) {r->data[0]=2;return;}
    hostError=hostCancelled=0;command[0]=0;
    switch(q->command) {
    case SS_SET_RINGS:
        if(q->length!=12) break;
        sprintf(command,"ring poses %d %u %d %u %d %u",(int16_t)ss_get16(q->data),ss_get16(q->data+2),(int16_t)ss_get16(q->data+4),ss_get16(q->data+6),(int16_t)ss_get16(q->data+8),ss_get16(q->data+10));break;
    case SS_APPLY_SCALE: case SS_SAVE_SCALE:
        if(q->length!=4) break;
        a=ss_get16(q->data);b=ss_get16(q->data+2);
        if(!scaleValid(a,b)) break;
        if(q->command==SS_SAVE_SCALE && !scaleSave(a,b)) {r->data[0]=4;return;}
        scaleForward=a;scaleRight=b;hostApplyScale();
        {uint16_t sf,sr;scaleSaved=scaleLoad(&sf,&sr) && sf==a && sr==b;}
        r->length=6;r->data[1]=scaleSaved;ss_put16(r->data+2,a);ss_put16(r->data+4,b);return;
    case SS_SERVO:
        if(q->length!=3 || q->data[0]<2 || q->data[0]>4) break;
        a=ss_get16(q->data+1);if(a>(q->data[0]==4?3600:2700)) break;
        if(q->data[0]==4) invalidateMechanism("turret_manual");
        servoSetTargetMdeg(q->data[0],(uint32_t)a*100,1200);return;
    case SS_AUX:
        if(q->length!=3) break;
        sprintf(command,"auxmove %u %d 30 50",q->data[0],(int16_t)ss_get16(q->data+1));break;
    case SS_DRIVE:
        if(q->length!=1 || (q->data[0]!=1 && q->data[0]!=255)) break;
        if(!hostDriveActive) {invalidateNavigation("manual_drive");setAllMotorsEnabled(true);}
        hostDriveActive=1;hostDriveAt=clockMs;motionMode=8;
        sendRouteSpeeds(q->data[0]==1?30:-30,0,0);return;
    case SS_PREPARE: case SS_MISSION: case SS_ROUTE:
        if(!ss_route_request_valid(q->data,q->length)) break;
        a=ss_get16(q->data+3);
        if(q->command!=SS_ROUTE && !mechanismState.valid) {r->data[0]=6;return;}
        if(!prepareObstacleTurret()) {r->data[0]=4;return;}
        routeObstacleRegions=ss_get16(q->data+1);
        if(q->command==SS_PREPARE) {
            missionButtonRpm=a;missionButtonReady=1;
            return;
        }
        hostStartPending=q->command;hostStartZone=q->data[0];hostStartRpm=a;
        return;
    case SS_ACTION:
        if(q->length!=1) break;
        if(!mechanismState.valid) {r->data[0]=6;return;}
        if(q->data[0]==SS_ACTION_AXIS6_TEST) {
            if(!axis6TestBuild(&mechanismState.current,axis6TestActions)) {r->data[0]=1;return;}
            if(motionInterrupted() || !mechanismActionStart(&rawPickInitial,axis6TestActions,2)) {
                r->data[0]=4;return;
            }
            hostHeartbeatActive=1;hostHeartbeatStamp=clockMs;
            return;
        }
        if(q->data[0]==0) {
            rawPickActionsForSlot(rawPickRouteActions,1);
            if(!mechanismActionStart(&rawPickInitial,rawPickRouteActions,RAW_PICK_ACTION_COUNT)) {r->data[0]=2;}return;
        }
        if(q->data[0]==1 || q->data[0]==2) {
            const uint8_t rings[3]={1,2,3};
            uint16_t count=missionBuildTaskPlace(missionWorkActions,rings,1,q->data[0]==1);
            if(!mechanismActionStart(q->data[0]==1?&missionCoarseInitial:&missionInitial,missionWorkActions,count)) r->data[0]=2;
            return;
        }
        break;
    case SS_SAMPLE:
        if(q->length!=2 || !ss_get16(q->data)) break;
        sprintf(command,"vision sample ring 2 %u",ss_get16(q->data));break;
    case SS_VISION_JOG:
        if(q->length!=4) break;
        sprintf(command,"vision jog %d %d 15",(int16_t)ss_get16(q->data),(int16_t)ss_get16(q->data+2));break;
    case SS_ALIGN:
        if(q->length) break;strcpy(command,"vision align ring 2");break;
    default: break;
    }
    if(!command[0]) {r->data[0]=1;return;}
    armed=1;hostExecuting=1;processCommand(command);hostExecuting=0;
    if(hostError || jogCanFault) r->data[0]=4;
}
static void hostService(void) {
    static uint8_t operationActive,operationSequence,operationCommand;
    SsFrame q,r;unsigned i;
    if(hostOverflow) {
        hostOperationFailed=1;
        __disable_irq();hostTail=hostHead;hostOverflow=0;__enable_irq();hostParser.count=0;
        stopServoMotion();stopAllMotors();hostDriveActive=0;
    }
    if(hostDriveActive && (!motionMode || clockMs-hostDriveAt>300)) {
        hostOperationFailed=1;
        stopServoMotion();stopAllMotors();hostDriveActive=0;
    }
    while(hostTail!=hostHead) {
        uint8_t b=hostRx[hostTail];hostTail=(uint16_t)((hostTail+1)%sizeof(hostRx));
        if(!ss_feed(&hostParser,b,&q) || q.flags!=SS_REQUEST) continue;
        for(i=0;i<16;++i) if(q.command!=SS_STOP && q.command!=SS_ESTOP && hostCache[i].valid &&
            clockMs-hostCache[i].at<=10000 && hostCache[i].request.sequence==q.sequence &&
            hostCache[i].request.command==q.command && hostCache[i].request.length==q.length &&
            !memcmp(hostCache[i].request.data,q.data,q.length)) break;
        if(i<16) {hostSend(&hostCache[i].response);continue;}
        hostExecute(&q,&r);if(r.data[0]) r.flags|=SS_ERROR;
        if(q.command!=SS_HEARTBEAT && q.command!=SS_STATUS && q.command!=SS_STOP && q.command!=SS_ESTOP) {
            hostCache[hostCacheIndex].request=q;hostCache[hostCacheIndex].response=r;
            hostCache[hostCacheIndex].valid=1;hostCache[hostCacheIndex].at=clockMs;
            hostCacheIndex=(uint8_t)((hostCacheIndex+1)%16);
        }
        hostSend(&r);
        if(!r.data[0] && q.command>=SS_SERVO && q.command<=SS_ALIGN && hostBusy()) {
            hostOperationFailed=0;operationActive=1;operationSequence=q.sequence;operationCommand=q.command;
        }
    }
    serviceObstacleHostStart();
    if(clockMs-hostStateAt>=200) {
        hostStateAt=clockMs;r.flags=SS_EVENT;r.sequence=0;r.command=SS_STATE;r.data[0]=0;hostSnapshot(&r);hostSend(&r);
    }
    if(operationActive && !hostBusy()) {
        r.flags=SS_EVENT;r.sequence=operationSequence;r.command=operationCommand;r.length=1;
        r.data[0]=(hostEStop||hostOperationFailed||hostCancelled||jogCanFault)?4:0;hostSend(&r);operationActive=0;
    }
}
#endif
