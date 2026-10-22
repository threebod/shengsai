#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include "../firmware/mechanism_action.h"
#include "../firmware/axis_feedback.h"
typedef enum { S_FLAG,S_TPOS,S_CPOS } SysParams_t;
static MechanismState mechanismState;
static AxisFeedback mechanismFeedback[2];
static uint8_t mechanismAxes,mechanismPreparing,mechanismPollAxis,mechanismPollKind[2],mechanismBaselineMask;
static uint8_t servoChannelMoving[3],fault;
static uint32_t clockMs,mechanismFeedbackTimeout,mechanismPollAt;
static int32_t mechanismBaseline[2];
static int32_t replies[2][3];static uint8_t ready[2][3];
static unsigned stops,dones,queries;
static uint8_t simulateReplies,missingMotor,missingFn;
static unsigned queryKinds[2][3];
static unsigned prematurePositions;
static uint32_t arrivalAt[2];
static uint32_t faultAt[2];
static uint32_t lastQueryAt;
static int slot(uint8_t fn){return fn==0x33?0:fn==0x3a?1:2;}
static uint8_t jogCanTakeReply(uint8_t id,uint8_t fn,int32_t *v){int s=slot(fn);if(!ready[id-5][s])return 0;*v=replies[id-5][s];ready[id-5][s]=0;return 1;}
static void jogCanClearReply(uint8_t id,uint8_t fn){ready[id-5][slot(fn)]=0;}
static void reply(int id,int fn,int32_t v){replies[id-5][slot((uint8_t)fn)]=v;ready[id-5][slot((uint8_t)fn)]=1;}
static uint8_t motionInterrupted(void){return fault;}
static void stopServoMotion(void){servoChannelMoving[2]=0;}
static void stopAllMotors(void){++stops;mechanismStateInvalidate(&mechanismState);}
static void dispatchMechanismPose(void){mechanismPreparing=0;}
static void serialSendString(const char *s){(void)s;}
static void serialSendUint(uint16_t v){(void)v;}
static void serialSendUint32(uint32_t v){(void)v;}
static void Emm_V5_Read_Sys_Params(uint8_t id,SysParams_t p){
    uint8_t fn=p==S_TPOS?0x33:p==S_FLAG?0x3a:0x36;
    assert(!queries || clockMs-lastQueryAt>=MECHANISM_FEEDBACK_POLL_MS);
    lastQueryAt=clockMs;
    ++queries;++queryKinds[id-5][slot(fn)];
    if(simulateReplies && p==S_CPOS && clockMs<arrivalAt[id-5]) ++prematurePositions;
    if(simulateReplies && !(id==missingMotor && (!missingFn || fn==missingFn)))
        reply(id,fn,p==S_FLAG?(faultAt[id-5] && clockMs>=faultAt[id-5]?7:
              clockMs>=arrivalAt[id-5]?3:1):mechanismFeedback[id-5].expected);
}
static void printMechanismPose(const char *s,const MechanismPose *p){(void)s;(void)p;++dones;}
#include "../firmware/mechanism_runtime.h"
static void reset(void){
    memset(&mechanismState,0,sizeof(mechanismState));memset(ready,0,sizeof(ready));
    mechanismState.valid=mechanismState.running=1;mechanismState.deadlineMs=100;mechanismState.target.horizontalDmm=10;
    mechanismFeedbackTimeout=3000;clockMs=500;mechanismAxes=3;mechanismPreparing=0;mechanismPollAt=clockMs;
    stops=dones=queries=0;fault=servoChannelMoving[2]=0;
    simulateReplies=missingMotor=missingFn=0;mechanismPollAxis=0;prematurePositions=0;
    mechanismPollKind[0]=mechanismPollKind[1]=0;mechanismBaselineMask=0;
    memset(queryKinds,0,sizeof(queryKinds));arrivalAt[0]=arrivalAt[1]=0;
    faultAt[0]=faultAt[1]=0;
    axisBegin(&mechanismFeedback[0],0,3200);axisBegin(&mechanismFeedback[1],0,1600);
}
int main(void){
    uint8_t axes,phase;
    uint32_t latency,maximumLatency=0;
    reset();serviceMechanism();assert(mechanismState.running && !stops && !dones);
    reply(5,0x3a,3);reply(6,0x3a,3);serviceMechanism();assert(!dones);
    reply(5,0x33,0);reply(6,0x33,0);serviceMechanism();assert(!dones);
    reply(5,0x33,65536);reply(6,0x33,32768);reply(5,0x3a,3);serviceMechanism();assert(!dones);
    reply(5,0x3a,3);reply(5,0x36,65536);serviceMechanism();assert(!dones);
    reply(5,0x36,65536);reply(6,0x3a,3);serviceMechanism();assert(!dones);
    servoChannelMoving[2]=1;reply(6,0x36,32768);serviceMechanism();assert(!dones);
    servoChannelMoving[2]=0;serviceMechanism();assert(dones==1 && !stops && !mechanismState.running);
    reset();clockMs=3000;serviceMechanism();assert(stops==1 && !mechanismState.valid && !dones);
    reset();reply(5,0x3a,7);serviceMechanism();assert(stops==1 && !mechanismState.valid);
    reset();fault=1;serviceMechanism();assert(stops==1 && !mechanismState.valid);
    reset();mechanismAxes=0;servoChannelMoving[2]=1;serviceMechanism();assert(!dones);
    servoChannelMoving[2]=0;serviceMechanism();assert(dones==1 && !stops);
    /* Reply only to queries actually sent by the production scheduler. */
    reset();simulateReplies=1;arrivalAt[0]=1400;arrivalAt[1]=800;
    while(mechanismState.running && clockMs<3000){clockMs+=50;serviceMechanism();}
    assert(dones==1 && !stops && mechanismState.valid && clockMs>=1400);
    assert(queryKinds[0][1] && queryKinds[0][2] && queryKinds[1][1] && queryKinds[1][2]);
    reset();simulateReplies=1;arrivalAt[0]=800;arrivalAt[1]=1400;
    while(mechanismState.running && clockMs<3000){clockMs+=50;serviceMechanism();}
    assert(dones==1 && !stops && clockMs>=1400);
    for(axes=1;axes<=2;++axes){
        reset();simulateReplies=1;mechanismAxes=axes;
        while(mechanismState.running && clockMs<3000){clockMs+=50;serviceMechanism();}
        assert(dones==1 && !stops && mechanismState.valid);
    }
    reset();simulateReplies=1;missingMotor=6;
    while(mechanismState.running && clockMs<3000){clockMs+=50;serviceMechanism();}
    assert(stops==1 && !dones && !mechanismState.valid);
    reset();simulateReplies=1;arrivalAt[0]=arrivalAt[1]=1000;
    while(mechanismState.running && clockMs<3000){++clockMs;serviceMechanism();}
    latency=clockMs-1000;
    printf("Both-axis completion delay after simulated arrival: %lu ms; premature position queries: %u\n",
           (unsigned long)latency,prematurePositions);
    if(dones!=1 || stops || latency>82 || prematurePositions){
        fprintf(stderr,"feedback polling added excessive inter-action delay\n");return 1;
    }
    for(phase=0;phase<40;++phase){
        reset();simulateReplies=1;arrivalAt[0]=arrivalAt[1]=1000+phase;
        while(mechanismState.running && clockMs<3000){++clockMs;serviceMechanism();}
        latency=clockMs-arrivalAt[0];
        assert(dones==1 && !stops && !prematurePositions && latency<=82);
        if(latency>maximumLatency) maximumLatency=latency;
    }
    printf("Maximum dual-axis delay across arrival phases: %lu ms\n",(unsigned long)maximumLatency);
    for(axes=1;axes<=2;++axes){
        reset();simulateReplies=1;mechanismAxes=axes;arrivalAt[0]=arrivalAt[1]=1000;
        while(mechanismState.running && clockMs<3000){++clockMs;serviceMechanism();}
        assert(dones==1 && !stops && clockMs-1000<=42 && !prematurePositions);
    }
    reset();simulateReplies=1;missingMotor=6;missingFn=0x36;
    while(mechanismState.running && clockMs<3000){++clockMs;serviceMechanism();}
    assert(stops==1 && !dones && !mechanismState.valid && mechanismFeedback[1].reached &&
           !mechanismFeedback[1].actualAfterArrival);
    reset();simulateReplies=1;missingMotor=6;missingFn=0x36;faultAt[1]=1000;
    while(mechanismState.running && clockMs<3000){++clockMs;serviceMechanism();}
    if(stops!=1 || dones || mechanismState.valid || clockMs>=1200){
        fprintf(stderr,"lost position feedback hid a subsequent drive fault\n");return 1;
    }
    reset();simulateReplies=1;missingMotor=5;missingFn=0x3a;
    while(mechanismState.running && clockMs<3000){++clockMs;serviceMechanism();}
    assert(stops==1 && !dones && !mechanismState.valid && !mechanismFeedback[0].reached);
    puts("PASS production service: independent single/dual-axis polling, both arrival orders, stale feedback and servo gate, timeout/fault invalidate");return 0;
}
