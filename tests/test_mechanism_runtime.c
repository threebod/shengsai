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
static uint8_t mechanismAxes,mechanismPreparing,mechanismPollAxis,mechanismPollKind[2],mechanismBaselineMask,mechanismPositionPriority;
static uint8_t servoChannelMoving[3],fault;
static uint32_t clockMs,mechanismFeedbackTimeout,mechanismPollAt;
static int32_t mechanismBaseline[2];
static int32_t replies[2][3];static uint8_t ready[2][3];
static unsigned stops,queries;
static uint8_t simulateReplies,missingMotor,missingFn;
static unsigned queryKinds[2][3];
static unsigned prematurePositions;
static uint32_t arrivalAt[2];
static uint32_t faultAt[2];
static int32_t positionOffset[2],positionStep[2];
static uint32_t lastQueryAt;
static char logText[4096];
static int slot(uint8_t fn){return fn==0x33?0:fn==0x3a?1:2;}
static uint8_t jogCanTakeReply(uint8_t id,uint8_t fn,int32_t *v){int s=slot(fn);if(!ready[id-5][s])return 0;*v=replies[id-5][s];ready[id-5][s]=0;return 1;}
static void jogCanClearReply(uint8_t id,uint8_t fn){ready[id-5][slot(fn)]=0;}
static void reply(int id,int fn,int32_t v){replies[id-5][slot((uint8_t)fn)]=v;ready[id-5][slot((uint8_t)fn)]=1;}
static uint8_t motionInterrupted(void){return fault;}
static void stopServoMotion(void){servoChannelMoving[2]=0;}
static void stopAllMotors(void){++stops;mechanismStateInvalidate(&mechanismState);}
static void dispatchMechanismPose(void){mechanismPreparing=0;}
static void serialSendString(const char *s){assert(strlen(logText)+strlen(s)<sizeof(logText));strcat(logText,s);}
static void serialSendUint32(uint32_t v){char text[12];snprintf(text,sizeof(text),"%lu",(unsigned long)v);serialSendString(text);}
static void serialSendUint(uint16_t v){serialSendUint32(v);}
static void Emm_V5_Read_Sys_Params(uint8_t id,SysParams_t p){
    uint8_t fn=p==S_TPOS?0x33:p==S_FLAG?0x3a:0x36;
    assert(!queries || clockMs-lastQueryAt>=MECHANISM_FEEDBACK_POLL_MS);
    lastQueryAt=clockMs;
    ++queries;++queryKinds[id-5][slot(fn)];
    if(simulateReplies && p==S_CPOS && clockMs<arrivalAt[id-5]) ++prematurePositions;
    if(simulateReplies && !(id==missingMotor && (!missingFn || fn==missingFn)))
        reply(id,fn,p==S_FLAG?(faultAt[id-5] && clockMs>=faultAt[id-5]?7:
              clockMs>=arrivalAt[id-5]?3:1):mechanismFeedback[id-5].expected+
              (p==S_CPOS?positionOffset[id-5]+positionStep[id-5]*(int32_t)queryKinds[id-5][2]:0));
}
#include "../firmware/mechanism_runtime.h"
static uint8_t completed(void){return !mechanismState.running && mechanismState.valid;}
static void reset(void){
    memset(&mechanismState,0,sizeof(mechanismState));memset(ready,0,sizeof(ready));
    mechanismState.valid=mechanismState.running=1;mechanismState.deadlineMs=100;mechanismState.target.horizontalDmm=10;
    mechanismFeedbackTimeout=3000;clockMs=500;mechanismAxes=3;mechanismPreparing=0;mechanismPollAt=clockMs;
    stops=queries=0;fault=servoChannelMoving[2]=0;
    simulateReplies=missingMotor=missingFn=0;mechanismPollAxis=0;prematurePositions=0;
    logText[0]='\0';
    mechanismPollKind[0]=mechanismPollKind[1]=0;mechanismBaselineMask=mechanismPositionPriority=0;
    memset(queryKinds,0,sizeof(queryKinds));arrivalAt[0]=arrivalAt[1]=0;
    faultAt[0]=faultAt[1]=0;
    positionOffset[0]=positionOffset[1]=positionStep[0]=positionStep[1]=0;
    axisBegin(&mechanismFeedback[0],0,3200);axisBegin(&mechanismFeedback[1],0,1600);
}
int main(void){
    uint8_t axes,phase;
    uint32_t latency,maximumLatency=0;
    MechanismInitialState initial = {0};
    /* A repeated open command can overlap tray rotation only when its PWM
     * command is already complete; the angle is configured, not hard-coded. */
    initial.gripperOpenDeg=200U;
    assert(mechanismGripperOpenCommandComplete(&initial,1U,0U,200000U,200000U));
    assert(!mechanismGripperOpenCommandComplete(&initial,0U,0U,200000U,200000U));
    assert(!mechanismGripperOpenCommandComplete(&initial,1U,1U,200000U,200000U));
    assert(!mechanismGripperOpenCommandComplete(&initial,1U,0U,155000U,200000U));
    assert(!mechanismGripperOpenCommandComplete(&initial,1U,0U,200000U,155000U));
    initial.gripperOpenDeg=185U;
    assert(mechanismGripperOpenCommandComplete(&initial,1U,0U,185000U,185000U));
    assert(!mechanismGripperOpenCommandComplete(&initial,1U,0U,200000U,200000U));
    reset();serviceMechanism();assert(mechanismState.running && !stops && !completed());
    reply(5,0x3a,3);reply(6,0x3a,3);serviceMechanism();assert(!completed());
    reply(5,0x33,0);reply(6,0x33,0);serviceMechanism();assert(!completed());
    reply(5,0x33,65536);reply(6,0x33,32768);reply(5,0x3a,3);serviceMechanism();assert(!completed());
    reply(5,0x3a,3);reply(5,0x36,65536);serviceMechanism();assert(!completed());
    reply(5,0x36,65536);reply(6,0x3a,3);serviceMechanism();assert(!completed());
    servoChannelMoving[2]=1;reply(6,0x36,32768);serviceMechanism();assert(!completed());
    servoChannelMoving[2]=0;serviceMechanism();assert(completed()==1 && !stops && !mechanismState.running);
    /* Losing arrival while the other axis/servo is pending invalidates the old position. */
    reset();mechanismAxes=2;servoChannelMoving[2]=1;
    reply(6,0x33,32768);serviceMechanism();
    reply(6,0x3a,3);serviceMechanism();reply(6,0x36,32768);serviceMechanism();
    assert(mechanismFeedback[1].actualAfterArrival && !completed());
    reply(6,0x36,32952);reply(6,0x3a,1);serviceMechanism();
    assert(!mechanismFeedback[1].reached && !mechanismFeedback[1].actualAfterArrival);
    servoChannelMoving[2]=0;serviceMechanism();assert(!completed() && mechanismState.running);
    reply(6,0x36,32768);reply(6,0x3a,3);serviceMechanism();assert(!completed());
    reply(6,0x36,32768);serviceMechanism();assert(completed()==1 && !stops);
    reset();clockMs=3000;serviceMechanism();assert(stops==1 && !mechanismState.valid && !completed());
    reset();reply(5,0x3a,7);serviceMechanism();assert(stops==1 && !mechanismState.valid);
    reset();fault=1;serviceMechanism();assert(stops==1 && !mechanismState.valid);
    reset();mechanismAxes=0;servoChannelMoving[2]=1;serviceMechanism();assert(!completed());
    servoChannelMoving[2]=0;serviceMechanism();assert(completed()==1 && !stops);
    /* Arrival on the just-polled axis gets the next slot, without increasing
     * bus frequency or starving the other axis on a missing position reply. */
    reset();mechanismPollAxis=1;
    axisTarget(&mechanismFeedback[0],65536);axisTarget(&mechanismFeedback[1],32768);
    reply(6,0x3a,3);clockMs=520;serviceMechanism();
    assert(queryKinds[1][2]==1 && queries==1 && !completed());
    clockMs=540;serviceMechanism();assert(queryKinds[0][1]==1 && queries==2);
    clockMs=560;serviceMechanism();assert(queryKinds[1][1]==1 && queries==3);
    reset();axisTarget(&mechanismFeedback[0],65536);axisTarget(&mechanismFeedback[1],32768);
    reply(5,0x3a,3);reply(6,0x3a,3);clockMs=520;serviceMechanism();
    clockMs=540;serviceMechanism();
    assert(queryKinds[0][2]==1 && queryKinds[1][2]==1 && queries==2 && !completed());
    /* A fresh arrival after a diagnostic position query must still query
     * position first, rather than inheriting that axis's old query phase. */
    reset();mechanismPollKind[1]=1;mechanismPollAxis=1;
    axisTarget(&mechanismFeedback[1],32768);reply(6,0x3a,3);
    clockMs=520;serviceMechanism();assert(queryKinds[1][2]==1);
    /* Reply only to queries actually sent by the production scheduler. */
    reset();simulateReplies=1;arrivalAt[0]=1400;arrivalAt[1]=800;
    while(mechanismState.running && clockMs<3000){clockMs+=50;serviceMechanism();}
    assert(completed()==1 && !stops && mechanismState.valid && clockMs>=1400);
    assert(queryKinds[0][1] && queryKinds[0][2] && queryKinds[1][1] && queryKinds[1][2]);
    reset();simulateReplies=1;arrivalAt[0]=800;arrivalAt[1]=1400;
    while(mechanismState.running && clockMs<3000){clockMs+=50;serviceMechanism();}
    assert(completed()==1 && !stops && clockMs>=1400);
    for(axes=1;axes<=2;++axes){
        reset();simulateReplies=1;mechanismAxes=axes;
        while(mechanismState.running && clockMs<3000){clockMs+=50;serviceMechanism();}
        assert(completed()==1 && !stops && mechanismState.valid);
    }
    reset();simulateReplies=1;missingMotor=6;
    while(mechanismState.running && clockMs<3000){clockMs+=50;serviceMechanism();}
    assert(stops==1 && !completed() && !mechanismState.valid);
    reset();simulateReplies=1;arrivalAt[0]=arrivalAt[1]=1000;
    while(mechanismState.running && clockMs<3000){++clockMs;serviceMechanism();}
    latency=clockMs-1000;
    printf("Both-axis completion delay after simulated arrival: %lu ms; premature position queries: %u\n",
           (unsigned long)latency,prematurePositions);
    if(completed()!=1 || stops || latency>82 || prematurePositions){
        fprintf(stderr,"feedback polling added excessive inter-action delay\n");return 1;
    }
    for(phase=0;phase<40;++phase){
        reset();simulateReplies=1;arrivalAt[0]=arrivalAt[1]=1000+phase;
        while(mechanismState.running && clockMs<3000){++clockMs;serviceMechanism();}
        latency=clockMs-arrivalAt[0];
        assert(completed()==1 && !stops && !prematurePositions && latency<=82);
        if(latency>maximumLatency) maximumLatency=latency;
    }
    printf("Maximum dual-axis delay across arrival phases: %lu ms\n",(unsigned long)maximumLatency);
    for(axes=1;axes<=2;++axes){
        reset();simulateReplies=1;mechanismAxes=axes;arrivalAt[0]=arrivalAt[1]=1000;
        while(mechanismState.running && clockMs<3000){++clockMs;serviceMechanism();}
        assert(completed()==1 && !stops && clockMs-1000<=42 && !prematurePositions);
    }
    reset();simulateReplies=1;missingMotor=6;missingFn=0x36;
    while(mechanismState.running && clockMs<3000){++clockMs;serviceMechanism();}
    assert(stops==1 && !completed() && !mechanismState.valid && mechanismFeedback[1].reached &&
           !mechanismFeedback[1].actualAfterArrival);
    reset();simulateReplies=1;missingMotor=6;missingFn=0x36;faultAt[1]=1000;
    while(mechanismState.running && clockMs<3000){++clockMs;serviceMechanism();}
    if(stops!=1 || completed() || mechanismState.valid || clockMs>=1200){
        fprintf(stderr,"lost position feedback hid a subsequent drive fault\n");return 1;
    }
    reset();simulateReplies=1;missingMotor=5;missingFn=0x3a;
    while(mechanismState.running && clockMs<3000){++clockMs;serviceMechanism();}
    assert(stops==1 && !completed() && !mechanismState.valid && !mechanismFeedback[0].reached);
    reset();simulateReplies=1;arrivalAt[1]=4000;
    while(mechanismState.running && clockMs<3000){++clockMs;serviceMechanism();}
    assert(stops==1 && !completed() && !mechanismFeedback[1].reached &&
           !mechanismFeedback[1].actualAfterArrival && mechanismFeedback[1].actualReplies>1);
    assert(mechanismFeedback[1].lastStatus==1 && mechanismFeedback[1].statusReplies>0);
    assert(strstr(logText,"AXIS FEEDBACK motor=6 status_rx=") && strstr(logText,"actual_enc=32768"));
    assert(mechanismFeedback[1].sampleCount==4 && strstr(logText,"AXIS SAMPLE motor=6 elapsed_ms="));
    for(phase=0;phase<4;++phase) {
        assert(mechanismFeedback[1].samples[phase].elapsedMs>=2500 &&
               mechanismFeedback[1].samples[phase].elapsedMs<3000);
        if(phase) assert(mechanismFeedback[1].samples[phase-1].elapsedMs<mechanismFeedback[1].samples[phase].elapsedMs);
    }
    /* Reproduce the observed 184-count residual without allowing a position-only completion. */
    reset();simulateReplies=1;arrivalAt[1]=4000;mechanismFeedback[1].expected=28622;positionOffset[1]=184;
    while(mechanismState.running && clockMs<3000){++clockMs;serviceMechanism();}
    assert(stops==1 && !completed() && !mechanismState.valid && !mechanismFeedback[1].reached);
    assert(strstr(logText,"expected_enc=28622 actual_enc=28806") && mechanismFeedback[1].sampleCount==4);
    for(phase=0;phase<4;++phase) assert(mechanismFeedback[1].samples[phase].position==28806);
    /* Keep the most recent four changing samples in chronological order. */
    reset();simulateReplies=1;arrivalAt[1]=4000;positionStep[1]=-20;
    while(mechanismState.running && clockMs<3000){++clockMs;serviceMechanism();}
    assert(mechanismFeedback[1].sampleCount==4 && mechanismFeedback[1].actualReplies>4);
    for(phase=0;phase<4;++phase)
        assert(mechanismFeedback[1].samples[phase].position==32768-20*(int32_t)(mechanismFeedback[1].actualReplies-3+phase));
    reset();simulateReplies=1;arrivalAt[1]=4000;faultAt[1]=2800;
    while(mechanismState.running && clockMs<3000){++clockMs;serviceMechanism();}
    assert(stops==1 && !completed() && !mechanismState.valid && clockMs<2900);
    reset();simulateReplies=1;missingMotor=6;missingFn=0x3a;
    while(mechanismState.running && clockMs<3000){++clockMs;serviceMechanism();}
    assert(stops==1 && !completed() && mechanismFeedback[1].statusReplies==0 &&
           mechanismFeedback[1].actualReplies>1 && !mechanismFeedback[1].reached);
    assert(strstr(logText,"AXIS FEEDBACK motor=6 status_rx=0 status=0 position_rx="));
    reset();clockMs=3000;serviceMechanism();assert(strstr(logText,"actual_enc=unknown"));
    reset();serialSendInt32(-1);serialSendString(" ");serialSendInt32(INT32_MIN);
    serialSendString(" ");serialSendInt32(INT32_MAX);serialSendString(" ");serialSendInt32(0);
    assert(strcmp(logText,"-1 -2147483648 2147483647 0")==0);
    puts("PASS production service: independent single/dual-axis polling, both arrival orders, stale feedback and servo gate, timeout/fault invalidate");return 0;
}
