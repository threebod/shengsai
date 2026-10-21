#include <assert.h>
#include <stdio.h>
#include "../firmware/mechanism_action.h"
#include "../firmware/axis_feedback.h"
typedef enum { S_FLAG,S_TPOS,S_CPOS } SysParams_t;
static MechanismState mechanismState;
static AxisFeedback mechanismFeedback[2];
static uint8_t mechanismAxes,mechanismPreparing,mechanismPollAxis,mechanismPollKind,mechanismBaselineMask;
static uint8_t servoChannelMoving[3],fault;
static uint32_t clockMs,mechanismFeedbackTimeout,mechanismPollAt;
static int32_t mechanismBaseline[2];
static int32_t replies[2][3];static uint8_t ready[2][3];
static unsigned stops,dones,queries;
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
static void Emm_V5_Read_Sys_Params(uint8_t id,SysParams_t p){(void)id;(void)p;++queries;}
static void printMechanismPose(const char *s,const MechanismPose *p){(void)s;(void)p;++dones;}
#include "../firmware/mechanism_runtime.h"
static void reset(void){
    memset(&mechanismState,0,sizeof(mechanismState));memset(ready,0,sizeof(ready));
    mechanismState.valid=mechanismState.running=1;mechanismState.deadlineMs=100;mechanismState.target.horizontalDmm=10;
    mechanismFeedbackTimeout=3000;clockMs=500;mechanismAxes=3;mechanismPreparing=0;mechanismPollAt=clockMs;
    stops=dones=queries=0;fault=servoChannelMoving[2]=0;
    axisBegin(&mechanismFeedback[0],0,3200);axisBegin(&mechanismFeedback[1],0,1600);
}
int main(void){
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
    puts("PASS production service: old deadline does not stop, stale target/status ignored, both axes and servo gate, timeout/fault invalidate");return 0;
}
