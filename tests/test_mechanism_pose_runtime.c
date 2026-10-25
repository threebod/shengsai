#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include "../firmware/mechanism_action.h"
#include "../firmware/axis_feedback.h"

#define SERVO_EMPTY_TURRET_SPEED_DPS10 2500U
static MechanismState mechanismState;
static AxisFeedback mechanismFeedback[2];
static uint8_t mechanismAxes,mechanismBaselineMask,mechanismEnabledAxes,mechanismPreparing;
static uint8_t mechanismPollAxis,mechanismPollKind[2],mechanismPositionPriority;
static uint8_t servoChannelEnabled[3],servoChannelMoving[3],gripperMayHoldMaterial;
static uint32_t clockMs,mechanismPollAt,mechanismFeedbackTimeout,currentAngleMdeg[3],targetAngleMdeg[3];
static int32_t mechanismBaseline[2],mechanismSignedPulses[2];
static uint8_t jogCanFault,testAxis6,interrupted,failEnable;
static unsigned enables[2],positions[2],syncs,servos,clears,stops,delays;
static uint8_t syncId;
static uint8_t axis6TestActive(void){return testAxis6;}
static uint8_t motionInterrupted(void){return interrupted || jogCanFault;}
static void stopServoMotion(void){memset(servoChannelMoving,0,sizeof(servoChannelMoving));}
static void stopAllMotors(void){++stops;mechanismEnabledAxes=0;mechanismStateInvalidate(&mechanismState);}
static void delay_ms(unsigned ms){delays+=ms;}
static void Emm_V5_En_Control(uint8_t id,bool enable,bool sync)
{
    assert(id>=5 && id<=6 && enable && !sync);++enables[id-5];
    if(failEnable) jogCanFault=1;
}
static void Emm_V5_Pos_Control(uint8_t id,uint8_t dir,uint16_t rpm,uint8_t acc,
                               uint32_t pulses,bool absolute,bool sync)
{
    assert(id>=5 && id<=6 && dir<=1 && rpm && acc && pulses && !absolute && sync);
    ++positions[id-5];
}
static void Emm_V5_Synchronous_motion(uint8_t id){++syncs;syncId=id;}
static void jogCanClearReply(uint8_t id,uint8_t fn){(void)id;(void)fn;++clears;}
static void servoSetTargetMdeg(uint8_t channel,uint32_t angle,uint16_t speed)
{
    assert(channel==4 && speed);++servos;targetAngleMdeg[2]=angle;
    servoChannelEnabled[2]=servoChannelMoving[2]=1;
}
static void serialSendString(const char *text){(void)text;}
#include "../firmware/mechanism_pose_runtime.h"

static MechanismPose origin={0,0,2600,1000,200,2000,240,2200};
static void reset(void)
{
    mechanismStateReset(&mechanismState);mechanismStateInitialize(&mechanismState,&origin);
    mechanismEnabledAxes=mechanismAxes=mechanismBaselineMask=mechanismPreparing=0;
    mechanismPositionPriority=0;clockMs=500;
    memset(enables,0,sizeof(enables));memset(positions,0,sizeof(positions));
    memset(servoChannelEnabled,0,sizeof(servoChannelEnabled));memset(servoChannelMoving,0,sizeof(servoChannelMoving));
    servoChannelEnabled[2]=1;currentAngleMdeg[2]=targetAngleMdeg[2]=260000;
    jogCanFault=testAxis6=interrupted=failEnable=0;syncs=servos=clears=stops=delays=0;
}
int main(void)
{
    MechanismPose target;
    reset();startMechanismPose(&origin);
    assert(!mechanismState.running && !enables[0] && !enables[1] && !clears && !servos && !syncs && !delays);
    /* Pose validity, PWM startup and unfinished servo motion cannot be skipped. */
    reset();mechanismState.valid=0;startMechanismPose(&origin);assert(!enables[0] && !clears && !mechanismState.valid);
    reset();mechanismState.running=1;startMechanismPose(&origin);assert(mechanismState.running && !enables[0] && !clears);
    reset();target=origin;target.liftDmm=1501;startMechanismPose(&target);assert(!enables[0] && !clears);
    reset();interrupted=1;startMechanismPose(&origin);assert(stops==1 && !mechanismState.valid);
    reset();servoChannelEnabled[2]=0;startMechanismPose(&origin);assert(mechanismState.running);
    dispatchMechanismPose();assert(servos==1 && servoChannelEnabled[2]);
    reset();servoChannelMoving[2]=1;startMechanismPose(&origin);assert(mechanismState.running);
    reset();targetAngleMdeg[2]=53000;startMechanismPose(&origin);assert(mechanismState.running);
    reset();currentAngleMdeg[2]=53000;startMechanismPose(&origin);assert(mechanismState.running);
    /* Consecutive actual poses enable once; a stop clears the cache. */
    reset();target=origin;target.horizontalDmm=50;startMechanismPose(&target);
    assert(enables[0]==1 && enables[1]==1 && mechanismEnabledAxes==3 && mechanismPreparing);
    dispatchMechanismPose();assert(positions[1]==1 && !positions[0] && syncs==1);
    mechanismState.current=target;mechanismState.running=0;
    startMechanismPose(&origin);assert(enables[0]==1 && enables[1]==1);
    stopAllMotors();assert(!mechanismEnabledAxes);
    mechanismStateInitialize(&mechanismState,&target);startMechanismPose(&origin);
    assert(enables[0]==2 && enables[1]==2);
    /* A combined release approach queues both axes for one synchronous start. */
    reset();target=origin;target.horizontalDmm=50;target.liftDmm=1270;
    startMechanismPose(&target);
    assert(mechanismAxes==3 && mechanismPreparing && !positions[0] && !positions[1] && !syncs);
    dispatchMechanismPose();
    assert(positions[0]==1 && positions[1]==1 && syncs==1 && syncId==0);
    assert(mechanismState.running && mechanismState.current.horizontalDmm==0 && mechanismState.current.liftDmm==0);
    /* A failed enable cannot be cached or followed by motion commands. */
    reset();failEnable=1;target=origin;target.horizontalDmm=50;startMechanismPose(&target);
    assert(stops==1 && !mechanismState.valid && !mechanismEnabledAxes && !positions[0] && !positions[1] && !syncs);
    /* The diagnostic round trip must never enable axis 5 or servo PWM. */
    reset();testAxis6=1;servoChannelEnabled[2]=0;startMechanismPose(&target);dispatchMechanismPose();
    assert(!enables[0] && enables[1]==1 && !servos && !positions[0] && positions[1]==1 && syncId==6);
    mechanismState.current=target;mechanismState.running=0;
    startMechanismPose(&origin);dispatchMechanismPose();
    assert(!enables[0] && enables[1]==1 && positions[1]==2 && !servos && syncId==6);
    puts("PASS pose dispatch: no-op gate, PWM startup, enable caching/reset, fault and axis6 isolation");
    return 0;
}
