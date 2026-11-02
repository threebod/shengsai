#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include "../firmware/vision_control.h"

#define VISION_MOVE_STATUS_POLL_MS 20U
enum { S_FLAG };
static VisionSession visionSession;
static uint8_t motionMode,emergencyStop,fault,visionMotionAutomatic;
static uint8_t visionMotionMotorMask,visionMotionQueryId,visionMotionSeenMask,visionMotionStatus[4];
static uint32_t clockMs,motionStart,motionDuration,lastControl;
static uint8_t statusReady,statusValue[4];
static unsigned stops,failures,queries,reports;
static const char *failureReason;
static uint8_t motionInterrupted(void){return fault;}
static void stopDriveMotors(void){++stops;}
static void failVision(const char *reason){++failures;failureReason=reason;stopDriveMotors();motionMode=0;visionSession.state=VISION_STATE_FAILED;}
static void printVisionMotionStatus(void){++reports;}
static void serialSendString(const char *s){(void)s;}
static uint8_t jogCanTakeDriveStatus(uint8_t id,uint8_t *status){
    uint8_t bit=(uint8_t)(1U<<(id-1));
    if(!(statusReady&bit)) return 0;
    statusReady&=(uint8_t)~bit;*status=statusValue[id-1];return 1;
}
static void jogCanExpectDriveStatus(uint8_t id){assert(id>=1 && id<=4);}
static void Emm_V5_Read_Sys_Params(uint8_t id,int param){assert(id>=1 && id<=4 && param==S_FLAG);++queries;}
static uint8_t trackingActive;
static unsigned trackingServices;
static uint8_t visionTrackingActive(void){return trackingActive;}
static void serviceVisionTrackingMotion(void){++trackingServices;}
#include "../firmware/vision_motion_runtime.h"

static void reset(uint8_t mode,uint8_t automatic){
    vision_session_init(&visionSession);visionSession.mode=mode;visionSession.state=VISION_STATE_MOVE;
    motionMode=4;emergencyStop=fault=0;visionMotionAutomatic=automatic;
    motionStart=clockMs=100;motionDuration=1200;lastControl=clockMs;
    visionMotionMotorMask=15;visionMotionQueryId=visionMotionSeenMask=statusReady=0;
    stops=failures=queries=reports=0;failureReason=NULL;
}
int main(void){
    uint8_t mode,automatic,status;
    trackingActive=1;serviceMotion();assert(trackingServices==1);trackingActive=0;
    /* Reproduce the log: m1=1, m2/m3/m4=3 must not prevent timed completion. */
    reset(VISION_MODE_RING,1);
    statusReady=15;statusValue[0]=1;statusValue[1]=statusValue[2]=statusValue[3]=3;
    clockMs=motionStart+motionDuration;serviceMotion();
    assert(stops==1 && !failures && motionMode==0 && visionSession.state==VISION_STATE_SETTLE);
    for(mode=VISION_MODE_MATERIAL;mode<=VISION_MODE_RING;++mode) for(automatic=0;automatic<=1;++automatic){
        reset(mode,automatic);
        clockMs=motionStart+motionDuration-1;serviceMotion();assert(!stops && !failures && motionMode==4);
        clockMs++;serviceMotion();assert(stops==1 && !failures && motionMode==0);
        assert(visionSession.state==(automatic?VISION_STATE_SETTLE:VISION_STATE_PAUSED));
        serviceMotion();assert(stops==1);
    }
    reset(VISION_MODE_RING,1);statusReady=15;
    for(status=0;status<4;++status) statusValue[status]=3;
    clockMs=motionStart+100;serviceMotion();assert(!stops && !failures && motionMode==4);
    for(status=0;status<=13;++status){
        if(status==1 || status==3) continue;
        reset(VISION_MODE_RING,1);statusReady=1;statusValue[0]=status;
        serviceMotion();assert(failures==1 && reports==1 && stops==1);
        assert(strcmp(failureReason,"MOTOR_STATUS_FAULT")==0);
    }
    reset(VISION_MODE_RING,1);fault=1;serviceMotion();assert(failures==1 && stops==1);
    assert(strcmp(failureReason,"MOTION_FAULT")==0);
    reset(VISION_MODE_RING,1);emergencyStop=1;clockMs+=motionDuration;
    serviceMotion();assert(!stops && !failures && visionSession.state==VISION_STATE_MOVE);
    puts("PASS vision timed completion without arrival replies, no early completion, fault and emergency gates");return 0;
}
