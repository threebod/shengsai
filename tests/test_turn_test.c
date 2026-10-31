#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include "../firmware/turn_test_log.h"

#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
static uint8_t imuValid,fault,failEnable,missionButtonReady,missionOffline;
static uint8_t presetRouteActive,missionActive,fullRouteRunning,arcRoutePhase;
static uint8_t missionArcActive,missionArcExit,rawPickRouteActive,presetExecutionStep;
static uint8_t routeWaiting,routeTurnInBand,routeStationAlignment,routeSentValid,routeIndex;
static uint8_t routeOnlyTurn,routeRotating,routeActive,turnTestActive,hostHeartbeatActive,navRunning;
static int8_t routeHeading,routeNextHeading,yawSign;
static int16_t routeForward,routeRight;
static uint16_t routeTurnRpmLimit;
static uint16_t motorTrim[5]={1000,1000,1000,1000,1000};
static float imuYaw,routeBaseYaw,routeYaw,routeX,routeY,routeDriveRpm,routeTurnRpm,routeCorrection;
static uint32_t clockMs,imuStamp,routeTick,routeLegStart,routeTurnStart,hostHeartbeatStamp;
static RouteHeadingPid routeHeadingPid;static RouteTurnPid routeTurnPid;
static TurnTestLog turnTestLog;static TaskLog taskLog;
static unsigned enables,stops,invalidations;
static uint8_t motionInterrupted(void){return fault;}
static void invalidateNavigation(const char *s){assert(!strcmp(s,"turn_test"));++invalidations;navRunning=0;}
static void setAllMotorsEnabled(bool enabled){assert(enabled);++enables;if(failEnable)fault=1;}
static void finishTurnTest(uint8_t aborted);
static void stopAllMotors(void){++stops;routeActive=0;finishTurnTest(1);}
#include "../firmware/turn_test.h"

static void reset(void){
    clockMs=imuStamp=1000;imuValid=1;imuYaw=179.0f;yawSign=1;
    fault=failEnable=routeActive=turnTestActive=hostHeartbeatActive=0;
    enables=stops=invalidations=0;missionButtonReady=missionOffline=navRunning=1;
    taskLogReset(&taskLog);taskLogAppend(&taskLog,(const uint8_t *)"OLD",3);
}
static void contains(const char *s){
    char text[TASK_LOG_CAPACITY+1];assert(taskLog.first==0 && taskLog.end<TASK_LOG_CAPACITY);
    memcpy(text,taskLog.bytes,taskLog.end);text[taskLog.end]=0;assert(strstr(text,s));assert(!strstr(text,"OLD"));
}
/* Synthetic 100ms ARX yaw dynamics are only a transport/replay fixture.
 * Controller calls and emitted logs use the production C implementations. */
static void fixture(int direction,const char *directory,unsigned index){
    float actual=179.0f,increment=0,area=0,command=0,raw,error;
    uint32_t stable=0;int16_t wheels[4];char path[512];FILE *file;unsigned i;
    reset();assert(startTurnTest((int8_t)direction));
    for(i=0;i<800 && turnTestActive;++i){
        area+=command*0.02f;clockMs+=20;
        if(clockMs-imuStamp>=100){
            increment=0.4f*increment+(index>4?0.11f:0.07f)*(area/0.1f);actual+=increment;area=0;
            imuYaw=headingError(actual,0);imuStamp=clockMs;
        }
        error=headingError(routeYaw,imuYaw);
        if(!routeTurnInBand && fabsf(error)<=0.5f)routeTurnInBand=1;
        if(routeTurnInBand && fabsf(error)>0.5f)routeTurnInBand=0;
        raw=routeTurnPidStep(&routeTurnPid,error,imuStamp,routeTurnRpmLimit,yawSign);
        routeTurnRpm=routeTurnRamp(routeTurnRpm,routeTurnInBand?0:raw,20);
        command=(float)routeRound(routeTurnRpm);
        wheels[0]=wheels[3]=(int16_t)command;wheels[1]=wheels[2]=(int16_t)-command;
        turnTestLogRecord(&taskLog,&turnTestLog,clockMs,imuStamp,routeYaw,imuYaw,
                          raw,routeTurnRpm,&routeTurnPid,wheels,routeTurnInBand);
        if(routeTurnInBand && !command){
            if(routeTurnInBand==1){stable=clockMs;routeTurnInBand=2;}
            if(clockMs-stable>=200 && (int32_t)(imuStamp-stable)>=200)finishTurnTest(0);
        }
    }
    assert(!turnTestActive && !taskLog.first);contains("TURN_END result=complete");
    if(directory){
        snprintf(path,sizeof(path),"%s/run%u.txt",directory,index);file=fopen(path,"wb");assert(file);
        assert(fwrite(taskLog.bytes,1,taskLog.end,file)==taskLog.end);fclose(file);
    }
}
int main(int argc,char **argv){
    int direction,sign;uint32_t task,end;int16_t wheels[4]={10,-10,-10,10};RouteTurnPid pid={0};
    for(direction=-1;direction<=1;direction+=2)for(sign=-1;sign<=1;sign+=2){
        reset();yawSign=(int8_t)sign;task=taskLog.task;assert(startTurnTest((int8_t)direction));
        assert(enables==1 && invalidations==1 && !navRunning && taskLog.task==task+1);
        assert(routeActive && routeOnlyTurn && routeRotating && turnTestActive && !routeWaiting);
        assert(routeYaw==179+90*direction*sign && routeNextHeading==direction && routeTurnRpmLimit==300);
        assert(!missionActive && !presetRouteActive && !missionOffline && !fullRouteRunning && !missionButtonReady);
        assert(!routeForward && !routeRight && !routeTurnRpm && hostHeartbeatActive);
        contains("TURN_CFG kp_m=2000 ki_m=500 kd_far_m=800 kd_near_m=400");
        turnTestLogRecord(&taskLog,&turnTestLog,1020,1000,routeYaw,imuYaw,180,10,&pid,wheels,0);
        end=taskLog.end;
        turnTestLogRecord(&taskLog,&turnTestLog,1040,1000,routeYaw,imuYaw,180,10,&pid,wheels,0);
        assert(taskLog.end==end);
        wheels[0]++;turnTestLogRecord(&taskLog,&turnTestLog,1060,1000,routeYaw,imuYaw,180,10,&pid,wheels,0);
        assert(taskLog.end>end);contains("TC 1060,");
        end=taskLog.end;turnTestLogRecord(&taskLog,&turnTestLog,1100,1100,routeYaw,imuYaw,180,10,&pid,wheels,0);
        assert(taskLog.end>end);contains("TY 1100,1100,");
        imuYaw=headingError(routeYaw,0);finishTurnTest(0);contains("TURN_END result=complete");
        end=taskLog.end;finishTurnTest(1);assert(taskLog.end==end);
    }
    reset();assert(startTurnTest(1));stopAllMotors();assert(!turnTestActive);contains("TURN_END result=aborted");
    reset();task=taskLog.task;assert(!startTurnTest(0) && !enables && taskLog.task==task);
    reset();imuValid=0;assert(!startTurnTest(1) && !enables);
    reset();imuStamp=749;assert(!startTurnTest(1) && !enables);
    reset();fault=1;assert(!startTurnTest(1) && !enables);
    reset();failEnable=1;task=taskLog.task;assert(!startTurnTest(1) && stops==1 && taskLog.task==task);
    for(direction=1;direction<=6;++direction)fixture(direction%2?1:-1,argc>1?argv[1]:NULL,(unsigned)direction);
    puts("PASS turn test: signed 90deg, production PID, wrap, timestamped command changes, fresh samples, completion and faults");
    return 0;
}
