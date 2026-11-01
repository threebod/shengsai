#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include "../firmware/placement_test.h"
static PresetRoute route;
int main(void)
{
    PlacementHeadingGate gate={0};
    unsigned i,a,b,events,valid=0;
    assert(placementTestBuild(&route,0));
    assert(route.legs[0].from.x==2250 && route.legs[0].from.y==150);
    assert(route.legs[0].drive.rightMm==-150 && route.legs[1].drive.upMm==250);
    assert(route.legs[2].drive.rightMm==-900 && route.legs[2].to.heading==-1);
    for(a=0;a<15;a++) for(b=a;b<15;b++) {
        uint16_t regions=(uint16_t)((1U<<a)|(1U<<b));
        if(!placementTestBuild(&route,regions)) { assert(!route.count);continue; }
        ++valid;events=0;
        for(i=0;i<route.count;i++) {
            assert(route.legs[i].taskIndex==6 || route.legs[i].taskIndex==8);
            assert(presetSegmentClear(route.legs[i].from,route.legs[i].to,regions));
            if(route.legs[i].arc) assert(presetArcClear(&route.legs[i],&route.legs[i+1],regions));
            if(route.legs[i].drive.event) ++events;
        }
        assert(events==2 && route.legs[route.count-1].taskIndex==8);
    }
    assert(valid && !placementTestBuild(&route,0x8000));
    assert(placementHeadingStep(&gate,179,-179,1000,1000,1)==0);
    assert(placementHeadingStep(&gate,179,-179,1020,1020,1)==0 && gate.rpm<0);
    assert(gate.corrected);
    assert(placementHeadingStep(&gate,179,179.2f,1040,1040,1)==0 && gate.rpm==0);
    assert(placementHeadingStep(&gate,179,179.2f,1040,1140,1)==0);
    assert(placementHeadingStep(&gate,179,179.2f,1040,1240,1)==0); /* Old sample cannot prove stability. */
    assert(placementHeadingStep(&gate,179,179.2f,1260,1260,1)==1);
    memset(&gate,0,sizeof(gate));
    placementHeadingStep(&gate,0,0,1000,1000,1);
    placementHeadingStep(&gate,0,0,1020,1020,1);
    placementHeadingStep(&gate,0,1,1100,1100,1); /* Leaving band restarts settling. */
    assert(placementHeadingStep(&gate,0,0,1220,1220,1)==0);
    assert(placementHeadingStep(&gate,0,0,1320,1320,1)==0);
    assert(placementHeadingStep(&gate,0,0,1420,1420,1)==1);
    assert(placementHeadingStep(&gate,0,0,1000,1500,1)==-1);
    puts("PASS zone2 coarse/temp-only plan, collision checks, shortest-angle gate and fresh stable IMU");
    return 0;
}
