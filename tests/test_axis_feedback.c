#include <assert.h>
#include <stdio.h>
#include "../firmware/axis_feedback.h"
int main(void){
    AxisFeedback a,b;
    axisBegin(&a,65536,3200);assert(a.expected==131072 && !a.reached);
    assert(axisStatus(&a,3)==0);assert(!axisTarget(&a,65536));assert(!a.reached);
    assert(axisTarget(&a,131072));assert(axisStatus(&a,1)==0);assert(axisStatus(&a,3)==1);
    axisBegin(&a,0,-1);assert(a.expected==-20);assert(!a.targetVerified);assert(axisTarget(&a,-21));assert(axisStatus(&a,3)==1);
    axisBegin(&a,0,1);assert(!axisTarget(&a,0));assert(axisStatus(&a,3)==0);
    assert(axisTarget(&a,20));assert(axisStatus(&a,7)==-1);assert(axisStatus(&a,2)==-1);
    axisBegin(&a,0,100);axisBegin(&b,0,200);axisTarget(&a,a.expected);axisTarget(&b,b.expected);
    assert(axisStatus(&a,3)==1);assert(!b.reached);assert(axisStatus(&b,1)==0);assert(axisStatus(&b,3)==1);
    puts("PASS stale arrival rejection, target verification, both directions, short moves, disabled/stall faults and independent axes");return 0;
}
