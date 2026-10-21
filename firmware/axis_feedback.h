#ifndef SHENGSAI_AXIS_FEEDBACK_H
#define SHENGSAI_AXIS_FEEDBACK_H
#include <stdint.h>
#include <stdlib.h>
/* Emm relative mode 0 accumulates from the previous input target, not a timer estimate. */
typedef struct { int32_t expected, actual; uint8_t targetVerified, reached, actualAfterArrival; } AxisFeedback;
static void axisBegin(AxisFeedback *a, int32_t baseline, int32_t signedPulses) {
    int32_t delta=(int32_t)(((int64_t)signedPulses*65536)/3200);
    a->expected=baseline+delta;a->actual=baseline;a->targetVerified=a->reached=a->actualAfterArrival=0;
}
static int axisTarget(AxisFeedback *a, int32_t reported) {
    /* At most one encoder count rounding from the pulse/encoder conversion. */
    a->targetVerified=llabs((int64_t)reported-a->expected)<=1;
    return a->targetVerified;
}
static int axisStatus(AxisFeedback *a, uint8_t status) {
    if(!(status&1) || (status&12)) return -1;
    if(a->targetVerified && (status&2)) a->reached=1;
    return a->reached;
}
#endif
