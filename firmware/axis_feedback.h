#ifndef SHENGSAI_AXIS_FEEDBACK_H
#define SHENGSAI_AXIS_FEEDBACK_H
#include <stdint.h>
#include <stdlib.h>
#define MECHANISM_FEEDBACK_POLL_MS 10U
#define MECHANISM_FEEDBACK_DIAG_MS 500U
#define MECHANISM_FEEDBACK_SAMPLES 4U
/* Emm relative mode 0 accumulates from the previous input target, not a timer estimate. */
typedef struct {
    int32_t expected, actual;
    uint8_t targetVerified, reached, actualAfterArrival, lastStatus;
    uint16_t statusReplies, actualReplies;
    uint8_t sampleCount;
    struct { int32_t position; uint32_t elapsedMs; } samples[MECHANISM_FEEDBACK_SAMPLES];
} AxisFeedback;
static void axisBegin(AxisFeedback *a, int32_t baseline, int32_t signedPulses) {
    int32_t delta=(int32_t)(((int64_t)signedPulses*65536)/3200);
    a->expected=baseline+delta;a->actual=baseline;a->targetVerified=a->reached=a->actualAfterArrival=0;
    a->lastStatus=0;a->statusReplies=a->actualReplies=0;a->sampleCount=0;
}
static int axisTarget(AxisFeedback *a, int32_t reported) {
    /* At most one encoder count rounding from the pulse/encoder conversion. */
    a->targetVerified=llabs((int64_t)reported-a->expected)<=1;
    if(!a->targetVerified) a->reached=a->actualAfterArrival=0;
    return a->targetVerified;
}
static int axisStatus(AxisFeedback *a, uint8_t status) {
    a->lastStatus=status;++a->statusReplies;
    a->reached=a->targetVerified && (status&1) && (status&2) && !(status&12);
    if(!a->reached) a->actualAfterArrival=0;
    if(!(status&1) || (status&12)) return -1;
    return a->reached;
}
#endif
