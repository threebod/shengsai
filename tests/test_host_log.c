#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../shared/protocol.h"

static uint8_t hostError,hostOperationFailed;
static unsigned frames;
static char output[1024];
static void hostSend(const SsFrame *f)
{
    assert(f->flags==SS_EVENT && f->command==SS_LOG && f->sequence==0);
    assert(strlen(output)+f->length<sizeof(output));
    strncat(output,(const char *)f->data,f->length);++frames;
}
#include "../firmware/host_log.h"

static void logLine(const char *s)
{
    while(*s) hostLogChar(*s++);
}
static void reset(void)
{
    frames=0;output[0]=0;hostError=hostOperationFailed=0;
    hostLogLength=hostLogContinued=hostLogKeep=0;
}
int main(void)
{
    unsigned i;
    const char *kept[]={"ERR MECH: feedback timeout", "VISION ERROR reason=CAMERA_TIMEOUT",
        "BUTTON ERROR prepare task first", "AXIS TIMEOUT motor=6", "AXIS FEEDBACK motor=6",
        "AXIS SAMPLE motor=6", "MECH INVALID reason=stopped", "NAV INVALID reason=can_fault",
        "ROUTE INVALID reason=mission_vision", "QR INVALID expected=CCC+PPP+CCC+PPP",
        "EMERGENCY STOP; motors 1..6 stopped", "OBSTACLE STOP; black blob detected",
        "WAIT: verify position", "HMI OBSTACLES rejected: busy", "BUTTON IGNORED busy",
        "ROUTE RAWPICK state=WAIT_MATERIAL color=1", "ROUTE RAWPICK state=WAIT_ACTIONS color=1",
        "VISION MOVE STATUS elapsed_ms=3000",
        "VISION CORRECTION LIMIT iterations=2", "VISION MEASURE id=3 ring=2 token=5 du=1 dv=2 quality=90",
        "VISION PAUSED"};
    reset();
    logLine("MECH RUN h=0 l=0 t=2600\r\nMECH DONE h=0 l=0 t=2600\n");
    logLine("AXIS DONE motor=6\nRAW RELEASE pwm_deg=200\nDONE servo=4\n");
    logLine("ROUTE MISSION phase=RAW_1\nPID TRACE target_cdeg=0\nAXIS6 TEST DONE\n");
    assert(!frames && !hostError && !hostOperationFailed);
    for(i=0;i<sizeof(kept)/sizeof(kept[0]);++i) {
        reset();logLine(kept[i]);logLine("\r\n");
        assert(frames==1 && strcmp(output,kept[i])==0);
        assert(hostError==(i<3) && hostOperationFailed==(i<3));
    }
    /* Long normal lines must not leak chunks that resemble an error. */
    reset();for(i=0;i<128;++i) hostLogChar('x');
    logLine("ERR hidden continuation\n");assert(!frames && !hostError);
    /* Long errors retain every chunk and still mark the operation failed. */
    reset();logLine("ERR ");for(i=0;i<260;++i) hostLogChar('x');logLine("\n");
    assert(frames==3 && strlen(output)==264 && hostError && hostOperationFailed);
    reset();logLine("VISION MEASURE id=");logLine("4 ring=2 token=6 du=0 dv=1 quality=90\r\n");
    assert(frames==1 && strstr(output,"id=4 ring=2"));
    puts("PASS log filtering: blockers/errors only, calibration data, fragmented and long lines");
    return 0;
}
