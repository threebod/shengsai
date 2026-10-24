#ifndef SHENGSAI_HOST_LOG_H
#define SHENGSAI_HOST_LOG_H

static uint8_t hostLogLength,hostLogContinued,hostLogKeep;
static uint8_t hostLog[128];

static uint8_t hostLogStartsWith(const char *prefix)
{
    unsigned n=(unsigned)strlen(prefix);
    return hostLogLength>=n && memcmp(hostLog,prefix,n)==0;
}

static void hostLogChar(char c)
{
    SsFrame f;
    if(c=='\r') return;
    if(c=='\n' || hostLogLength==sizeof(hostLog)) {
        if(hostLogLength) {
            if(!hostLogContinued) {
                uint8_t error=hostLogStartsWith("ERR") ||
                    hostLogStartsWith("VISION ERROR") || hostLogStartsWith("BUTTON ERROR");
                if(error) { hostError=1;hostOperationFailed=1; }
                /* Calibration still consumes MEASURE/PAUSED as functional data. */
                hostLogKeep=error || hostLogStartsWith("AXIS TIMEOUT ") ||
                    hostLogStartsWith("AXIS FEEDBACK ") || hostLogStartsWith("AXIS SAMPLE ") ||
                    hostLogStartsWith("MECH INVALID ") || hostLogStartsWith("NAV INVALID ") ||
                    hostLogStartsWith("ROUTE INVALID ") || hostLogStartsWith("QR INVALID ") ||
                    hostLogStartsWith("EMERGENCY STOP") || hostLogStartsWith("OBSTACLE STOP") ||
                    hostLogStartsWith("WAIT:") || hostLogStartsWith("BUTTON IGNORED ") ||
                    hostLogStartsWith("HMI OBSTACLES rejected:") ||
                    hostLogStartsWith("ROUTE RAWPICK state=WAIT_MATERIAL") ||
                    hostLogStartsWith("ROUTE RAWPICK state=WAIT_ACTIONS") ||
                    hostLogStartsWith("VISION MOVE STATUS ") ||
                    hostLogStartsWith("VISION CORRECTION LIMIT ") ||
                    hostLogStartsWith("VISION MEASURE ") || hostLogStartsWith("VISION PAUSED");
            }
            if(hostLogKeep) {
                f.flags=SS_EVENT;f.command=SS_LOG;f.sequence=0;f.length=hostLogLength;
                memcpy(f.data,hostLog,hostLogLength);hostSend(&f);
            }
            hostLogLength=0;hostLogContinued=1;
        }
        if(c=='\n') hostLogContinued=hostLogKeep=0;
    }
    if(c!='\n') hostLog[hostLogLength++]=(uint8_t)c;
}

#endif
