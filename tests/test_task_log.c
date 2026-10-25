#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include "../firmware/task_log.h"

static TaskLog logBuffer={1U};
static SsFrame readLog(uint32_t task,uint32_t offset,uint32_t end) {
    SsFrame q={0},r={0};q.command=SS_GET_LOG;q.length=12U;
    ss_put32(q.data,task);ss_put32(q.data+4,offset);ss_put32(q.data+8,end);
    taskLogRead(&logBuffer,&q,&r);return r;
}
int main(void) {
    SsFrame r,q={0};uint32_t task,next,end;unsigned i,total;
    uint8_t line[128],collected[TASK_LOG_CAPACITY];
    r=readLog(0,0,0);assert(r.length==14 && !r.data[0] && ss_get32(r.data+5)==0);
    /* Recording has no connection/session dependency. Read a captured end in pages. */
    memset(line,'a',sizeof(line));taskLogAppend(&logBuffer,line,sizeof(line));
    taskLogAppend(&logBuffer,(const uint8_t *)"ERR offline",11U);
    r=readLog(0,0,0);task=ss_get32(r.data+1);next=ss_get32(r.data+5);end=ss_get32(r.data+9);
    assert(r.length==SS_MAX_PAYLOAD && !r.data[0] && next==114U && end==141U);
    memcpy(collected,r.data+14,114U);
    taskLogAppend(&logBuffer,(const uint8_t *)"new live line",13U);
    r=readLog(task,next,end);
    assert(!r.data[0] && ss_get32(r.data+5)==end && ss_get32(r.data+9)==end);
    memcpy(collected+114,r.data+14,r.length-14);
    assert(!memcmp(collected,line,128U) && !memcmp(collected+128,"\nERR offline\n",13U));
    /* Re-reading does not consume logs, and new appends do not extend a snapshot. */
    r=readLog(task,next,end);assert(!r.data[0] && ss_get32(r.data+9)==141U);
    q.length=11U;taskLogRead(&logBuffer,&q,&r);assert(r.data[0]==1);
    r=readLog(task,0,logBuffer.end+1);assert(r.data[0]==5);
    r=readLog(task,2,1);assert(r.data[0]==5);
    /* Retain the newest complete lines, advertise truncation, reject overwritten cursors. */
    for(i=0;i<400U;++i) taskLogAppend(&logBuffer,line,sizeof(line));
    r=readLog(task,next,end);assert(r.data[0]==5);
    r=readLog(0,0,0);assert(!r.data[0] && r.data[13]==1);
    total=0;end=ss_get32(r.data+9);
    do {
        unsigned n=r.length-14U;
        memcpy(collected+total,r.data+14,n);total+=n;
        next=ss_get32(r.data+5);
        assert(total<=TASK_LOG_CAPACITY && ss_get32(r.data+9)==end);
        if(next==end) break;
        r=readLog(task,next,end);assert(!r.data[0]);
    } while(1);
    assert(total>32000U && total%129U==0U);
    for(i=0;i<total;++i) assert(collected[i]==(i%129U==128U?'\n':'a'));
    taskLogReset(&logBuffer);
    r=readLog(task,next,end);assert(r.data[0]==5);
    r=readLog(0,0,0);assert(!r.data[0] && !r.data[13] && ss_get32(r.data+9)==0);
    assert(ss_get32(r.data+1)!=task);
    puts("task log ok");return 0;
}
