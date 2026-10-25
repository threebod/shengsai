#ifndef SHENGSAI_TASK_LOG_H
#define SHENGSAI_TASK_LOG_H
#include "../shared/protocol.h"

#define TASK_LOG_CAPACITY 32768U
#define TASK_LOG_HEADER_SIZE 14U
typedef struct {
    uint32_t task, first, end;
    uint8_t bytes[TASK_LOG_CAPACITY];
} TaskLog;

static void taskLogReset(TaskLog *log) {
    if (++log->task==0U) log->task=1U;
    log->first=log->end=0U;
}
static void taskLogAppend(TaskLog *log,const uint8_t *line,unsigned length) {
    unsigned i;
    /* Drop complete lines so a full buffer never starts halfway through a line. */
    while(log->end-log->first+length+1U>TASK_LOG_CAPACITY) {
        while(log->first<log->end && log->bytes[log->first++%TASK_LOG_CAPACITY]!='\n') {}
    }
    for(i=0;i<length;++i) log->bytes[log->end++%TASK_LOG_CAPACITY]=line[i];
    log->bytes[log->end++%TASK_LOG_CAPACITY]='\n';
}
static void taskLogRead(const TaskLog *log,const SsFrame *q,SsFrame *r) {
    uint32_t task,offset,end;unsigned i,n;
    r->length=1U;r->data[0]=1U;
    if(q->length!=12U) return;
    task=ss_get32(q->data);offset=ss_get32(q->data+4);end=ss_get32(q->data+8);
    if(!task && !offset && !end) {offset=log->first;end=log->end;}
    else if(task!=log->task || offset<log->first || end>log->end || offset>end) {
        r->data[0]=5U;return;
    }
    n=(unsigned)(end-offset);
    if(n>SS_MAX_PAYLOAD-TASK_LOG_HEADER_SIZE) n=SS_MAX_PAYLOAD-TASK_LOG_HEADER_SIZE;
    r->data[0]=0U;ss_put32(r->data+1,log->task);ss_put32(r->data+5,offset+n);
    ss_put32(r->data+9,end);r->data[13]=(uint8_t)(log->first!=0U);
    for(i=0;i<n;++i) r->data[TASK_LOG_HEADER_SIZE+i]=log->bytes[(offset+i)%TASK_LOG_CAPACITY];
    r->length=(uint16_t)(TASK_LOG_HEADER_SIZE+n);
}
#endif
