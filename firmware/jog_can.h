#ifndef JOG_CAN_H
#define JOG_CAN_H

#include <stdint.h>

typedef struct {
    uint32_t extId;
    uint8_t dlc;
    uint8_t data[8];
} JogCanReply;

/* 0: interrupted, 1: reply received, 2: timed out. */
uint8_t jogCanReadStatus(uint8_t motorId, const volatile uint8_t *stopRequested,
                         JogCanReply *reply);

/* Nonblocking status mailboxes for drive motors 1..4, armed after each move. */
void jogCanResetDriveStatus(void);
void jogCanExpectDriveStatus(uint8_t motorId);
uint8_t jogCanTakeDriveStatus(uint8_t motorId, uint8_t *status);

void jogCanClearReply(uint8_t id, uint8_t fn);
uint8_t jogCanTakeReply(uint8_t id, uint8_t fn, int32_t *value);
#endif
