#ifndef SHENGSAI_ROUTE_CANDIDATES_H
#define SHENGSAI_ROUTE_CANDIDATES_H
#include <stdint.h>

/* Absolute nominal endpoints, calibrated signed commands in mm, travel heading.
 * Each group is ordered by priority. Workstation events are bound by the caller,
 * so both rounds and both start zones use the same station-to-station tables.
 * Detours reuse nominal corridors and existing station calibration; physical
 * validation is still required. */
typedef struct {
    int16_t x, y, upMm, rightMm;
    int8_t heading;
} PresetStep;
typedef struct {
    uint8_t count;
    PresetStep steps[6];
} PresetCandidate;

/* Centre; original four-point left route; right outside. */
static const PresetCandidate presetRawToCoarse[3] = {
    {1, {{1200,400,-1680,0,2}}},
    {3, {{400,2080,0,-830,1}, {400,400,-1740,0,2}, {1200,400,0,910,-1}}},
    {4, {{2100,2080,0,900,-1}, {2100,300,-1780,0,2},
         {1200,300,0,-900,1}, {1200,400,100,0,1}}}
};
/* Left then up (14/13/10); up then left (11/8/7); upper outside (2/1/4). */
static const PresetCandidate presetCoarseToTemp[3] = {
    /* Four-point version: left 611, original arc, up 601. No extra moves. */
    {2, {{400,400,0,-860,-1}, {400,1200,850,0,2}}},
    {2, {{1200,1200,800,0,2}, {400,1200,0,-800,2}}},
    {5, {{1200,2080,1680,0,0}, {300,2080,0,-900,1}, {300,1300,-780,0,2},
         {400,1300,0,100,2}, {400,1200,-106,0,2}}}
};
/* User-requested 7/13 experiment: join x=400 at the upper bend, no final sidestep. */
static const PresetCandidate presetCoarseToTemp713 =
    {3, {{1200,2080,1680,0,0}, {400,2080,0,-860,1}, {400,1200,-886,0,2}}};
/* Up then right; right then up; right outside via 6/3. */
static const PresetCandidate presetTempToRaw[3] = {
    /* Four-point version: up 631, original arc, right 601. */
    {2, {{400,2080,880,0,2}, {1200,2080,0,850,1}}},
    {2, {{1200,1200,0,800,2}, {1200,2080,880,0,2}}},
    {6, {{400,1150,-50,0,2}, {1300,1150,0,900,-1}, {1300,1200,50,0,-1},
         {2100,1200,0,800,-1}, {2100,2080,880,0,0}, {1200,2080,0,-900,1}}}
};

/* Start-zone tables are intentionally independent. Keep the initial exit
 * heading during lateral detours: a turn at y=2250/150 cannot fit the body. */
static const PresetCandidate presetStart1ToQr[3] = {
    {2, {{2100,2250,0,-200,0}, {2100,1200,-1080,0,0}}},
    {5, {{2100,2250,0,-200,0}, {2100,2100,-150,0,0},
         {1200,2100,0,-900,0}, {1200,1200,-900,0,0}, {2100,1200,0,900,0}}},
    {5, {{2100,2250,0,-200,0}, {2100,2100,-150,0,0},
         {300,2100,0,-1800,0}, {300,1200,-900,0,0}, {2100,1200,0,1800,0}}}
};
static const PresetCandidate presetStart2ToQr[3] = {
    {2, {{2100,150,0,-150,0}, {2100,1200,1020,0,0}}},
    {5, {{2100,150,0,-150,0}, {2100,300,150,0,0},
         {1200,300,0,-900,0}, {1200,1200,900,0,0}, {2100,1200,0,900,0}}},
    {5, {{2100,150,0,-150,0}, {2100,300,150,0,0},
         {300,300,0,-1800,0}, {300,1200,900,0,0}, {2100,1200,0,1800,0}}}
};

/* 6/3; 9/5; 9/8/7/4/1/2. Keep separate QR->RAW calibration per start zone. */
static const PresetCandidate presetQr1ToRaw[3] = {
    {2, {{2100,2080,860,0,0}, {1200,2080,0,-870,1}}},
    {2, {{1200,1200,0,-900,0}, {1200,2080,880,0,0}}},
    {6, {{2100,1150,-50,0,0}, {400,1150,0,-1700,0}, {400,1300,150,0,2},
         {300,1300,0,-100,2}, {300,2080,780,0,2}, {1200,2080,0,870,1}}}
};
static const PresetCandidate presetQr2ToRaw[3] = {
    {2, {{2100,2080,855,0,0}, {1200,2080,0,-870,1}}},
    {2, {{1200,1200,0,-900,0}, {1200,2080,880,0,0}}},
    {6, {{2100,1150,-50,0,0}, {400,1150,0,-1700,0}, {400,1300,150,0,2},
         {300,1300,0,-100,2}, {300,2080,780,0,2}, {1200,2080,0,870,1}}}
};
#endif
