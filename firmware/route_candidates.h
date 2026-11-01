#ifndef SHENGSAI_ROUTE_CANDIDATES_H
#define SHENGSAI_ROUTE_CANDIDATES_H
#include <stdint.h>

/* Complete immutable motion templates. Headings: 0 up, 1 left, 2 down, -1 right.
 * up/right are signed calibrated commands BEFORE the fixed 249mm arc trims.
 * Each step freezes departure, nominal arrival, command-end heading and arc.
 * Region selection may reject a whole template, never edit its steps. */
typedef struct { int16_t x, y; int8_t heading; } PresetPose;
typedef struct {
    int16_t x, y, upMm, rightMm;
    int8_t departureHeading, arrivalHeading, commandHeading;
    uint8_t arc;
} PresetStep;
typedef struct {
    const char *code;
    const PresetStep *steps;
    uint8_t count;
} PresetCandidate;
typedef struct {
    PresetPose start, goal;
    const uint8_t *templates;
    uint8_t count;
} PresetGroup;

/* S1-1 */
static const PresetStep presetSteps1[] = {
    {2100,2250,0,-200,0,0,0,0},
    {2100,1200,-1080,0,0,0,0,0},
};
/* S1-2 */
static const PresetStep presetSteps2[] = {
    {2250,2000,-250,0,0,0,0,0},
    {1200,2000,0,-1000,0,0,0,0},
    {1200,1200,-800,0,0,0,0,0},
    {2100,1200,0,900,0,0,0,0},
};
/* S1-3 */
static const PresetStep presetSteps3[] = {
    {2250,2100,-150,0,0,0,0,0},
    {300,2100,0,-2000,0,0,0,0},
    {300,1200,-900,0,0,0,0,0},
    {2100,1200,0,1800,0,0,0,0},
};
/* S1-B1 */
static const PresetStep presetSteps4[] = {
    {2250,2100,-150,0,0,0,0,0},
    {2100,2100,0,-150,0,0,0,0},
    {300,2100,0,-1800,1,1,2,1},
    {300,300,-1800,0,2,2,-1,1},
    {2100,300,0,1800,-1,-1,0,1},
    {2100,1200,926,0,0,0,0,0},
};
/* S2-1 */
static const PresetStep presetSteps5[] = {
    {2100,150,0,-150,0,0,0,0},
    {2100,1200,1020,0,0,0,0,0},
};
/* S2-2 */
static const PresetStep presetSteps6[] = {
    {2250,300,150,0,0,0,0,0},
    {1200,300,0,-1050,0,0,0,0},
    {1200,1200,900,0,0,0,0,0},
    {2100,1200,0,900,0,0,0,0},
};
/* S2-3 */
static const PresetStep presetSteps7[] = {
    {2250,300,150,0,0,0,0,0},
    {300,300,0,-1950,0,0,0,0},
    {300,1200,900,0,0,0,0,0},
    {2100,1200,0,1800,0,0,0,0},
};
/* S2-B1 */
static const PresetStep presetSteps8[] = {
    {2250,300,150,0,0,0,0,0},
    {2100,300,0,-150,0,0,0,0},
    {300,300,0,-1800,1,1,0,1},
    {300,2080,1780,0,0,0,-1,1},
    {2100,2080,0,1800,-1,-1,2,1},
    {2100,1250,-830,0,2,2,2,0},
    {2100,1200,-49,0,0,0,0,0},
};
/* Q1-1 */
static const PresetStep presetSteps9[] = {
    {2100,2080,860,0,0,0,1,1},
    {1200,2080,0,-870,1,1,1,0},
};
/* Q-2 */
static const PresetStep presetSteps10[] = {
    {1200,1200,0,-900,0,0,0,0},
    {1200,2080,800,0,0,1,1,0},
};
/* Q-3 */
static const PresetStep presetSteps11[] = {
    {2100,1150,-50,0,0,0,0,0},
    {400,1150,0,-1700,0,0,0,0},
    {400,1300,150,0,2,2,2,0},
    {300,1300,0,-100,2,2,2,0},
    {300,2080,780,0,2,2,1,1},
    {1200,2080,0,870,1,1,1,0},
};
/* Q-B3 */
static const PresetStep presetSteps14[] = {
    {2100,300,-900,0,2,2,1,1},
    {300,300,0,-1800,1,1,0,1},
    {300,2080,1780,0,0,0,-1,1},
    {1200,2080,0,897,-1,1,1,0},
};
/* Q2-1 */
static const PresetStep presetSteps16[] = {
    {2100,2080,855,0,0,0,1,1},
    {1200,2080,0,-900,1,1,1,0},
};
/* RC-1 nominal corridor; presetExecutionLeg supplies the fixed 100/50/1680 departure. */
static const PresetStep presetSteps17[] = {
    {1200,400,-1680,0,2,-1,-1,0},
};
/* RC-2 */
static const PresetStep presetSteps19[] = {
    {400,2080,0,-830,1,1,2,2},
    {400,400,-1740,0,2,2,-1,2},
    {1200,400,0,910,-1,-1,-1,0},
};
/* RC-3: reverse through both bends; station vision absorbs the extra down travel. */
#define PRESET_RC3_EXTRA_DOWN_MM 50
static const PresetStep presetSteps20[] = {
    {2100,2080,0,900,1,1,0,1},
    {2100,400,-(1680+PRESET_RC3_EXTRA_DOWN_MM),0,0,0,-1,1},
    {1200,400,0,-900,-1,-1,-1,0},
};
/* CT-1 */
static const PresetStep presetSteps26[] = {
    {400,400,0,-860,-1,-1,2,2},
    {400,1200,850,0,2,2,2,0},
};
/* CT-2 */
static const PresetStep presetSteps27[] = {
    {1200,1200,800,0,2,2,2,0},
    {400,1200,0,-800,2,2,2,0},
};
/* CT-3 */
static const PresetStep presetSteps28[] = {
    {1150,400,0,-50,-1,-1,-1,0},
    {1150,2080,1680,0,0,0,0,0},
    {400,2080,0,-910,1,1,2,2},
    {400,1200,-886,0,2,2,2,0},
};
/* CT-B1 */
static const PresetStep presetSteps29[] = {
    {1200,1150,750,0,-1,-1,-1,0},
    {400,1150,0,-800,1,1,1,0},
    {400,1250,100,0,1,1,1,0},
    {400,1200,-53,0,2,2,2,0},
};
/* CT-B2 */
static const PresetStep presetSteps30[] = {
    {1300,400,0,100,-1,-1,-1,0},
    {1300,300,-100,0,-1,-1,-1,0},
    {2100,300,0,800,-1,-1,0,1},
    {2100,2080,1780,0,0,0,1,1},
    {300,2080,0,-1800,1,1,2,1},
    {300,1300,-780,0,2,2,2,0},
    {400,1300,0,100,2,2,2,0},
    {400,1200,-106,0,2,2,2,0},
};
/* CT-B3 */
static const PresetStep presetSteps31[] = {
    {1200,1150,750,0,-1,-1,-1,0},
    {400,1150,0,-800,1,1,1,0},
    {400,1200,53,0,2,2,2,0},
};
/* TR-1 */
static const PresetStep presetSteps32[] = {
    {400,2080,880,0,2,2,1,2},
    {1200,2080,0,850,1,1,1,0},
};
/* TR-2 */
#define PRESET_TEMP_RIGHT_UP_MM 50
static const PresetStep presetSteps33[] = {
    {400,1200,PRESET_TEMP_RIGHT_UP_MM,0,2,2,2,0},
    {1200,1200,0,800,2,2,2,0},
    {1200,2080,880,0,2,1,1,0},
};
/* TR-3 */
static const PresetStep presetSteps34[] = {
    {400,1200,PRESET_TEMP_RIGHT_UP_MM,0,2,2,2,0},
    {2100,1200,0,1700,-1,-1,0,1},
    {2100,2080,880,0,0,0,1,1},
    {1200,2080,0,-900,1,1,1,0},
};
/* TR-4 */
static const PresetStep presetSteps35[] = {
    {400,400,-850,0,2,2,-1,2},
    {1200,400,0,860,-1,-1,0,2},
    {1200,2080,1680,0,0,1,1,0},
};
/* TR-B1 */
static const PresetStep presetSteps36[] = {
    {400,300,-900,0,2,2,2,0},
    {2100,300,0,1700,-1,-1,0,1},
    {2100,2080,1780,0,0,0,1,1},
    {1300,2080,0,-800,1,1,1,0},
    {1200,2080,0,-106,1,1,1,0},
};
/* TR-B2 */
static const PresetStep presetSteps37[] = {
    {400,1250,50,0,2,2,2,0},
    {1200,1250,0,800,-1,-1,-1,0},
    {1200,1300,50,0,-1,-1,-1,0},
    {1200,2080,780,0,1,1,1,0},
};
/* TR-B3 */
static const PresetStep presetSteps38[] = {
    {400,1150,-50,0,2,2,2,0},
    {1200,1150,0,800,-1,-1,-1,0},
    {1200,2080,930,0,0,0,0,0},
    {1100,2080,0,-100,0,0,0,0},
    {1200,2080,0,106,1,1,1,0},
};
/* H1-1 */
static const PresetStep presetSteps39[] = {
    {400,2080,880,0,2,2,1,2},
    {2250,2080,0,1850,1,1,1,0},
    {2250,2250,170,0,1,1,1,0},
};
/* H1-2 */
static const PresetStep presetSteps40[] = {
    {1200,1200,0,800,2,2,2,0},
    {1200,2080,880,0,2,2,1,1},
    {2250,2080,0,1050,1,1,1,0},
    {2250,2250,170,0,1,1,1,0},
};
/* H1-3 */
static const PresetStep presetSteps41[] = {
    {2100,1200,0,1700,-1,-1,0,1},
    {2100,2080,880,0,0,0,0,0},
    {2250,2080,0,150,1,1,1,0},
    {2250,2250,170,0,1,1,1,0},
};
/* H1-4 */
static const PresetStep presetSteps42[] = {
    {400,400,-850,0,2,2,-1,2},
    {1200,400,0,860,-1,-1,0,2},
    {1200,2080,1680,0,0,0,0,0},
    {2250,2080,0,1050,1,1,1,0},
    {2250,2250,170,0,1,1,1,0},
};
/* H1-B1 */
static const PresetStep presetSteps43[] = {
    {400,300,-900,0,2,2,2,0},
    {2100,300,0,1700,-1,-1,0,1},
    {2100,2080,1780,0,0,0,0,0},
    {2250,2080,0,150,1,1,1,0},
    {2250,2250,170,0,1,1,1,0},
};
/* H1-B2 */
static const PresetStep presetSteps44[] = {
    {400,1250,50,0,2,2,2,0},
    {1300,1250,0,900,-1,-1,-1,0},
    {1300,1200,-50,0,-1,-1,-1,0},
    {2100,1200,0,800,-1,-1,0,1},
    {2100,2080,880,0,0,0,0,0},
    {2250,2080,0,150,1,1,1,0},
    {2250,2250,170,0,1,1,1,0},
};
/* H2-B1 */
static const PresetStep presetSteps45[] = {
    {300,1200,0,-100,2,2,2,0},
    {300,300,-900,0,2,2,-1,1},
    {2100,300,0,1800,-1,-1,-1,0},
    {2100,150,-150,0,-1,-1,-1,0},
    {2250,150,0,150,-1,-1,-1,0},
};
/* H2-B2 */
static const PresetStep presetSteps46[] = {
    {300,1200,0,-100,2,2,2,0},
    {300,2080,880,0,2,2,1,1},
    {2100,2080,0,1800,1,1,1,0},
    {2100,300,-1780,0,2,2,2,0},
    {2100,150,-150,0,-1,-1,-1,0},
    {2250,150,0,150,-1,-1,-1,0},
};
/* H2-B3 */
static const PresetStep presetSteps47[] = {
    {400,1150,-50,0,2,2,2,0},
    {1200,1150,0,800,2,2,2,0},
    {1200,400,-750,0,2,2,2,0},
    {2100,400,0,900,-1,-1,-1,0},
    {2100,150,-250,0,-1,-1,-1,0},
    {2250,150,0,150,-1,-1,-1,0},
};
/* H2-B4 */
static const PresetStep presetSteps48[] = {
    {400,1150,-50,0,2,2,2,0},
    {1200,1150,0,800,-1,-1,-1,0},
    {1200,400,-750,0,-1,-1,-1,0},
    {2100,400,0,900,-1,-1,-1,0},
    {2100,150,-250,0,-1,-1,-1,0},
    {2250,150,0,150,-1,-1,-1,0},
};
/* H2-B5 */
static const PresetStep presetSteps49[] = {
    {400,1150,-50,0,2,2,2,0},
    {2100,1150,0,1700,-1,-1,-1,0},
    {2100,300,-850,0,2,2,2,0},
    {2100,150,-150,0,-1,-1,-1,0},
    {2250,150,0,150,-1,-1,-1,0},
};

/* S1-B2: centre corridor, then y=380. Upper/centre-bottom parking turns;
 * the lower-right bend is a fixed arc. QR calibration: round(820*1080/1050)=843. */
static const PresetStep presetSteps50[] = {
    {2250,2100,-150,0,0,0,0,0},
    {1200,2100,0,-1050,1,1,1,0},
    {1200,380,-1720,0,2,2,2,0},
    {2100,380,0,900,-1,-1,0,1},
    {2100,1200,843,0,0,0,0,0},
};

/* Q-4: lower-right arc into the shared y=380 corridor, then the centre
 * channel to RAW. Upper arrival keeps the workstation's left-facing pose. */
static const PresetStep presetSteps51[] = {
    {2100,380,-820,0,2,2,1,1},
    {1200,380,0,-900,1,1,1,0},
    {1200,2080,1700,0,0,1,1,0},
};

/* IDs are stable, one-based catalog indices; zero means no template. */
static const PresetCandidate presetTemplates[] = {
    {0,0,0},
    {"S1-1",presetSteps1,2},
    {"S1-2",presetSteps2,4},
    {"S1-3",presetSteps3,4},
    {"S1-B1",presetSteps4,6},
    {"S2-1",presetSteps5,2},
    {"S2-2",presetSteps6,4},
    {"S2-3",presetSteps7,4},
    {"S2-B1",presetSteps8,7},
    {"Q1-1",presetSteps9,2},
    {"Q-2",presetSteps10,2},
    {"Q-3",presetSteps11,6},
    {0,0,0}, /* Retired ID 12; keep other template IDs stable. */
    {0,0,0}, /* Retired ID 13. */
    {"Q-B3",presetSteps14,4},
    {0,0,0}, /* Retired ID 15. */
    {"Q2-1",presetSteps16,2},
    {"RC-1",presetSteps17,1},
    {0,0,0}, /* Retired ID 18. */
    {"RC-2",presetSteps19,3},
    {"RC-3",presetSteps20,3},
    {0,0,0}, /* Retired ID 21. */
    {0,0,0}, /* Retired ID 22. */
    {0,0,0}, /* Retired ID 23. */
    {0,0,0}, /* Retired ID 24. */
    {0,0,0}, /* Retired ID 25. */
    {"CT-1",presetSteps26,2},
    {"CT-2",presetSteps27,2},
    {"CT-3",presetSteps28,4},
    {"CT-B1",presetSteps29,4},
    {"CT-B2",presetSteps30,8},
    {"CT-B3",presetSteps31,3},
    {"TR-1",presetSteps32,2},
    {"TR-2",presetSteps33,3},
    {"TR-3",presetSteps34,4},
    {"TR-4",presetSteps35,3},
    {"TR-B1",presetSteps36,5},
    {"TR-B2",presetSteps37,4},
    {"TR-B3",presetSteps38,5},
    {"H1-1",presetSteps39,3},
    {"H1-2",presetSteps40,4},
    {"H1-3",presetSteps41,4},
    {"H1-4",presetSteps42,5},
    {"H1-B1",presetSteps43,5},
    {"H1-B2",presetSteps44,7},
    {"H2-B1",presetSteps45,5},
    {"H2-B2",presetSteps46,6},
    {"H2-B3",presetSteps47,6},
    {"H2-B4",presetSteps48,6},
    {"H2-B5",presetSteps49,5},
    {"S1-B2",presetSteps50,5},
    {"Q-4",presetSteps51,3},
};
#define PRESET_TEMPLATE_COUNT ((unsigned)(sizeof(presetTemplates)/sizeof(presetTemplates[0])-1U))

enum { PRESET_GROUP_S1, PRESET_GROUP_S2, PRESET_GROUP_Q1, PRESET_GROUP_Q2, PRESET_GROUP_RC, PRESET_GROUP_CT, PRESET_GROUP_TR, PRESET_GROUP_H1, PRESET_GROUP_H2, PRESET_GROUP_COUNT };
static const uint8_t presetOrderS1[] = {1,2,50,3,4};
static const uint8_t presetOrderS2[] = {5,6,7,8};
static const uint8_t presetOrderQ1[] = {9,10,11,51,14};
static const uint8_t presetOrderQ2[] = {16,10,11,51,14};
static const uint8_t presetOrderRC[] = {17,19,20};
static const uint8_t presetOrderCT[] = {26,27,28,29,30,31};
static const uint8_t presetOrderTR[] = {32,33,34,35,36,37,38};
static const uint8_t presetOrderH1[] = {39,40,41,42,43,44};
static const uint8_t presetOrderH2[] = {45,46,47,48,49};
static const PresetGroup presetGroups[PRESET_GROUP_COUNT] = {
    {{2250,2250,0},{2100,1200,0},presetOrderS1,5}, /* S1 */
    {{2250,150,0},{2100,1200,0},presetOrderS2,4}, /* S2 */
    {{2100,1200,0},{1200,2080,1},presetOrderQ1,5}, /* Q1 */
    {{2100,1200,0},{1200,2080,1},presetOrderQ2,5}, /* Q2 */
    {{1200,2080,1},{1200,400,-1},presetOrderRC,3}, /* RC */
    {{1200,400,-1},{400,1200,2},presetOrderCT,6}, /* CT */
    {{400,1200,2},{1200,2080,1},presetOrderTR,7}, /* TR */
    {{400,1200,2},{2250,2250,1},presetOrderH1,6}, /* H1 */
    {{400,1200,2},{2250,150,-1},presetOrderH2,5}, /* H2 */
};
#endif
