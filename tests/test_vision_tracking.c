#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include "../firmware/vision_tracking.h"

static VisionTracking tracking;
static VisionCalibration calibration = {1U,{0,0,320,240},160,120,{-1,0,0,-1}};
static VisionPacket packet;
static float forward, right;

static void begin(uint16_t rpm, uint32_t duration, float budget)
{
    memset(&packet,0,sizeof(packet));
    calibration.calibrated = 1U;
    calibration.matrix[0] = calibration.matrix[3] = -1.0f;
    packet.type = VISION_MESSAGE_RESULT;
    packet.token = 50U; packet.mode = VISION_MODE_RING; packet.target = 2U;
    packet.flags = VISION_FLAG_VALID | VISION_FLAG_TRACKING;
    packet.value[0] = 120; packet.value[1] = 80; packet.value[2] = 80;
    packet.value[6] = 1;
    vision_tracking_begin(&tracking,50U,VISION_MODE_RING,0U,2U,budget,0,rpm,duration,100U);
    assert(tracking.state == VISION_TRACK_WAIT);
}

static void receive(uint16_t sequence, uint32_t now)
{
    packet.value[6] = (int16_t)sequence;
    vision_tracking_observe(&tracking,&calibration,&packet,now,now);
}

static VisionTrackingState tick(uint32_t now)
{
    return vision_tracking_tick(&tracking,now,1.0f,1.0f,&forward,&right);
}

static void handshakeAndIdentity(void)
{
    unsigned i;
    for (i=0;i<6;++i) {
        begin(30U,1200U,80.0f);
        if(i==0) packet.token++;
        if(i==1) packet.mode=VISION_MODE_MATERIAL;
        if(i==2) packet.selector=1;
        if(i==3) packet.target=1;
        if(i==4) packet.flags&=(uint8_t)~VISION_FLAG_TRACKING;
        if(i==5) packet.type=VISION_MESSAGE_REQUEST;
        receive(1U,200U);
        assert(tracking.state==VISION_TRACK_WAIT && !tracking.sequenceValid);
        assert(tick(1899U)==VISION_TRACK_WAIT && forward==0 && right==0);
        assert(tick(1900U)==VISION_TRACK_FAILED && forward==0 && right==0);
        assert(!strcmp(tracking.reason,"TRACK_ACK_TIMEOUT"));
    }
    begin(30U,1200U,80.0f);
    assert(tick(1899U)==VISION_TRACK_WAIT && forward==0 && right==0);
    receive(1U,1899U);
    assert(tracking.state==VISION_TRACK_RUN && tracking.deadlineMs==3099U);
    assert(tick(1899U)==VISION_TRACK_RUN && forward==0 && right==0);
    assert(tick(1919U)==VISION_TRACK_RUN && fabsf(forward)+fabsf(right)>0);
    begin(30U,1200U,80.0f); receive(1U,1900U);
    assert(tracking.state==VISION_TRACK_FAILED); /* No late ACK resurrection. */
}

static void frameValidation(void)
{
    unsigned i;
    for(i=0;i<8;++i) {
        begin(30U,1200U,80.0f);
        if(i==0) packet.flags&=(uint8_t)~VISION_FLAG_VALID;
        if(i==1) packet.value[2]=59;
        if(i==2) packet.value[0]=-1;
        if(i==3) packet.value[1]=240;
        if(i==4) calibration.calibrated=0;
        if(i==5) calibration.matrix[0]=NAN;
        if(i==6) packet.flags|=VISION_FLAG_MATERIAL_LAYOUT;
        if(i==7) packet.flags|=VISION_FLAG_STABLE;
        receive(1U,100U);
        assert(tracking.state==VISION_TRACK_FAILED);
        assert(tick(120U)==VISION_TRACK_FAILED && forward==0 && right==0);
    }
    begin(30U,1200U,80.0f); packet.value[7]=219; receive(1U,100U);
    assert(tracking.state==VISION_TRACK_WAIT); /* 219+32 > 250; does not ACK. */
    packet.value[7]=218; receive(1U,100U);
    assert(tracking.state==VISION_TRACK_RUN && tracking.lastFrameAt==100U-250U);
    assert(tick(100U)==VISION_TRACK_RUN);
    assert(tick(101U)==VISION_TRACK_FAILED);
    begin(30U,1200U,80.0f); receive(1U,100U);
    packet.value[7]=200;
    vision_tracking_observe(&tracking,&calibration,&packet,100U,120U);
    assert(tracking.state==VISION_TRACK_RUN); /* Duplicate never refreshes age. */
    packet.value[6]=2;
    vision_tracking_observe(&tracking,&calibration,&packet,100U,120U);
    assert(tracking.state==VISION_TRACK_FAILED); /* 200+32+20 > 250. */
    begin(30U,1200U,80.0f); receive(1U,100U);
    receive(2U,319U); assert(tracking.state==VISION_TRACK_FAILED); /* Late new frame cannot restart. */
    begin(30U,1200U,80.0f); receive(1U,100U);
    packet.flags=VISION_FLAG_TRACKING; receive(2U,120U);
    assert(tracking.state==VISION_TRACK_FAILED); /* Explicit target loss stops immediately. */

    begin(30U,1200U,80.0f);
    tracking.mode=packet.mode=VISION_MODE_MATERIAL;
    tracking.selector=packet.selector=3U; packet.value[5]=3;
    packet.value[2]=0;
    receive(1U,100U); assert(tracking.state==VISION_TRACK_RUN); /* Original material quality rule. */
    packet.value[5]=2;
    receive(2U,120U); assert(tracking.state==VISION_TRACK_FAILED);
}

static void sequenceFreshness(void)
{
    begin(30U,1200U,80.0f); receive(1U,100U);
    receive(1U,200U); assert(tracking.sequence==1U && tracking.lastFrameAt==68U);
    receive(0U,220U); assert(tracking.sequence==1U && tracking.lastFrameAt==68U);
    assert(tick(318U)==VISION_TRACK_RUN);
    assert(tick(319U)==VISION_TRACK_FAILED);
    begin(30U,1200U,80.0f); receive(65535U,100U);
    receive(0U,120U); assert(tracking.sequence==0U && tracking.lastFrameAt==88U);
    receive(30U,140U); assert(tracking.sequence==30U && tracking.lastFrameAt==108U);
    receive(29U,160U); assert(tracking.sequence==30U && tracking.lastFrameAt==108U);
    receive(31U,180U); assert(tracking.sequence==31U && tracking.lastFrameAt==148U);
}

static void thirteenFpsCamera(void)
{
    uint32_t now, nextFrame = 209U;
    uint16_t sequence = 0U;
    begin(30U,3000U,1000.0f);
    packet.value[7] = 77; /* Capture + detection; receiver adds the existing 32ms wire budget. */
    for(now=100U;now<=2000U;++now) {
        if(now==nextFrame) {
            receive(++sequence,now);
            nextFrame += 77U;
        }
        assert(tick(now)==(now<209U?VISION_TRACK_WAIT:VISION_TRACK_RUN));
    }
    assert(sequence>20U && tracking.sequence==sequence);
    assert(fabsf(forward)+fabsf(right)>0 && fabsf(forward)+fabsf(right)<=30.0f);
}

static void speedAndDirection(void)
{
    unsigned i;
    float previous;
    begin(30U,3000U,500.0f); receive(1U,100U);
    for(i=120U;i<=200U;i+=20U) {
        receive((uint16_t)i,i); assert(tick(i)==VISION_TRACK_RUN);
        assert(fabsf(forward)+fabsf(right)<=30.0f);
    }
    previous=forward;
    packet.value[0]=157; packet.value[1]=120; receive(201U,220U); tick(220U);
    assert(forward>=0 && forward<previous && previous-forward<=ROUTE_BRAKE_RPM_S*0.020f+1.0f);
    packet.value[0]=200; receive(202U,240U); tick(240U);
    assert(forward==0); /* Sign reversal first sends a zero command. */
    receive(203U,260U); tick(260U); assert(forward<0);
    begin(10U,3000U,500.0f); receive(1U,100U);
    for(i=120U;i<=200U;i+=20U) {
        receive((uint16_t)i,i); tick(i);
        assert(fabsf(forward)+fabsf(right)<=10.0f);
    }
    begin(30U,3000U,500.0f); receive(1U,100U);
    for(i=101U;i<120U;++i) { tick(i); assert(forward==0 && right==0); }
    tick(120U); assert(forward>0 && right>0); /* Fast polling must not starve the 20ms ramp. */
    previous=forward+right;
    for(i=121U;i<=140U;++i) tick(i);
    assert(fabsf(tracking.travelledMm-previous*ROUTE_MM_PER_REV/60000.0f*20.0f)<0.001f);
    assert(vision_tracking_tick(&tracking,160U,0.0f,1.0f,&forward,&right)==VISION_TRACK_FAILED);
    begin(30U,3000U,500.0f); receive(1U,100U); tick(120U);
    packet.value[0]=200; receive(2U,121U); tick(121U);
    assert(forward==0); /* A frame arriving between 20ms updates stops the wrong sign now. */
    tick(140U); assert(forward==0);
    tick(141U); assert(forward<0);
}

static void completionAndBudget(void)
{
    uint32_t now;
    begin(30U,1200U,80.0f); receive(1U,100U); tick(120U);
    packet.value[0]=160; packet.value[1]=120; receive(2U,140U);
    assert(tracking.state==VISION_TRACK_RUN && tracking.confirmations==1U);
    assert(tick(140U)==VISION_TRACK_RUN && forward==0 && right==0);
    receive(2U,150U);assert(tracking.confirmations==1U); /* Duplicate is not a second confirmation. */
    receive(3U,160U);
    assert(tracking.state==VISION_TRACK_DONE && !strcmp(tracking.reason,"TRACK_IN_RANGE"));
    assert(tick(160U)==VISION_TRACK_DONE && forward==0 && right==0);
    receive(4U,180U); assert(tracking.state==VISION_TRACK_DONE);
    begin(15U,1200U,80.0f);packet.value[0]=160;packet.value[1]=120;receive(1U,100U);
    packet.value[0]=161;receive(2U,120U);assert(!tracking.confirmations);
    packet.value[0]=160;receive(3U,140U);assert(tracking.state==VISION_TRACK_RUN);
    receive(4U,160U);assert(tracking.state==VISION_TRACK_DONE);
    begin(30U,60U,80.0f); receive(1U,100U); tick(120U); tick(140U);
    assert(tick(160U)==VISION_TRACK_DONE && !strcmp(tracking.reason,"TRACK_DEADLINE"));
    begin(30U,10000U,1.0f); receive(1U,100U);
    for(now=120U;now<1000U && tracking.state==VISION_TRACK_RUN;now+=20U) {
        receive((uint16_t)now,now); tick(now);
        assert(tracking.travelledMm<=tracking.budgetMm+0.0001f);
    }
    assert(tracking.state==VISION_TRACK_DONE && !strcmp(tracking.reason,"TRACK_BUDGET"));
    begin(30U,10000U,8.0f); receive(1U,100U); tick(120U);
    assert(forward+right>0);
    for(now=240U;now<3000U && tracking.state==VISION_TRACK_RUN;now+=150U) {
        receive((uint16_t)now,now); tick(now);
        assert(tracking.travelledMm<=tracking.budgetMm+0.0001f);
    }
    assert(tracking.state==VISION_TRACK_DONE && !strcmp(tracking.reason,"TRACK_BUDGET"));
    vision_tracking_cancel(&tracking); assert(tracking.state==VISION_TRACK_IDLE);
    receive(1000U,1000U); assert(tracking.state==VISION_TRACK_IDLE);
    assert(tick(1000U)==VISION_TRACK_IDLE && forward==0 && right==0);
    vision_tracking_begin(&tracking,1U,VISION_MODE_MATERIAL,1U,0U,10,0,30,1000,0);
    assert(tracking.state==VISION_TRACK_FAILED); /* Raw-material picking is excluded. */
    vision_tracking_begin(&tracking,1U,VISION_MODE_RING,0U,2U,0,0,30,1000,0);
    assert(tracking.state==VISION_TRACK_DONE);
}

static void zeroPixelTolerance(void)
{
    static const int16_t offsets[7][2] = {{0,0},{1,0},{-1,0},{0,1},{0,-1},{2,2},{-2,-2}};
    unsigned index, mode;
    for(mode=VISION_MODE_MATERIAL;mode<=VISION_MODE_RING;++mode) {
        for(index=0;index<7;++index) {
            begin(30U,1200U,80.0f);
            if(mode==VISION_MODE_MATERIAL) {
                tracking.mode=packet.mode=VISION_MODE_MATERIAL;
                tracking.selector=packet.selector=3U; packet.value[5]=3;
            }
            packet.value[0]=(int16_t)(160+offsets[index][0]);
            packet.value[1]=(int16_t)(120+offsets[index][1]);
            receive(1U,100U);
            if(index==0U && mode==VISION_MODE_RING) {
                assert(tracking.state==VISION_TRACK_RUN);
                receive(2U,120U);
            }
            assert(tracking.state==(index==0U?VISION_TRACK_DONE:VISION_TRACK_RUN));
            if(index==0U) assert(!strcmp(tracking.reason,"TRACK_IN_RANGE"));
        }
    }
}

int main(void)
{
    handshakeAndIdentity(); frameValidation(); sequenceFreshness(); thirteenFpsCamera();
    speedAndDirection(); completionAndBudget(); zeroPixelTolerance();
    puts("PASS vision tracking: explicit ACK, strict frames, bounded fresh velocity, reversal stop, deadline/budget and cancellation");
    return 0;
}
