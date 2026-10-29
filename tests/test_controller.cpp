#include <QtTest>
#include <limits>
#include "../upper_computer/Controller.h"
class Wire:public QIODevice {
public: QByteArray written;
    Wire(){open(ReadWrite);}
    qint64 readData(char *,qint64)override{return 0;}
    qint64 writeData(const char *p,qint64 n)override{written.append(p,int(n));return n;}
    SsFrame last(){SsParser parser{};SsFrame f{},result{};for(char b:written)if(ss_feed(&parser,quint8(b),&f))result=f;written.clear();return result;}
};
class ControllerTest:public QObject {
    Q_OBJECT
private slots:
    void turnTestDirectionsAndCompletion(){
        Wire wire;Controller c;c.device_=&wire;
        c.testTurn(1);QVERIFY(wire.written.isEmpty());c.connected_=true;
        c.testTurn(1);QVERIFY(wire.written.isEmpty());c.confirmed_=true;
        for(int direction:{0,2,-2}){c.testTurn(direction);QVERIFY(wire.written.isEmpty());}
        c.busy_=true;c.testTurn(1);QVERIFY(wire.written.isEmpty());c.busy_=false;
        for(int direction:{1,-1}){
            c.testTurn(direction);auto q=wire.last();QCOMPARE(int(q.command),int(SS_TURN_TEST));
            QCOMPARE(int(q.length),1);QCOMPARE(int(qint8(q.data[0])),direction);QVERIFY(c.turnTesting() && c.busy());
            c.testTurn(1);c.testStraight();c.testLateral(1,100);QTest::qWait(120);QVERIFY(wire.written.isEmpty());
            SsFrame r{};r.command=q.command;r.sequence=q.sequence;r.flags=SS_RESPONSE;r.length=5;
            ss_put16(r.data+1,300);ss_put16(r.data+3,100);c.handle(r);
            QVERIFY(c.turnTesting() && c.turnTestState().contains("300RPM"));
            r.flags=SS_EVENT;r.length=1;r.sequence++;c.handle(r);QVERIFY(c.turnTesting());
            r.sequence=q.sequence;c.handle(r);QVERIFY(!c.turnTesting() && c.turnTestState().contains("完成"));
        }
        c.testTurn(1);auto q=wire.last();SsFrame r{};r.command=q.command;r.sequence=q.sequence;r.flags=SS_RESPONSE|SS_ERROR;r.length=1;r.data[0]=1;c.handle(r);
        QVERIFY(!c.turnTesting() && c.turnTestState().contains("未启动"));
        c.testTurn(1);q=wire.last();r.sequence=q.sequence;r.flags=SS_RESPONSE;r.data[0]=0;c.handle(r);
        q=wire.last();QCOMPARE(int(q.command),int(SS_STOP));QVERIFY(!c.turnTesting());
        c.testTurn(1);q=wire.last();c.pauseManual();QCOMPARE(int(wire.last().command),int(SS_STOP));
        QVERIFY(c.turnTesting() && c.turnTestState().contains("停止"));
        r.command=SS_TURN_TEST;r.sequence=q.sequence;r.flags=SS_EVENT;r.length=1;r.data[0]=4;c.handle(r);
        QVERIFY(!c.turnTesting() && c.turnTestState().contains("中止"));
        c.testTurn(-1);wire.last();c.disconnectDevice();QCOMPARE(int(wire.last().command),int(SS_STOP));
        QVERIFY(!c.turnTesting() && c.turnTestState().contains("未知"));c.device_=nullptr;
    }
    void straightTestRequiresSafeIdleAndCompletion(){
        Wire wire;Controller c;c.device_=&wire;
        c.testStraight();QVERIFY(wire.written.isEmpty());
        c.connected_=true;c.testStraight();QVERIFY(wire.written.isEmpty());
        c.confirmed_=true;c.busy_=true;c.testStraight();QVERIFY(wire.written.isEmpty());c.busy_=false;
        c.calibrationStage_=1;c.testStraight();QVERIFY(wire.written.isEmpty());c.calibrationStage_=0;
        c.servoTargets_.insert(4,100);c.testStraight();QVERIFY(wire.written.isEmpty());c.servoTargets_.clear();
        c.axis6TestSequence_=9;c.testStraight();QVERIFY(wire.written.isEmpty());c.axis6TestSequence_=-1;
        c.lateralTestSequence_=9;c.testStraight();QVERIFY(wire.written.isEmpty());c.lateralTestSequence_=-1;
        QVERIFY(!c.mechanismValid());c.testStraight();auto q=wire.last();
        QCOMPARE(int(q.command),int(SS_STRAIGHT_TEST));QCOMPARE(int(q.length),0);
        QVERIFY(c.straightTesting() && c.busy());
        c.testStraight();c.testLateral(1,100);c.testAxis6();QTest::qWait(150);QVERIFY(wire.written.isEmpty());
        SsFrame r{};r.command=q.command;r.sequence=q.sequence;r.flags=SS_RESPONSE;r.length=3;
        ss_put16(r.data+1,240);c.handle(r);
        QVERIFY(c.straightTesting() && c.straightTestState().contains("240RPM"));
        r.flags=SS_EVENT;r.length=1;r.sequence=q.sequence+1;c.handle(r);QVERIFY(c.straightTesting());
        r.sequence=q.sequence;c.handle(r);QVERIFY(!c.straightTesting());
        QVERIFY(c.straightTestState().contains("1680mm") && c.straightTestState().contains("完成"));
        c.testStraight();q=wire.last();r.sequence=q.sequence;r.data[0]=4;c.handle(r);
        QVERIFY(!c.straightTesting() && c.straightTestState().contains("中止"));
        c.disconnectDevice();c.device_=nullptr;
    }
    void straightTestRejectsInvalidReplyAndStopsOnPauseOrDisconnect(){
        Wire wire;Controller c;c.device_=&wire;c.connected_=c.confirmed_=true;
        c.testStraight();auto q=wire.last();SsFrame r{};
        r.command=q.command;r.sequence=q.sequence;r.flags=SS_RESPONSE|SS_ERROR;r.length=1;r.data[0]=1;c.handle(r);
        QVERIFY(!c.straightTesting() && c.straightTestState().contains("未启动"));
        c.testStraight();q=wire.last();r.sequence=q.sequence;r.flags=SS_RESPONSE;r.data[0]=0;c.handle(r);
        q=wire.last();QCOMPARE(int(q.command),int(SS_STOP));
        QVERIFY(!c.straightTesting() && c.straightTestState().contains("应答无效"));
        c.testStraight();wire.last();c.pauseManual();q=wire.last();QCOMPARE(int(q.command),int(SS_STOP));
        QVERIFY(c.straightTesting() && c.straightTestState().contains("停止"));
        c.disconnectDevice();q=wire.last();QCOMPARE(int(q.command),int(SS_STOP));
        QVERIFY(!c.straightTesting() && c.straightTestState().contains("未知"));c.device_=nullptr;
    }
    void lateralTestRequiresSafeIdleAndCompletion(){
        Wire wire;Controller c;c.device_=&wire;
        c.testLateral(-1,1000);QVERIFY(wire.written.isEmpty());
        c.connected_=true;c.testLateral(-1,1000);QVERIFY(wire.written.isEmpty());
        c.confirmed_=true;c.busy_=true;c.testLateral(-1,1000);QVERIFY(wire.written.isEmpty());c.busy_=false;
        for(double value:{0.,99.,2001.,100.5,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}){
            c.testLateral(-1,value);QVERIFY(wire.written.isEmpty());
        }
        c.testLateral(0,1000);QVERIFY(wire.written.isEmpty());
        c.testLateral(-1,1000);auto q=wire.last();
        QCOMPARE(int(q.command),int(SS_LATERAL_TEST));QCOMPARE(int(q.length),2);
        QCOMPARE(qint16(ss_get16(q.data)),qint16(-1000));QVERIFY(c.lateralTesting() && c.busy());
        c.testLateral(1,1000);QTest::qWait(150);QVERIFY(wire.written.isEmpty());
        SsFrame r{};r.command=q.command;r.sequence=q.sequence;r.flags=SS_RESPONSE;r.length=3;ss_put16(r.data+1,150);
        c.handle(r);QVERIFY(c.lateralTesting() && c.lateralTestState().contains("150RPM"));
        r.flags=SS_EVENT;r.length=1;r.sequence=q.sequence+1;c.handle(r);QVERIFY(c.lateralTesting());
        r.sequence=q.sequence;c.handle(r);QVERIFY(!c.lateralTesting());QVERIFY(c.lateralTestState().contains("左1000mm"));
        c.testLateral(1,2000);q=wire.last();QCOMPARE(qint16(ss_get16(q.data)),qint16(2000));
        r.sequence=q.sequence;r.data[0]=4;c.handle(r);QVERIFY(!c.lateralTesting() && c.lateralTestState().contains("中止"));
        c.disconnectDevice();c.device_=nullptr;
    }
    void lateralTestRejectsAndStopsOnPauseOrDisconnect(){
        Wire wire;Controller c;c.device_=&wire;c.connected_=c.confirmed_=true;
        c.testLateral(1,100);auto q=wire.last();SsFrame r{};
        r.command=q.command;r.sequence=q.sequence;r.flags=SS_RESPONSE|SS_ERROR;r.length=1;r.data[0]=1;c.handle(r);
        QVERIFY(!c.lateralTesting() && c.lateralTestState().contains("未启动"));
        c.testLateral(1,100);wire.last();c.pauseManual();q=wire.last();QCOMPARE(int(q.command),int(SS_STOP));
        QVERIFY(c.lateralTestState().contains("停止"));
        c.disconnectDevice();q=wire.last();QCOMPARE(int(q.command),int(SS_STOP));
        QVERIFY(!c.lateralTesting() && c.lateralTestState().contains("未知"));c.device_=nullptr;
    }
    void taskLogsReadInPagesWithoutReplayingCalibration(){
        Wire wire;Controller c;c.device_=&wire;
        c.readTaskLog();QVERIFY(wire.written.isEmpty());
        c.connected_=true;QSignalSpy logs(&c,&Controller::taskLogReceived);
        c.readTaskLog();auto q=wire.last();QCOMPARE(int(q.command),int(SS_GET_LOG));QCOMPARE(int(q.length),12);
        c.readTaskLog();QVERIFY(wire.written.isEmpty());QVERIFY(c.logsLoading());
        const QByteArray text=QStringLiteral("ERR offline\nVISION PAUSED\n中文日志\n").toUtf8();
        const int split=text.indexOf(char(0xe4))+1;
        SsFrame r{};r.flags=SS_RESPONSE;r.sequence=q.sequence;r.command=q.command;r.length=quint16(14+split);
        ss_put32(r.data+1,7);ss_put32(r.data+5,split);ss_put32(r.data+9,text.size());r.data[13]=1;
        memcpy(r.data+14,text.constData(),split);c.handle(r);
        QVERIFY(logs.isEmpty());q=wire.last();QCOMPARE(ss_get32(q.data),quint32(7));
        QCOMPARE(ss_get32(q.data+4),quint32(split));QCOMPARE(ss_get32(q.data+8),quint32(text.size()));
        c.calibrationStage_=2;
        r.sequence=q.sequence;r.length=quint16(14+text.size()-split);ss_put32(r.data+5,text.size());
        memcpy(r.data+14,text.constData()+split,text.size()-split);c.handle(r);
        QCOMPARE(logs.size(),1);QCOMPARE(logs[0][0].toString(),QString::fromUtf8(text));
        QCOMPARE(c.calibrationStage_,2);QVERIFY(!c.logsLoading());QVERIFY(c.logsState().contains("覆盖"));
        QVERIFY(wire.written.isEmpty());
        c.readTaskLog();wire.last();c.disconnectDevice();QVERIFY(!c.logsLoading());
        QCOMPARE(logs.size(),1);QVERIFY(c.logsState().contains("中断"));c.device_=nullptr;
    }
    void taskLogErrorsKeepPreviouslyDisplayedLogs(){
        Wire wire;Controller c;c.device_=&wire;c.connected_=true;
        QSignalSpy logs(&c,&Controller::taskLogReceived);
        for(int error:{1,5}){
            c.readTaskLog();auto q=wire.last();SsFrame r{};
            r.flags=SS_RESPONSE|SS_ERROR;r.command=q.command;r.sequence=q.sequence;r.length=1;r.data[0]=quint8(error);
            c.handle(r);QVERIFY(!c.logsLoading());QVERIFY(logs.isEmpty());
        }
        c.readTaskLog();auto q=wire.last();SsFrame r{};
        r.flags=SS_RESPONSE;r.command=q.command;r.sequence=q.sequence;r.length=14;
        ss_put32(r.data+1,1);ss_put32(r.data+9,10);c.handle(r);
        QVERIFY(!c.logsLoading());QVERIFY(c.logsState().contains("无效"));QVERIFY(logs.isEmpty());
        c.readTaskLog();q=wire.last();r.sequence=q.sequence;ss_put32(r.data+9,0);c.handle(r);
        QCOMPARE(logs.size(),1);QVERIFY(logs[0][0].toString().contains("尚无"));
        c.disconnectDevice();c.device_=nullptr;
    }
    void handshakeAndConfirmation(){
        Wire wire;Controller c;c.device_=&wire;c.opened();auto q=wire.last();QCOMPARE(int(q.command),int(SS_HELLO));QVERIFY(!c.connected());
        SsFrame reply{};reply.flags=SS_RESPONSE;reply.sequence=q.sequence;reply.command=SS_HELLO;reply.length=4;reply.data[1]=1;reply.data[2]=SS_ROUTE_REVISION;reply.data[3]=1;c.handle(reply);
        QVERIFY(c.connected());QVERIFY(!c.confirmed());
        c.confirmZero();q=wire.last();QCOMPARE(int(q.command),int(SS_CLEAR));
        reply.sequence=q.sequence;reply.command=SS_CLEAR;reply.length=1;c.handle(reply);QVERIFY(c.confirmed());
        c.disconnectDevice();QVERIFY(!c.confirmed() && !c.connected());c.device_=nullptr;
    }
    void ringsRequireReadAndDeviceAck(){
        Wire wire;Controller c;c.device_=&wire;c.connected_=true;
        QVariantList values;for(int i=0;i<3;++i)values.append(QVariantMap{{"horizontal",-32.0},{"turret",261.2}});
        c.saveRings(values);QVERIFY(wire.written.isEmpty());
        c.readRings();auto q=wire.last();SsFrame r{};r.flags=SS_RESPONSE;r.sequence=q.sequence;r.command=SS_GET_RINGS;r.length=14;
        for(int i=0;i<3;++i){ss_put16(r.data+2+i*4,quint16(-320));ss_put16(r.data+4+i*4,2612);}
        c.handle(r);QVERIFY(c.ringsRead());QCOMPARE(c.rings()[0].toMap()["horizontal"].toDouble(),-32.0);
        c.saveRings(values);q=wire.last();QCOMPARE(int(q.command),int(SS_SET_RINGS));QCOMPARE(qint16(ss_get16(q.data)),qint16(-320));QVERIFY(c.ringState().contains("等待"));
        c.disconnectDevice();QVERIFY(!c.ringsRead());c.device_=nullptr;
    }
    void oldFirmwareCannotStartRegionRoutes(){
        Wire wire;Controller c;c.device_=&wire;c.opened();auto q=wire.last();
        SsFrame r{};r.flags=SS_RESPONSE;r.sequence=q.sequence;r.command=SS_HELLO;r.length=4;r.data[1]=1;r.data[3]=1;
        c.handle(r);QVERIFY(!c.connected());QVERIFY(c.status().contains("15区"));
        c.run(1,1,0x0810);QVERIFY(wire.written.isEmpty());c.device_=nullptr;
    }
    void regionRunValidatesAndSendsFiveBytes(){
        Wire wire;Controller c;c.device_=&wire;c.connected_=c.confirmed_=true;
        for(int type=0;type<3;++type)for(int regions:{0,1,0x4000,0x0810}){
            c.run(type,2,regions,240);auto q=wire.last();
            QCOMPARE(int(q.command),int(SS_PREPARE)+type);QCOMPARE(int(q.length),5);
            QCOMPARE(int(q.data[0]),2);QCOMPARE(ss_get16(q.data+1),quint16(regions));QCOMPARE(ss_get16(q.data+3),quint16(240));
            c.run(type,2,0x0810);QVERIFY(wire.written.isEmpty());
            SsFrame r{};r.flags=SS_RESPONSE;r.sequence=q.sequence;r.command=q.command;r.length=1;c.handle(r);
        }
        for(int invalid:{-1,7,0x8001,0x10003}){c.run(1,1,invalid);QVERIFY(wire.written.isEmpty());}
        c.run(-1,1,0x0810);c.run(3,1,0x0810);c.run(1,0,0x0810);c.run(1,3,0x0810);
        c.run(1,1,0x0810,9);c.run(1,1,0x0810,251);QVERIFY(wire.written.isEmpty());
        c.busy_=true;c.run(1,1,0x0810);QVERIFY(wire.written.isEmpty());
        QCOMPARE(c.obstacleRegions().size(),15);QCOMPARE(c.obstacleRegions()[7].toMap()["x0"].toInt(),1000);
        c.disconnectDevice();c.device_=nullptr;
    }
    void invalidScaleAndJogAreNotSent(){
        Wire wire;Controller c;c.device_=&wire;c.connected_=c.confirmed_=true;
        c.applyScale(std::numeric_limits<double>::quiet_NaN(),1,true);QVERIFY(wire.written.isEmpty());
        c.applyScale(0.049,1,true);QVERIFY(wire.written.isEmpty());
        c.jogAxis(4,1);c.jogAxis(5,6);QVERIFY(wire.written.isEmpty());
        c.applyScale(1.111,1.143,true);auto q=wire.last();QCOMPARE(int(q.command),int(SS_SAVE_SCALE));QCOMPARE(ss_get16(q.data),quint16(1111));
        c.disconnectDevice();c.device_=nullptr;
    }
    void servoCoalescingAndReleaseStop(){
        Wire wire;Controller c;c.device_=&wire;c.connected_=c.confirmed_=true;
        c.setServo(2,110);c.setServo(2,112.3);QVERIFY(wire.written.isEmpty());c.flushServos();
        auto q=wire.last();QCOMPARE(int(q.command),int(SS_SERVO));QCOMPARE(ss_get16(q.data+1),quint16(1123));
        c.drive(1);QVERIFY(c.driveTimer_.isActive());wire.last();c.pauseManual();QVERIFY(!c.driveTimer_.isActive());
        q=wire.last();QCOMPARE(int(q.command),int(SS_STOP));c.disconnectDevice();c.device_=nullptr;
    }
    void noAutomaticMovementRetry(){
        Wire wire;Controller c;c.device_=&wire;c.connected_=c.confirmed_=true;c.jogAxis(5,1);auto q=wire.last();
        QCOMPARE(int(q.command),int(SS_AUX));QTest::qWait(150);QVERIFY(wire.written.isEmpty());
        c.disconnectDevice();c.device_=nullptr;
    }
    void axis6TestRequiresSafeIdleAndCompletionEvent(){
        Wire wire;Controller c;c.device_=&wire;
        c.testAxis6();QVERIFY(wire.written.isEmpty());
        c.connected_=true;c.testAxis6();QVERIFY(wire.written.isEmpty());
        c.confirmed_=true;c.testAxis6();QVERIFY(wire.written.isEmpty());
        c.mechanismValid_=c.busy_=true;c.testAxis6();QVERIFY(wire.written.isEmpty());
        c.busy_=false;c.testAxis6();auto q=wire.last();
        QCOMPARE(int(q.command),int(SS_ACTION));QCOMPARE(int(q.length),1);QCOMPARE(int(q.data[0]),3);
        QVERIFY(c.axis6Testing() && c.busy());
        c.testAxis6();QVERIFY(wire.written.isEmpty());
        SsFrame r{};r.flags=SS_RESPONSE;r.sequence=q.sequence;r.command=SS_ACTION;r.length=1;c.handle(r);
        QVERIFY(c.axis6Testing() && c.axis6TestState().contains("进行中"));
        r.flags=SS_EVENT;r.sequence=q.sequence+1;c.handle(r);QVERIFY(c.axis6Testing());
        r.sequence=q.sequence;c.handle(r);QVERIFY(!c.axis6Testing() && c.axis6TestState().contains("完成"));
        c.disconnectDevice();c.device_=nullptr;
    }
    void axis6TestRejectionAndFailureAreNotSuccess(){
        Wire wire;Controller c;c.device_=&wire;c.connected_=c.confirmed_=c.mechanismValid_=true;
        c.testAxis6();auto q=wire.last();SsFrame r{};r.flags=SS_RESPONSE|SS_ERROR;
        r.command=SS_ACTION;r.sequence=q.sequence;r.length=1;r.data[0]=1;c.handle(r);
        QVERIFY(!c.axis6Testing() && c.axis6TestState().contains("未启动"));
        c.testAxis6();q=wire.last();r.flags=SS_RESPONSE;r.sequence=q.sequence;r.data[0]=0;c.handle(r);
        r.flags=SS_EVENT;r.data[0]=4;c.handle(r);
        QVERIFY(!c.axis6Testing() && !c.mechanismValid() && c.axis6TestState().contains("中止"));
        c.disconnectDevice();c.device_=nullptr;
    }
    void axis6TestPausesAndDisconnectsWithoutRepeatingMovement(){
        Wire wire;Controller c;c.device_=&wire;c.connected_=c.confirmed_=c.mechanismValid_=true;
        c.testAxis6();wire.last();QTest::qWait(150);QVERIFY(wire.written.isEmpty());
        c.pauseManual();auto q=wire.last();QCOMPARE(int(q.command),int(SS_STOP));
        QVERIFY(c.axis6TestState().contains("停止"));
        c.disconnectDevice();q=wire.last();QCOMPARE(int(q.command),int(SS_STOP));
        QVERIFY(!c.axis6Testing() && c.axis6TestState().contains("未知"));c.device_=nullptr;
    }
    void failedSaveIsNotDisplayedAsSuccess(){
        Wire wire;Controller c;c.device_=&wire;c.connected_=true;c.applyScale(1.111,1.143,true);
        QVERIFY(c.parametersPending());auto q=wire.last();SsFrame r{};r.command=q.command;r.sequence=q.sequence;r.flags=SS_RESPONSE|SS_ERROR;r.length=1;r.data[0]=4;c.handle(r);
        QVERIFY(!c.parametersPending());QVERIFY(c.scaleState().contains("失败"));
        c.disconnectDevice();c.device_=nullptr;
    }
};
QTEST_GUILESS_MAIN(ControllerTest)
#include "test_controller.moc"
