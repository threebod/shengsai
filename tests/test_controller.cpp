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
    void handshakeAndConfirmation(){
        Wire wire;Controller c;c.device_=&wire;c.opened();auto q=wire.last();QCOMPARE(int(q.command),int(SS_HELLO));QVERIFY(!c.connected());
        SsFrame reply{};reply.flags=SS_RESPONSE;reply.sequence=q.sequence;reply.command=SS_HELLO;reply.length=4;reply.data[1]=1;reply.data[3]=1;c.handle(reply);
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
    void failedSaveIsNotDisplayedAsSuccess(){
        Wire wire;Controller c;c.device_=&wire;c.connected_=true;c.applyScale(1.111,1.143,true);
        QVERIFY(c.parametersPending());auto q=wire.last();SsFrame r{};r.command=q.command;r.sequence=q.sequence;r.flags=SS_RESPONSE|SS_ERROR;r.length=1;r.data[0]=4;c.handle(r);
        QVERIFY(!c.parametersPending());QVERIFY(c.scaleState().contains("失败"));
        c.disconnectDevice();c.device_=nullptr;
    }
};
QTEST_GUILESS_MAIN(ControllerTest)
#include "test_controller.moc"
