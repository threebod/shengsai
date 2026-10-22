#pragma once
#include <QObject>
#include <QVariantList>
#include <QTimer>
#include <QElapsedTimer>
#include <QHash>
#include <QPointF>
#include <QIODevice>
#ifdef Q_OS_ANDROID
#include <QBluetoothSocket>
#include <QBluetoothDeviceDiscoveryAgent>
#else
#include <QSerialPort>
#endif
extern "C" {
#include "../shared/protocol.h"
}

class Controller : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool connected READ connected NOTIFY changed)
    Q_PROPERTY(bool busy READ busy NOTIFY changed)
    Q_PROPERTY(bool axis6Testing READ axis6Testing NOTIFY changed)
    Q_PROPERTY(QString axis6TestState READ axis6TestState NOTIFY changed)
    Q_PROPERTY(bool confirmed READ confirmed NOTIFY changed)
    Q_PROPERTY(bool mechanismValid READ mechanismValid NOTIFY changed)
    Q_PROPERTY(bool ringsRead READ ringsRead NOTIFY changed)
    Q_PROPERTY(bool calibrating READ calibrating NOTIFY changed)
    Q_PROPERTY(bool parametersPending READ parametersPending NOTIFY changed)
    Q_PROPERTY(QString status READ status NOTIFY changed)
    Q_PROPERTY(QString task READ task NOTIFY changed)
    Q_PROPERTY(QString scaleState READ scaleState NOTIFY changed)
    Q_PROPERTY(QString ringState READ ringState NOTIFY changed)
    Q_PROPERTY(QString connectionName READ connectionName NOTIFY changed)
    Q_PROPERTY(QVariantList devices READ devices NOTIFY changed)
    Q_PROPERTY(QVariantList rings READ rings NOTIFY changed)
    Q_PROPERTY(QVariantList obstacleRegions READ obstacleRegions CONSTANT)
    Q_PROPERTY(QVariantList servoAngles READ servoAngles NOTIFY changed)
    Q_PROPERTY(double forwardScale READ forwardScale NOTIFY changed)
    Q_PROPERTY(double rightScale READ rightScale NOTIFY changed)
public:
    explicit Controller(QObject *parent=nullptr);
    bool connected()const{return connected_;} bool busy()const{return busy_ || axis6Testing();}
    bool axis6Testing()const{return axis6TestSequence_>=0;}
    QString axis6TestState()const{return axis6TestState_;}
    bool confirmed()const{return confirmed_;} bool ringsRead()const{return ringsRead_;}
    bool mechanismValid()const{return mechanismValid_;}
    bool calibrating()const{return calibrationStage_!=0;}
    bool parametersPending()const;
    QString status()const{return status_;} QString task()const{return task_;}
    QString scaleState()const{return scaleState_;} QString ringState()const{return ringState_;}
    QString connectionName()const{return connectionName_;}
    QVariantList devices()const{return devices_;} QVariantList rings()const{return rings_;}
    QVariantList obstacleRegions()const;
    QVariantList servoAngles()const{return angles_;}
    double forwardScale()const{return forward_/1000.0;} double rightScale()const{return right_/1000.0;}
    Q_INVOKABLE void scan();
    Q_INVOKABLE void connectDevice(const QString &address);
    Q_INVOKABLE void disconnectDevice();
    Q_INVOKABLE void confirmZero();
    Q_INVOKABLE void stop(bool emergency=false);
    Q_INVOKABLE void pauseManual();
    Q_INVOKABLE void readRings();
    Q_INVOKABLE void saveRings(const QVariantList &values);
    Q_INVOKABLE void readScale();
    Q_INVOKABLE void applyScale(double forward,double right,bool save);
    Q_INVOKABLE void setServo(int channel,double angle,bool finalValue=false);
    Q_INVOKABLE void jogAxis(int motor,double mm);
    Q_INVOKABLE void testAxis6();
    Q_INVOKABLE void drive(int direction);
    Q_INVOKABLE void run(int type,int zone,int regions,int rpm=240);
    Q_INVOKABLE void runAction(int action);
    Q_INVOKABLE void calibrate();
    Q_INVOKABLE void alignRing();
signals:
    void changed();
    void lineReceived(QString line);
    void requestAccepted(int command);
private:
    friend class ControllerTest;
    struct Pending {int command; qint64 sent;};
    int request(int command,const QByteArray &data={});
    void opened();void ingest();void handle(const SsFrame &frame);
    void log(const QString &line);void failCalibration(const QString &reason);
    void sample();void calculate();void setStatus(const QString &text);
    void flushServos();
    static QByteArray pair(int a,int b);
    QIODevice *device_{};
#ifdef Q_OS_ANDROID
    QBluetoothSocket bluetooth_{QBluetoothServiceInfo::RfcommProtocol};
    QBluetoothDeviceDiscoveryAgent discovery_;
#else
    QSerialPort serial_;
#endif
    SsParser parser_{};QHash<int,Pending> pending_;int sequence_{};
    QTimer heartbeat_,watchdog_,servoTimer_,driveTimer_,calibrationTimeout_;
    QElapsedTimer clock_;qint64 lastRx_{};
    QHash<int,int> servoTargets_;
    bool connected_{},busy_{},confirmed_{},ringsRead_{},mechanismValid_{};
    int axis6TestSequence_{-1};
    QString axis6TestState_{QStringLiteral("尚未测试")};
    QString status_{QStringLiteral("未连接")},task_{QStringLiteral("待启动")};
    QString scaleState_{QStringLiteral("尚未读取主控比例")},ringState_{QStringLiteral("请先读取主控参数")},connectionName_;
    QVariantList devices_,rings_,angles_{0.0,0.0,0.0};
    int forward_{1111},right_{1143},driveDirection_{};
    int calibrationStage_{},sampleIndex_{},sampleId_{},calibrationGeneration_{};
    QPointF samples_[5];
};
