#pragma once
#include <QObject>
#include <QVariantList>
#include <QTimer>
#include <QElapsedTimer>
#include <QHash>
#include <QPointF>
#include <QIODevice>
#include "../shared/lidar.h"
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
    Q_PROPERTY(bool turnTesting READ turnTesting NOTIFY changed)
    Q_PROPERTY(QString turnTestState READ turnTestState NOTIFY changed)
    Q_PROPERTY(bool straightTesting READ straightTesting NOTIFY changed)
    Q_PROPERTY(QString straightTestState READ straightTestState NOTIFY changed)
    Q_PROPERTY(bool lateralTesting READ lateralTesting NOTIFY changed)
    Q_PROPERTY(QString lateralTestState READ lateralTestState NOTIFY changed)
    Q_PROPERTY(bool confirmed READ confirmed NOTIFY changed)
    Q_PROPERTY(bool mechanismValid READ mechanismValid NOTIFY changed)
    Q_PROPERTY(bool ringsRead READ ringsRead NOTIFY changed)
    Q_PROPERTY(bool pickupRead READ pickupRead NOTIFY changed)
    Q_PROPERTY(QVariantList pickup READ pickup NOTIFY changed)
    Q_PROPERTY(QString pickupState READ pickupState NOTIFY changed)
    Q_PROPERTY(bool heightsRead READ heightsRead NOTIFY changed)
    Q_PROPERTY(QVariantList placementHeights READ placementHeights NOTIFY changed)
    Q_PROPERTY(QString heightsState READ heightsState NOTIFY changed)
    Q_PROPERTY(bool calibrating READ calibrating NOTIFY changed)
    Q_PROPERTY(bool parametersPending READ parametersPending NOTIFY changed)
    Q_PROPERTY(QString status READ status NOTIFY changed)
    Q_PROPERTY(QString task READ task NOTIFY changed)
    Q_PROPERTY(QString scaleState READ scaleState NOTIFY changed)
    Q_PROPERTY(QString ringState READ ringState NOTIFY changed)
    Q_PROPERTY(QString connectionName READ connectionName NOTIFY changed)
    Q_PROPERTY(bool logsLoading READ logsLoading NOTIFY changed)
    Q_PROPERTY(QString logsState READ logsState NOTIFY changed)
    Q_PROPERTY(QVariantList devices READ devices NOTIFY changed)
    Q_PROPERTY(QVariantList rings READ rings NOTIFY changed)
    Q_PROPERTY(QVariantList obstacleRegions READ obstacleRegions CONSTANT)
    Q_PROPERTY(QVariantList servoAngles READ servoAngles NOTIFY changed)
    Q_PROPERTY(double forwardScale READ forwardScale NOTIFY changed)
    Q_PROPERTY(double rightScale READ rightScale NOTIFY changed)
    Q_PROPERTY(bool lidarRunning READ lidarRunning NOTIFY lidarChanged)
    Q_PROPERTY(bool lidarPending READ lidarPending NOTIFY lidarChanged)
    Q_PROPERTY(bool lidarFresh READ lidarFresh NOTIFY lidarChanged)
    Q_PROPERTY(QString lidarStatus READ lidarStatus NOTIFY lidarChanged)
    Q_PROPERTY(QVariantList lidarPoints READ lidarPoints NOTIFY lidarPointsChanged)
    Q_PROPERTY(int lidarPeriod READ lidarPeriod NOTIFY lidarChanged)
    Q_PROPERTY(int lidarErrors READ lidarErrors NOTIFY lidarChanged)
    Q_PROPERTY(int lidarAge READ lidarAge NOTIFY lidarChanged)
public:
    explicit Controller(QObject *parent=nullptr);
    bool connected()const{return connected_;} bool busy()const{return busy_ || axis6Testing() || lateralTesting() || straightTesting() || turnTesting();}
    bool axis6Testing()const{return axis6TestSequence_>=0;}
    QString axis6TestState()const{return axis6TestState_;}
    bool turnTesting()const{return turnTestSequence_>=0;}
    QString turnTestState()const{return turnTestState_;}
    bool straightTesting()const{return straightTestSequence_>=0;}
    QString straightTestState()const{return straightTestState_;}
    bool lateralTesting()const{return lateralTestSequence_>=0;}
    QString lateralTestState()const{return lateralTestState_;}
    bool confirmed()const{return confirmed_;} bool ringsRead()const{return ringsRead_;}
    bool pickupRead()const{return pickupRead_;}
    QVariantList pickup()const{return pickup_;}
    QString pickupState()const{return pickupState_;}
    bool heightsRead()const{return heightsRead_;}
    QVariantList placementHeights()const{return heights_;}
    QString heightsState()const{return heightsState_;}
    bool mechanismValid()const{return mechanismValid_;}
    bool calibrating()const{return calibrationStage_!=0;}
    bool parametersPending()const;
    QString status()const{return status_;} QString task()const{return task_;}
    QString scaleState()const{return scaleState_;} QString ringState()const{return ringState_;}
    QString connectionName()const{return connectionName_;}
    bool logsLoading()const{return logsLoading_;}
    QString logsState()const{return logsState_;}
    QVariantList devices()const{return devices_;} QVariantList rings()const{return rings_;}
    QVariantList obstacleRegions()const;
    QVariantList servoAngles()const{return angles_;}
    double forwardScale()const{return forward_/1000.0;} double rightScale()const{return right_/1000.0;}
    bool lidarRunning()const{return lidarRunning_;}
    bool lidarPending()const{return lidarPending_;}
    bool lidarFresh()const{return lidarRunning_ && lidarLastScan_>=0 && clock_.elapsed()-lidarLastScan_<=2500;}
    QString lidarStatus()const{return lidarStatus_;}
    QVariantList lidarPoints()const{return lidarPoints_;}
    int lidarPeriod()const{return lidarPeriod_;}
    int lidarErrors()const{return lidarErrors_;}
    int lidarAge()const{return lidarLastScan_<0?-1:int(clock_.elapsed()-lidarLastScan_);}
    Q_INVOKABLE void setLidarScanning(bool enabled);
    Q_INVOKABLE void scan();
    Q_INVOKABLE void connectDevice(const QString &address);
    Q_INVOKABLE void disconnectDevice();
    Q_INVOKABLE void confirmZero();
    Q_INVOKABLE void stop(bool emergency=false);
    Q_INVOKABLE void pauseManual();
    Q_INVOKABLE void readRings();
    Q_INVOKABLE void readTaskLog();
    Q_INVOKABLE void saveRings(const QVariantList &values);
    Q_INVOKABLE void readPickup();
    Q_INVOKABLE void savePickup(const QVariantList &values);
    Q_INVOKABLE void readHeights();
    Q_INVOKABLE void saveHeights(const QVariantList &values);
    Q_INVOKABLE void readScale();
    Q_INVOKABLE void applyScale(double forward,double right,bool save);
    Q_INVOKABLE void setServo(int channel,double angle,bool finalValue=false);
    Q_INVOKABLE void jogAxis(int motor,double mm);
    Q_INVOKABLE void testAxis6();
    Q_INVOKABLE void testLateral(int direction,double distanceMm);
    Q_INVOKABLE void testStraight();
    Q_INVOKABLE void testTurn(int direction);
    Q_INVOKABLE void drive(int direction);
    Q_INVOKABLE void run(int type,int zone,int regions,int rpm=240);
    Q_INVOKABLE void runAction(int action);
    Q_INVOKABLE void calibrate();
    Q_INVOKABLE void alignRing();
signals:
    void changed();
    void lineReceived(QString line);
    void taskLogReceived(QString text);
    void requestAccepted(int command);
    void lidarChanged();
    void lidarPointsChanged();
private:
    friend class ControllerTest;
    friend class LayoutTest;
    struct Pending {int command; qint64 sent;};
    int request(int command,const QByteArray &data={});
    void opened();void ingest();void handle(const SsFrame &frame);
    void log(const QString &line);void failCalibration(const QString &reason);
    void sample();void calculate();void setStatus(const QString &text);
    void flushServos();
    void handleLidarScan(const SsFrame &frame);
    void resetLidar();
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
    bool pickupRead_{};
    QVariantList pickup_;
    QString pickupState_{QStringLiteral("请先读取取料区参数")};
    bool heightsRead_{};
    bool logsLoading_{};
    QByteArray taskLogBytes_;
    quint32 logTask_{},logOffset_{},logEnd_{};
    bool logTruncated_{};
    QString logsState_{QStringLiteral("连接后可读取主控最近任务日志；断电或重启清空")};
    QVariantList heights_;
    QString heightsState_{QStringLiteral("请先读取主控放置深度")};
    int turnTestSequence_{-1};
    QString turnTestState_{QStringLiteral("尚未测试")};
    int straightTestSequence_{-1};
    QString straightTestState_{QStringLiteral("尚未测试")};
    int axis6TestSequence_{-1};
    int lateralTestSequence_{-1},lateralTestDistance_{};
    QString lateralTestState_{QStringLiteral("尚未测试")};
    QString axis6TestState_{QStringLiteral("尚未测试")};
    QString status_{QStringLiteral("未连接")},task_{QStringLiteral("待启动")};
    QString scaleState_{QStringLiteral("尚未读取主控比例")},ringState_{QStringLiteral("请先读取主控参数")},connectionName_;
    QVariantList devices_,rings_,angles_{0.0,0.0,0.0};
    int forward_{1111},right_{1143},driveDirection_{};
    int calibrationStage_{},sampleIndex_{},sampleId_{},calibrationGeneration_{};
    QPointF samples_[5];
    bool lidarRunning_{},lidarPending_{};
    quint16 lidarToken_{},lidarScanId_{},lidarTotal_{};
    int lidarPeriod_{},lidarErrors_{};
    qint64 lidarLastScan_{-1},lidarWaitAt_{};
    QVariantList lidarPoints_,lidarPartial_;
    QString lidarStatus_{QStringLiteral("连接主控后可启动雷达扫描")};
};
