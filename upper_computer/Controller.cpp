#include "Controller.h"
#include <QCoreApplication>
#include <QRandomGenerator>
#include <QRegularExpression>
#include <QtMath>
#ifdef Q_OS_ANDROID
#include <QBluetoothPermission>
#include <QBluetoothUuid>
#else
#include <QSerialPortInfo>
#endif

Controller::Controller(QObject *parent):QObject(parent) {
    clock_.start();heartbeat_.setInterval(200);watchdog_.setInterval(100);
    servoTimer_.setInterval(100);driveTimer_.setInterval(100);
    calibrationTimeout_.setSingleShot(true);
    connect(&heartbeat_,&QTimer::timeout,this,[this]{if(connected_)request(SS_HEARTBEAT);});
    connect(&servoTimer_,&QTimer::timeout,this,&Controller::flushServos);
    connect(&driveTimer_,&QTimer::timeout,this,[this]{
        if(connected_)request(SS_DRIVE,QByteArray(1,char(driveDirection_)));
    });
    connect(&calibrationTimeout_,&QTimer::timeout,this,[this]{failCalibration(QStringLiteral("标定超时；未覆盖原比例"));});
    connect(&watchdog_,&QTimer::timeout,this,[this]{
        bool expired=false;
        for(const auto &p:pending_)if(clock_.elapsed()-p.sent>5000){expired=true;break;}
        if(device_ && device_->isOpen() && (expired || clock_.elapsed()-lastRx_>3000)) {
            stop();disconnectDevice();setStatus(QStringLiteral("主控应答超时；连接已关闭，请重新确认状态"));
        }
    });
#ifdef Q_OS_ANDROID
    device_=&bluetooth_;
    connect(&bluetooth_,&QBluetoothSocket::connected,this,&Controller::opened);
    connect(&bluetooth_,&QBluetoothSocket::disconnected,this,&Controller::disconnectDevice);
    connect(&bluetooth_,&QBluetoothSocket::readyRead,this,&Controller::ingest);
    connect(&bluetooth_,&QBluetoothSocket::errorOccurred,this,[this]{const auto error=bluetooth_.errorString();disconnectDevice();setStatus(error);});
    connect(&discovery_,&QBluetoothDeviceDiscoveryAgent::deviceDiscovered,this,[this](const QBluetoothDeviceInfo &d){
        devices_.append(QVariantMap{{"name",d.name()+" · "+d.address().toString()},{"address",d.address().toString()}});emit changed();
    });
    connect(&discovery_,&QBluetoothDeviceDiscoveryAgent::errorOccurred,this,[this]{setStatus(discovery_.errorString());});
#else
    device_=&serial_;
    connect(&serial_,&QSerialPort::readyRead,this,&Controller::ingest);
    connect(&serial_,&QSerialPort::errorOccurred,this,[this](QSerialPort::SerialPortError e){
        if(e==QSerialPort::ResourceError || e==QSerialPort::DeviceNotFoundError) {
            const auto message=serial_.errorString();disconnectDevice();setStatus(message);
        }
    });
#endif
    scan();
}
void Controller::setStatus(const QString &text){status_=text;emit changed();}
bool Controller::parametersPending()const{
    for(const auto &p:pending_)if(p.command>=SS_GET_RINGS && p.command<=SS_SAVE_SCALE)return true;
    return false;
}
QByteArray Controller::pair(int a,int b){QByteArray bytes(4,0);ss_put16(reinterpret_cast<uint8_t *>(bytes.data()),quint16(a));ss_put16(reinterpret_cast<uint8_t *>(bytes.data()+2),quint16(b));return bytes;}
void Controller::scan(){
    devices_.clear();emit changed();
#ifdef Q_OS_ANDROID
    QBluetoothPermission permission;permission.setCommunicationModes(QBluetoothPermission::Access);
    if(qApp->checkPermission(permission)==Qt::PermissionStatus::Undetermined){
        qApp->requestPermission(permission,this,[this](const QPermission &p){
            if(p.status()==Qt::PermissionStatus::Granted)scan();else setStatus(QStringLiteral("蓝牙权限被拒绝，请在系统设置中允许"));
        });return;
    }
    if(qApp->checkPermission(permission)!=Qt::PermissionStatus::Granted){setStatus(QStringLiteral("需要蓝牙扫描与连接权限"));return;}
    discovery_.stop();discovery_.start(QBluetoothDeviceDiscoveryAgent::ClassicMethod);
#else
    for(const auto &port:QSerialPortInfo::availablePorts())devices_.append(QVariantMap{{"name",port.portName()+" · "+port.description()},{"address",port.portName()}});
    emit changed();
#endif
}
void Controller::connectDevice(const QString &address){
    if(address.isEmpty()){setStatus(QStringLiteral("请先选择设备"));return;}
    disconnectDevice();connectionName_=address;setStatus(QStringLiteral("正在连接…"));
#ifdef Q_OS_ANDROID
    bluetooth_.connectToService(QBluetoothAddress(address),QBluetoothUuid(QBluetoothUuid::ServiceClassUuid::SerialPort));
#else
    serial_.setPortName(address);serial_.setBaudRate(115200);serial_.setDataBits(QSerialPort::Data8);
    serial_.setParity(QSerialPort::NoParity);serial_.setStopBits(QSerialPort::OneStop);serial_.setFlowControl(QSerialPort::NoFlowControl);
    if(serial_.open(QIODevice::ReadWrite))opened();else setStatus(serial_.errorString());
#endif
}
void Controller::opened(){
    parser_={};pending_.clear();lastRx_=clock_.elapsed();watchdog_.start();
    const quint32 session=QRandomGenerator::global()->generate();QByteArray data(4,0);ss_put32(reinterpret_cast<uint8_t *>(data.data()),session);
    request(SS_HELLO,data);
}
void Controller::disconnectDevice(){
    if(axis6Testing()){
        if(device_ && device_->isOpen())request(SS_STOP);
        axis6TestSequence_=-1;axis6TestState_=QStringLiteral("连接已断开，测试结果未知");
    }
    driveTimer_.stop();heartbeat_.stop();watchdog_.stop();servoTimer_.stop();servoTargets_.clear();
    calibrationTimeout_.stop();calibrationStage_=0;++calibrationGeneration_;
    connected_=confirmed_=ringsRead_=busy_=mechanismValid_=false;pending_.clear();parser_={};
    if(logsLoading_)logsState_=QStringLiteral("读取中断，已显示的日志仍保留；重连后可重新读取");
    logsLoading_=false;taskLogBytes_.clear();
    ringState_=QStringLiteral("断线后请重新读取");scaleState_=QStringLiteral("断线后请重新读取");
    if(device_ && device_->isOpen())device_->close();status_=QStringLiteral("未连接");emit changed();
}
int Controller::request(int command,const QByteArray &data){
    if(!device_ || !device_->isOpen() || data.size()>SS_MAX_PAYLOAD)return -1;
    if(pending_.size()>=64){setStatus(QStringLiteral("待确认指令过多，请停止并检查连接"));return -1;}
    do{sequence_=(sequence_+1)&255;}while(pending_.contains(sequence_));
    SsFrame f{};f.flags=SS_REQUEST;f.command=quint8(command);f.sequence=quint8(sequence_);f.length=quint16(data.size());
    memcpy(f.data,data.data(),size_t(data.size()));uint8_t bytes[SS_MAX_FRAME];unsigned n=ss_encode(&f,bytes);
    pending_.insert(sequence_,{command,clock_.elapsed()});
    if(device_->write(reinterpret_cast<const char *>(bytes),n)!=qint64(n)){
        pending_.remove(sequence_);setStatus(QStringLiteral("发送失败，请检查连接"));return -1;
    }
    emit changed();return sequence_;
}
void Controller::ingest(){
    const QByteArray bytes=device_->readAll();SsFrame frame;
    for(const char byte:bytes)if(ss_feed(&parser_,quint8(byte),&frame)){lastRx_=clock_.elapsed();handle(frame);}
}
void Controller::handle(const SsFrame &f){
    if(f.flags==SS_EVENT){
        if(f.command==SS_LOG){log(QString::fromUtf8(reinterpret_cast<const char *>(f.data),f.length));return;}
        if(f.command==SS_ACTION && f.length==1 && f.sequence==axis6TestSequence_){
            axis6TestSequence_=-1;
            if(f.data[0])mechanismValid_=false;
            axis6TestState_=f.data[0]?QStringLiteral("测试中止，请检查日志及机构状态"):QStringLiteral("往返测试完成，请查看日志");
            if(f.data[0])setStatus(axis6TestState_);else emit changed();
            return;
        }
        if(f.command==SS_STATE && f.length==21){
            busy_=f.data[1];confirmed_=f.data[8];mechanismValid_=f.data[3];
            const QStringList phases{QStringLiteral("待启动"),QStringLiteral("取料准备"),QStringLiteral("取料"),QStringLiteral("圆环对准"),QStringLiteral("暂存对准"),QStringLiteral("粗加工放置"),QStringLiteral("粗加工回收"),QStringLiteral("暂存放置")};
            task_=f.data[7]?QStringLiteral("比赛运行 · 航段 %1 · %2").arg(f.data[5]+1).arg(phases.value(f.data[4],QStringLiteral("执行中"))):f.data[6]?QStringLiteral("离线按钮启动已准备"):busy_?QStringLiteral("单项执行中"):QStringLiteral("待启动");
            if(f.data[2])status_=QStringLiteral("急停已锁定，需重新确认零点和安全条件");
            for(int i=0;i<3;++i)angles_[i]=ss_get16(f.data+13+i*2)/10.0;
            emit changed();return;
        }
        if(f.length && f.data[0])setStatus(QStringLiteral("操作中止，请检查主控日志并重新确认机构状态"));
        return;
    }
    if(!(f.flags&SS_RESPONSE) || !f.length || !pending_.contains(f.sequence) || pending_[f.sequence].command!=f.command)return;
    pending_.remove(f.sequence);
    if(f.data[0]){
        if(f.command==SS_GET_LOG){
            logsLoading_=false;taskLogBytes_.clear();
            logsState_=f.data[0]==5?QStringLiteral("日志已更新或被覆盖，请重新读取"):QStringLiteral("读取失败，请确认主控固件支持任务日志读取");
            emit changed();return;
        }
        if(f.command==SS_SET_RINGS)ringState_=QStringLiteral("固化失败，请检查主控状态后重试");
        if(f.command==SS_APPLY_SCALE || f.command==SS_SAVE_SCALE)scaleState_=QStringLiteral("写入失败，尚未获得主控确认");
        static const QStringList errors{QString(),QStringLiteral("参数或命令无效"),QStringLiteral("主控忙，请等待或停止"),QStringLiteral("急停锁定"),QStringLiteral("主控执行失败，请检查日志"),QStringLiteral("协议不匹配"),QStringLiteral("请先确认机构零点及安全条件")};
        const QString message=errors.value(f.data[0],QStringLiteral("主控拒绝指令"));
        if(f.command==SS_ACTION && f.sequence==axis6TestSequence_){
            axis6TestSequence_=-1;axis6TestState_=QStringLiteral("测试未启动：")+message;
        }
        if(calibrating())failCalibration(message);else setStatus(message);return;
    }
    switch(f.command){
    case SS_GET_LOG: {
        if(!logsLoading_)return;
        const quint32 task=f.length>=14?ss_get32(f.data+1):0;
        const quint32 next=f.length>=14?ss_get32(f.data+5):0;
        const quint32 end=f.length>=14?ss_get32(f.data+9):0;
        const int size=int(f.length)-14;
        if(f.length<14 || !task || f.data[13]>1 || next>end || size<0 ||
           next<quint32(size) || end-(next-quint32(size))>32768U ||
           (logTask_ && (task!=logTask_ || end!=logEnd_ || next!=logOffset_+quint32(size))) ||
           (!size && next!=end) || taskLogBytes_.size()+size>32768){
            logsLoading_=false;taskLogBytes_.clear();logsState_=QStringLiteral("日志应答无效，请重新读取");break;
        }
        logTask_=task;logOffset_=next;logEnd_=end;logTruncated_=logTruncated_ || f.data[13];
        taskLogBytes_.append(reinterpret_cast<const char *>(f.data+14),size);
        if(next<end){
            QByteArray data(12,0);auto *p=reinterpret_cast<uint8_t *>(data.data());
            ss_put32(p,task);ss_put32(p+4,next);ss_put32(p+8,end);
            if(request(SS_GET_LOG,data)<0){logsLoading_=false;logsState_=QStringLiteral("日志读取发送失败，请重试");}
        }else{
            logsLoading_=false;
            logsState_=logTruncated_?QStringLiteral("已读取最近任务日志（前部已覆盖，保留最新32KiB）"):QStringLiteral("已读取主控最近任务日志");
            emit taskLogReceived(taskLogBytes_.isEmpty()?QStringLiteral("主控尚无最近任务日志\n"):QString::fromUtf8(taskLogBytes_));
            taskLogBytes_.clear();
        }
        break;
    }
    case SS_HELLO:
        if(f.length!=4 || f.data[1]!=1 || f.data[3]!=1){disconnectDevice();setStatus(QStringLiteral("固件版本或协议不匹配"));return;}
        if(f.data[2]!=SS_ROUTE_REVISION){disconnectDevice();setStatus(QStringLiteral("请更新支持15区障碍选择的主控固件"));return;}
        connected_=true;heartbeat_.start();setStatus(QStringLiteral("已连接，请确认机构零点和安全条件"));readScale();break;
    case SS_CLEAR:confirmed_=mechanismValid_=true;setStatus(QStringLiteral("安全条件已确认，机构坐标设为物理零点"));break;
    case SS_ACTION:
        if(f.sequence==axis6TestSequence_)axis6TestState_=QStringLiteral("测试进行中 · 正向5mm后返回起点");
        else setStatus(QStringLiteral("主控已接受指令，执行状态以反馈为准"));break;
    case SS_GET_RINGS:
        if(f.length!=14)return;rings_.clear();
        for(int i=0;i<3;++i)rings_.append(QVariantMap{{"horizontal",qint16(ss_get16(f.data+2+i*4))/10.0},{"turret",ss_get16(f.data+4+i*4)/10.0}});
        ringsRead_=true;ringState_=f.data[1]?QStringLiteral("已读取主控固化参数"):QStringLiteral("已读取固件默认参数");break;
    case SS_SET_RINGS:ringState_=QStringLiteral("主控已确认固化成功");readRings();break;
    case SS_GET_SCALE:case SS_APPLY_SCALE:case SS_SAVE_SCALE:
        if(f.length!=6)return;forward_=ss_get16(f.data+2);right_=ss_get16(f.data+4);
        scaleState_=f.data[1]?QStringLiteral("与主控一致 · 已掉电保存"):QStringLiteral("与主控一致 · 本次上电有效");
        if(calibrationStage_==4){calibrationStage_=0;calibrationTimeout_.stop();setStatus(QStringLiteral("自动标定已应用；点击保存到主控可掉电保留"));}break;
    default:if(f.command!=SS_HEARTBEAT)setStatus(QStringLiteral("主控已接受指令，执行状态以反馈为准"));break;
    }
    emit requestAccepted(f.command);emit changed();
}
void Controller::confirmZero(){if(connected_)request(SS_CLEAR,QByteArray(1,1));}
void Controller::stop(bool emergency){
    driveTimer_.stop();servoTimer_.stop();servoTargets_.clear();
    calibrationTimeout_.stop();calibrationStage_=0;++calibrationGeneration_;
    if(axis6Testing())axis6TestState_=QStringLiteral("正在停止测试…");
    request(emergency?SS_ESTOP:SS_STOP);confirmed_=emergency?false:confirmed_;emit changed();
}
void Controller::pauseManual(){
    if(driveTimer_.isActive() || calibrating() || !servoTargets_.isEmpty() || axis6Testing())stop();
    servoTimer_.stop();servoTargets_.clear();
}
void Controller::readRings(){if(connected_)request(SS_GET_RINGS);}
void Controller::readTaskLog(){
    if(!connected_ || logsLoading_)return;
    taskLogBytes_.clear();logTask_=logOffset_=logEnd_=0;logTruncated_=false;
    logsLoading_=true;logsState_=QStringLiteral("正在读取主控最近任务日志…");
    if(request(SS_GET_LOG,QByteArray(12,0))<0){logsLoading_=false;logsState_=QStringLiteral("日志读取发送失败，请重试");}
    emit changed();
}
void Controller::saveRings(const QVariantList &values){
    if(!connected_ || !ringsRead_ || values.size()!=3 || busy_)return;
    QByteArray data(12,0);
    for(int i=0;i<3;++i){const auto map=values[i].toMap();int h=qRound(map["horizontal"].toDouble()*10),t=qRound(map["turret"].toDouble()*10);
        if(h< -1220 || h>850 || t<0 || t>3600){setStatus(QStringLiteral("圆环参数超出机械范围"));return;}
        ss_put16(reinterpret_cast<uint8_t *>(data.data()+i*4),quint16(h));ss_put16(reinterpret_cast<uint8_t *>(data.data()+i*4+2),quint16(t));
    }
    ringState_=QStringLiteral("等待主控固化确认");request(SS_SET_RINGS,data);emit changed();
}
void Controller::readScale(){if(connected_)request(SS_GET_SCALE);}
void Controller::applyScale(double f,double r,bool save){
    if(!connected_ || busy_)return;
    if(!qIsFinite(f) || !qIsFinite(r) || f<0.05 || f>2 || r<0.05 || r>2){setStatus(QStringLiteral("比例范围为 0.050～2.000 mm/像素"));return;}
    const int a=qRound(f*1000),b=qRound(r*1000);
    scaleState_=QStringLiteral("等待主控确认");request(save?SS_SAVE_SCALE:SS_APPLY_SCALE,pair(a,b));emit changed();
}
void Controller::setServo(int channel,double angle,bool finalValue){
    if(!connected_ || !confirmed_ || calibrating() || channel<2 || channel>4 || !qIsFinite(angle))return;
    const int value=qRound(angle*10);if(value<0 || value>(channel==4?3600:2700))return;
    servoTargets_[channel]=value;servoTimer_.start();if(finalValue)flushServos();
}
void Controller::flushServos(){
    if(!connected_ || !confirmed_){servoTargets_.clear();servoTimer_.stop();return;}
    const auto targets=servoTargets_;servoTargets_.clear();
    for(auto it=targets.begin();it!=targets.end();++it){QByteArray data(3,0);data[0]=char(it.key());ss_put16(reinterpret_cast<uint8_t *>(data.data()+1),quint16(it.value()));request(SS_SERVO,data);}
    if(servoTargets_.isEmpty())servoTimer_.stop();
}
void Controller::jogAxis(int motor,double mm){
    if(!connected_ || !confirmed_ || busy_ || (motor!=5 && motor!=6) || !qIsFinite(mm))return;
    const int d=qRound(mm*10);if(!d || qAbs(d)>50)return;QByteArray data(3,0);data[0]=char(motor);ss_put16(reinterpret_cast<uint8_t *>(data.data()+1),quint16(d));request(SS_AUX,data);
}
void Controller::testAxis6(){
    if(!connected_ || !confirmed_ || !mechanismValid_ || busy() || calibrating() || !servoTargets_.isEmpty())return;
    const int sequence=request(SS_ACTION,QByteArray(1,char(SS_ACTION_AXIS6_TEST)));
    if(sequence<0)return;
    axis6TestSequence_=sequence;axis6TestState_=QStringLiteral("等待主控开始测试");emit changed();
}
void Controller::drive(int direction){
    if(!connected_ || !confirmed_ || calibrating() || (direction!=1 && direction!=-1))return;
    driveDirection_=direction;request(SS_DRIVE,QByteArray(1,char(direction)));driveTimer_.start();
}
QVariantList Controller::obstacleRegions()const{
    QVariantList values;
    for(const auto &r:ssObstacleRegions)values.append(QVariantMap{{"x0",r.x0},{"y0",r.y0},{"x1",r.x1},{"y1",r.y1}});
    return values;
}
void Controller::run(int type,int zone,int regions,int rpm){
    if(!connected_ || !confirmed_ || busy() || calibrating())return;
    if(type<0 || type>2 || zone<1 || zone>2 || regions<0 || regions>0x7fff ||
       !ss_obstacle_regions_valid(quint16(regions)) || rpm<10 || rpm>250)return;
    for(const auto &p:pending_)if(p.command>=SS_PREPARE && p.command<=SS_ROUTE)return;
    QByteArray data(5,0);data[0]=char(zone);ss_put16(reinterpret_cast<uint8_t *>(data.data()+1),quint16(regions));ss_put16(reinterpret_cast<uint8_t *>(data.data()+3),quint16(rpm));
    request(type==0?SS_PREPARE:type==1?SS_MISSION:SS_ROUTE,data);
}
void Controller::runAction(int action){if(connected_ && confirmed_ && !busy_ && action>=0 && action<3)request(SS_ACTION,QByteArray(1,char(action)));}
void Controller::alignRing(){if(connected_ && confirmed_ && !busy_)request(SS_ALIGN);}
void Controller::sample(){
    calibrationStage_=1;if(++sampleId_>65535)sampleId_=1;
    QByteArray data(2,0);ss_put16(reinterpret_cast<uint8_t *>(data.data()),quint16(sampleId_));
    calibrationTimeout_.start(7000);request(SS_SAMPLE,data);setStatus(QStringLiteral("标定：读取稳定样本 %1/5").arg(sampleIndex_+1));
}
void Controller::calibrate(){
    if(!connected_ || !confirmed_ || busy_ || calibrating())return;
    sampleIndex_=0;++calibrationGeneration_;sample();
}
void Controller::failCalibration(const QString &reason){
    if(!calibrating())return;stop();setStatus(reason);
}
void Controller::log(const QString &line){
    emit lineReceived(line);
    if(line.startsWith("ERR") || line.startsWith("VISION ERROR")){
        if(calibrating())failCalibration(line);else setStatus(line);return;
    }
    if(calibrationStage_==1){
        const QRegularExpression re("^VISION MEASURE id=(\\d+) ring=2 token=(\\d+) du=(-?\\d+) dv=(-?\\d+) quality=(\\d+)$");
        const auto m=re.match(line);if(!m.hasMatch() || m.captured(1).toInt()!=sampleId_)return;
        const int du=m.captured(3).toInt(),dv=m.captured(4).toInt();
        if(m.captured(5).toInt()<60 || du< -160 || du>159 || dv< -120 || dv>119){failCalibration(QStringLiteral("样本质量不足；未覆盖原比例"));return;}
        calibrationTimeout_.stop();samples_[sampleIndex_]=QPointF(du,dv);
        if(sampleIndex_==4){calculate();return;}
        static const int forward[4]={20,-20,0,0},right[4]={0,0,20,-20};
        calibrationStage_=2;calibrationTimeout_.start(11000);request(SS_VISION_JOG,pair(forward[sampleIndex_],right[sampleIndex_]));
    }else if(calibrationStage_==2 && line=="VISION PAUSED"){
        calibrationStage_=3;calibrationTimeout_.stop();const int generation=calibrationGeneration_;
        QTimer::singleShot(300,this,[this,generation]{if(generation==calibrationGeneration_ && calibrationStage_==3){++sampleIndex_;sample();}});
    }
}
void Controller::calculate(){
    const QPointF f1=samples_[1]-samples_[0],f2=samples_[1]-samples_[2],r1=samples_[3]-samples_[2],r2=samples_[3]-samples_[4];
    const double f=(f1.x()+f2.x())/2,r=(r1.y()+r2.y())/2;
    const QPointF fd=samples_[2]-samples_[0],rd=samples_[4]-samples_[2];
    if(f<5 || r<5 || f1.x()<=0 || f2.x()<=0 || r1.y()<=0 || r2.y()<=0 ||
       qAbs(fd.x())>3 || qAbs(fd.y())>3 || qAbs(rd.x())>3 || qAbs(rd.y())>3 ||
       qAbs((f1.y()+f2.y())/2)>qMax(3.,f*.2) || qAbs((r1.x()+r2.x())/2)>qMax(3.,r*.2)){
        failCalibration(QStringLiteral("往返偏差或轴间耦合过大；未覆盖原比例"));return;
    }
    const int a=qRound(20000/f),b=qRound(20000/r);
    if(a<50 || a>2000 || b<50 || b>2000){failCalibration(QStringLiteral("计算比例超出有效范围"));return;}
    calibrationStage_=4;calibrationTimeout_.start(6000);request(SS_APPLY_SCALE,pair(a,b));
}
