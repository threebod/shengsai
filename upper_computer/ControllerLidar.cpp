#include "Controller.h"
#include <QtMath>

void Controller::resetLidar()
{
    lidarRunning_=lidarPending_=false;
    ++lidarToken_;lidarTotal_=0;lidarLastScan_=-1;
    lidarPartial_.clear();lidarPoints_.clear();lidarPeriod_=lidarErrors_=0;
    lidarStatus_=QStringLiteral("连接主控后可启动雷达扫描");
    emit lidarPointsChanged();
    emit lidarChanged();
}

void Controller::setLidarScanning(bool enabled)
{
    if(!connected_ || lidarPending_)return;
    ++lidarToken_;if(!lidarToken_)++lidarToken_;
    QByteArray data(3,0);data[0]=enabled?1:0;
    ss_put16(reinterpret_cast<uint8_t *>(data.data()+1),lidarToken_);
    lidarPartial_.clear();lidarTotal_=0;
    lidarPending_=true;
    if(enabled){lidarPoints_.clear();lidarLastScan_=-1;emit lidarPointsChanged();}
    lidarStatus_=enabled?QStringLiteral("等待主控接受扫描请求…"):QStringLiteral("等待主控停止雷达…");
    if(request(SS_LIDAR_CONTROL,data)<0){lidarPending_=false;lidarStatus_=QStringLiteral("雷达指令发送失败");}
    emit lidarChanged();
}

void Controller::handleLidarScan(const SsFrame &frame)
{
    if(!connected_ || !lidarRunning_ || lidarPending_ ||
       !ss_lidar_chunk_valid(frame.data,frame.length) || ss_get16(frame.data)!=lidarToken_)return;
    const int scan=ss_get16(frame.data+2),offset=ss_get16(frame.data+4),total=ss_get16(frame.data+6);
    const int period=ss_get16(frame.data+8);
    if(period<50 || period>500)return;
    if(!offset){lidarPartial_.clear();lidarScanId_=quint16(scan);lidarTotal_=quint16(total);}
    if(scan!=lidarScanId_ || total!=lidarTotal_ || offset!=lidarPartial_.size()){
        lidarPartial_.clear();lidarTotal_=0;return;
    }
    for(int at=SS_LIDAR_HEADER;at<frame.length;at+=SS_LIDAR_POINT_BYTES){
        const int angle=ss_get16(frame.data+at),packed=ss_get16(frame.data+at+2),distance=packed>>2;
        const double radians=qDegreesToRadians(angle/64.0);
        lidarPartial_.append(QVariantMap{{"angle",angle/64.0},{"distance",distance},
            {"intensity",frame.data[at+4]},{"flag",packed&3},
            {"x",distance*qSin(radians)},{"y",distance*qCos(radians)}});
    }
    if(lidarPartial_.size()!=total)return;
    const int age=ss_get16(frame.data+12);
    if(age>2000){lidarPartial_.clear();lidarTotal_=0;return;}
    lidarPoints_=lidarPartial_;lidarPartial_.clear();lidarTotal_=0;
    lidarPeriod_=period;lidarErrors_=ss_get16(frame.data+10);
    lidarLastScan_=clock_.elapsed()-age;
    lidarStatus_=QStringLiteral("已收到完整扫描 · 显示约1圈/秒");
    emit lidarPointsChanged();
    emit lidarChanged();
}
