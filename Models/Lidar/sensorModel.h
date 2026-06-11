#ifndef MODELS_LIDAR_SENSOR_MODEL_H_
#define MODELS_LIDAR_SENSOR_MODEL_H_

// Modelled off SlamTec RP LiDAR-A1
// https://www.slamtec.com/en/Lidar/A1Spec

#include <windows.h>
#include <thread>
#include <mutex>
#include <atomic>

#include "pointCloud.h"
#include "sensorPacket.h"
#include "..\..\World\map.h"
#include "..\..\World\Objects\opoint.h"
#include "..\motionModel.h"

class SensorModel {
public:
private:
    Map* map = nullptr;
    MotionModel* motionModel = nullptr;

    HANDLE sensorSemaphore;
    std::thread sensorThread;
    std::atomic<bool> destroyFlag = false;
    
    std::mutex guardTimestamp;
    double kickTimeStamp = 0.0;

    std::mutex guardPacketHistory;
    SensorPacket** packetHistory = nullptr;
public:
    SensorModel();
    ~SensorModel();

    void InitSensor();

    void GiveMap(Map* map);
    void GiveMotionModel(MotionModel* motionModel);
    void MainScanLoop();

    SensorPacket* GetScan(double currX, double currY, double currTheta);

    double GetKickTimeStamp();
    void SetKickTimeStamp(double timestamp);

    HANDLE GetSensorSemaphore();

    SensorPacket* GetLatestPacket();
    void AddPacket(SensorPacket* newPacket);

private:
    double GetCollisionDistance(double x, double y, double theta, double& dx, double& dy);
    void PrintRenderCloud(SensorPacket* packet);
};

#endif