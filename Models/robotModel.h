#ifndef MODELS_ROBOT_MODEL_H_
#define MODELS_ROBOT_MODEL_H_

#include <windows.h>
#include <thread>

#include "..\World\map.h"
#include "Lidar\sensorModel.h"
#include "Lidar\pointCloud.h"
#include "..\World\Objects\opoint.h"
#include "motionModel.h"
#include "..\config.h"

class RobotModel {
public:
private:
    MotionModel* motionModel = nullptr;
    SensorModel* sensorModel = nullptr;

    // map solely for simulation purposes
    Map* map = nullptr;
    
public:
    RobotModel();
    ~RobotModel();

    void InitializeRobot(Map* map);

    double GetRealX();
    double GetRealY();
    double GetRealTheta();

    void KickOffScan(double timestamp);
    PointCloud* CopyLatestScan(double& pointCloudTimestamp);
    OPoint** CopyLatestRenderScan();

    void CommandRobot(RobotCommand initialCommand, double& deltaDist, double& deltaTheta);

private:

};

#endif