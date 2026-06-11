#ifndef MODELS_LIDAR_SENSOR_PACKET_H_
#define MODELS_LIDAR_SENSOR_PACKET_H_

#include "pointCloud.h"
#include "..\..\World\Objects\opoint.h"

struct SensorPacket {
public:
    SensorPacket(PointCloud* cloud, OPoint** renderCloud, double x, double y, double theta, double timestamp);
    ~SensorPacket();

    PointCloud* cloud = nullptr;
    OPoint** renderCloud = nullptr;
    double x = 0;
    double y = 0;
    double theta = 0;
    double timestamp = 0;
};

#endif