#include <iostream>

#include "sensorPacket.h"
#include "pointCloud.h"
#include "..\..\World\Objects\opoint.h"
#include "..\..\config.h"

SensorPacket::SensorPacket(PointCloud* cloud, OPoint** renderCloud, double x, double y, double theta, double timestamp) {
    this->cloud = cloud;
    this->renderCloud = renderCloud;
    this->x = x;
    this->y = y;
    this->theta = theta;
    this->timestamp = timestamp;
}

SensorPacket::~SensorPacket() {
    delete this->cloud;

    if(this->renderCloud != nullptr) {
        for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; i++) {
            delete this->renderCloud[i];
        }
        delete[] this->renderCloud;
    }
}