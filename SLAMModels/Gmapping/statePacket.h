#ifndef SLAM_MODELS_GMAPPING_STATE_PACKET_H_
#define SLAM_MODELS_GMAPPING_STATE_PACKET_H_

#include "..\..\Models\Lidar\pointCloud.h"
#include "particle.h"

struct StatePacket {
public:
    StatePacket();
    ~StatePacket();

    PointCloud* pointCloud = nullptr;
    Particle** particles = nullptr;

    double commandTimestamp = 0.0;
    double pointCloudTimestamp = 0.0;

    double expDist = 0.0;
    double expTheta = 0.0;

    void CopyInfo(Particle** particles);
};

#endif