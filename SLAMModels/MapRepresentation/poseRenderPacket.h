#ifndef SLAM_MODELS_MAP_REPRESENTATION_POSE_RENDER_PACKET_H_
#define SLAM_MODELS_MAP_REPRESENTATION_POSE_RENDER_PACKET_H_

#include <initializer_list>

struct PoseRenderPacket {
public:
    PoseRenderPacket(int maxPoses, int valueSetSize);
    ~PoseRenderPacket();

    double* poses = nullptr;
    int maxPoses = 1.0;

    int numPoses = 0.0;
    int valueSetSize = 1.0;
    int numValues = 0.0;

    void AddPose(std::initializer_list<double> values);
    PoseRenderPacket* Copy();
};

#endif