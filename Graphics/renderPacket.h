#ifndef GRAPHICS_RENDER_PACKET_H_
#define GRAPHICS_RENDER_PACKET_H_

#include "..\World\Objects\opoint.h"
#include "..\SLAMModels\MapRepresentation\poseRenderPacket.h"

struct RenderPacket {
public:
    double realX;
    double realY;
    double realTheta;

    OPoint** pointCloud;
    PoseRenderPacket* poses;
    PoseRenderPacket* extendedPoses;

    double neff;

    RenderPacket(double realX, double realY, double realTheta, OPoint** pointCloud, PoseRenderPacket* poses, PoseRenderPacket* extendedPoses, double neff);
    ~RenderPacket();
};

#endif