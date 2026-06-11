#include "renderPacket.h"
#include "..\World\Objects\opoint.h"
#include "..\SLAMModels\MapRepresentation\poseRenderPacket.h"
#include "..\config.h"

RenderPacket::RenderPacket(double realX, double realY, double realTheta, OPoint** pointCloud, PoseRenderPacket* poses, PoseRenderPacket* extendedPoses, double neff) {
    this->realX = realX;
    this->realY = realY;
    this->realTheta = realTheta;

    this->pointCloud = pointCloud;
    this->poses = poses;
    this->extendedPoses = extendedPoses;
    
    this->neff = neff;
}

// does not currently delete grid
RenderPacket::~RenderPacket() {
    if(this->pointCloud != nullptr) {
        for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; i++) {
            delete this->pointCloud[i];
        }
        
        delete[] this->pointCloud;
    }

    delete this->poses;
}
