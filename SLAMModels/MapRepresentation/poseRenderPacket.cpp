#include <initializer_list>
#include <iostream>

#include "poseRenderPacket.h"
#include "..\..\config.h"

PoseRenderPacket::PoseRenderPacket(int maxPoses, int valueSetSize) {
    this->valueSetSize = valueSetSize;
    this->maxPoses = maxPoses;

    poses = new double[maxPoses*valueSetSize]();
}

PoseRenderPacket::~PoseRenderPacket() {
    delete[] poses;
}

void PoseRenderPacket::AddPose(std::initializer_list<double> values) {
    if(this->valueSetSize != ((int)(values.size()))) {
        std::cout << "wrong number of values passed to posepacket! panic!!" << std::endl;
        return;
    }

    for(double v : values) {
        this->poses[this->numValues] = v;
        (this->numValues)++;
    }

    (this->numPoses)++;
}

PoseRenderPacket* PoseRenderPacket::Copy() {
    PoseRenderPacket* newPacket = new PoseRenderPacket(this->maxPoses, this->valueSetSize);
    newPacket->numPoses = this->numPoses;
    newPacket->numValues = this->numValues;

    for(int i = 0; i < newPacket->numValues; i++) {
        newPacket->poses[i] = this->poses[i];
    }

    return newPacket;
}