#include <iostream>

#include "pointCloud.h"
#include "polarPoint.h"
#include "..\..\config.h"

PointCloud::PointCloud() {
    this->cloud = new PolarPoint*[SENSOR_MODEL_POINTS_PER_SCAN];
    this->cloudSize = 0;
}

PointCloud::~PointCloud() {
    if(this->cloud != nullptr) {
        for(int i = 0; i < this->cloudSize; i++) {
            delete this->cloud[i];
        }
        delete[] this->cloud;
    }
}

void PointCloud::Add(double range, double theta) {
    this->cloud[this->cloudSize] = new PolarPoint(range, theta);
    (this->cloudSize)++;
}

void PointCloud::Print() {
    std::cout << "Point Cloud of " << this->cloudSize << std::endl;
    for(int i = 0; i < this->cloudSize; i++) {
        this->cloud[i]->Print();
        std::cout << std::endl;
    }
}

PointCloud* PointCloud::Copy() {
    PointCloud* newPointCloud = new PointCloud();
    newPointCloud->cloudSize = this->cloudSize;

    PolarPoint** newCloud = new PolarPoint*[SENSOR_MODEL_POINTS_PER_SCAN];
    newPointCloud->cloud = newCloud;

    for(int i = 0; i < this->cloudSize; i++) {
        if(this->cloud[i] == nullptr) {
            break;
        }
        newCloud[i] = this->cloud[i]->Copy();
    }
    return newPointCloud;
}