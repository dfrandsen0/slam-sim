#include <windows.h>
#include <iostream>
#include <thread>

#include "robotModel.h"
#include "Lidar\sensorModel.h"
#include "Lidar\pointCloud.h"
#include "Lidar\sensorPacket.h"
#include "motionModel.h"
#include "..\World\map.h"
#include "..\World\Objects\opoint.h"
#include "..\Utilities\utilities.h"
#include "..\Utilities\mathUtilities.h"
#include "..\config.h"

RobotModel::RobotModel() {
    this->sensorModel = new SensorModel();
    this->motionModel = new MotionModel();
}

RobotModel::~RobotModel() {
    delete this->motionModel;
    delete this->sensorModel;

    //Do not delete map; (still used and) deleted elsewhere
}

void RobotModel::InitializeRobot(Map* map) {
    this->map = map;
    this->sensorModel->GiveMap(map);
    this->motionModel->GiveMap(map);
    this->sensorModel->GiveMotionModel(this->motionModel);

    if(RANDOM_START) {
        int startIndex = Utilities::GetRandomInt(0, map->GetStartsSize() - 1);

        this->motionModel->SetStartPosition(
            map->GetStart(startIndex)->x,
            map->GetStart(startIndex)->y,
            (double)Utilities::GetRandomInt(0, 6)
        );
    } else {
        this->motionModel->SetStartPosition(
            map->GetStart(0)->x,
            map->GetStart(0)->y,
            0
        );
    }

    this->sensorModel->InitSensor();
}

// Rendering function
double RobotModel::GetRealX() {
    return this->motionModel->GetRealX();
}

// Rendering function
double RobotModel::GetRealY() {
    return this->motionModel->GetRealY();
}

// Rendering function
double RobotModel::GetRealTheta() {
    return this->motionModel->GetRealTheta();
}

void RobotModel::KickOffScan(double timestamp) {
    this->sensorModel->SetKickTimeStamp(timestamp);
    ReleaseSemaphore(this->sensorModel->GetSensorSemaphore(), 1, NULL);
}

PointCloud* RobotModel::CopyLatestScan(double& pointCloudTimestamp) {
    SensorPacket* lastPacket = this->sensorModel->GetLatestPacket();
    PointCloud* cloud = lastPacket->cloud;
    pointCloudTimestamp = lastPacket->timestamp;
    
    if(cloud == nullptr) {
        return nullptr;
    } else {
        return cloud->Copy();
    }
}

OPoint** RobotModel::CopyLatestRenderScan() {
    SensorPacket* packet = this->sensorModel->GetLatestPacket();

    if(packet->renderCloud == nullptr) {
        return nullptr;
    }

    OPoint** cloud = packet->renderCloud;
    OPoint** newCloud = new OPoint*[SENSOR_MODEL_POINTS_PER_SCAN];

    for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; i++) {
        if(cloud[i] == nullptr) {
            newCloud[i] = nullptr;
        } else {
            newCloud[i] = cloud[i]->Copy();
        }
    }

    return newCloud;
}

void RobotModel::CommandRobot(RobotCommand initialCommand, double& deltaDist, double& deltaTheta) {

    //if you return refined command, here's the place.

    //TODO
    //finds how much it "moved", then adds noise and sends it to robot to update

    double idealDist, idealTheta;
    this->motionModel->GetMovement(initialCommand, idealDist, idealTheta);
    double groundTruthDist = 0;
    double groundTruthTheta = 0;

    switch(initialCommand) {
        case RobotCommand::RIGHT:
        case RobotCommand::LEFT:
            groundTruthTheta = idealTheta + Utilities::GetFixedNoise(MOTION_MODEL_ROTATION_FIXED);
            deltaTheta = groundTruthTheta + Utilities::GetFixedNoise(MOTION_MODEL_ROTATION_FIXED);
            groundTruthDist = 0;
            deltaDist = 0;
            break;

        case RobotCommand::FORWARD:
            //TODO check for walls, and truncate forward/backward

            groundTruthTheta = idealTheta + Utilities::GetFixedNoise(MOTION_MODEL_FORWARD_ROTATION_DEVIATION);
            groundTruthDist = idealDist + Utilities::GetRandomNoise(idealDist, MOTION_MODEL_FORWARD_DEVIATION);

            deltaDist = groundTruthDist + Utilities::GetRandomNoise(groundTruthDist, MOTION_MODEL_FORWARD_DEVIATION);
            deltaTheta = groundTruthTheta + Utilities::GetFixedNoise(MOTION_MODEL_FORWARD_ROTATION_DEVIATION);
            break;

        case RobotCommand::STOP:
            if(idealDist == 0) {
                groundTruthDist = 0;
                groundTruthTheta = 0;
                deltaDist = 0;
                deltaTheta = 0;
                break;
            }
            groundTruthTheta = idealTheta + Utilities::GetFixedNoise(MOTION_MODEL_FORWARD_ROTATION_DEVIATION);
            deltaTheta = groundTruthTheta + Utilities::GetFixedNoise(MOTION_MODEL_FORWARD_ROTATION_DEVIATION);
            //TODO check for walls, and truncate forward/backward
            // (idk where that is, either here or below switch)

            groundTruthDist = idealDist + Utilities::GetRandomNoise(idealDist, MOTION_MODEL_FORWARD_DEVIATION);
            deltaDist = groundTruthDist + Utilities::GetRandomNoise(groundTruthDist, MOTION_MODEL_FORWARD_DEVIATION);
            break;
    }

    double changeX = groundTruthDist * cos(groundTruthTheta + this->motionModel->GetRealTheta());
    double changeY = groundTruthDist * sin(groundTruthTheta + this->motionModel->GetRealTheta());

    this->motionModel->UpdateRobotPosition(changeX, changeY, groundTruthTheta);
}