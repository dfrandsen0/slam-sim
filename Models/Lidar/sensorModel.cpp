#include <windows.h>
#include <numbers>
#include <cmath>
#include <cfloat>
#include <iostream>
#include <thread>
#include <mutex>

#include "..\..\World\map.h"
#include "..\..\World\Objects\opoint.h"
#include "sensorModel.h"
#include "pointCloud.h"
#include "sensorPacket.h"
#include "..\..\Utilities\mathUtilities.h"
#include "..\..\Utilities\utilities.h"
#include "..\..\config.h"

SensorModel::SensorModel() {

}

//TODO stop thread before destroying
SensorModel::~SensorModel() {
    //do not delete map here.

    //TODO destroy thread
}

void SensorModel::InitSensor() {
    this->packetHistory = new SensorPacket*[SENSOR_NUM_TRACKED_PACKETS];
    for(int i = 0; i < SENSOR_NUM_TRACKED_PACKETS; i++) {
        this->packetHistory[i] = nullptr;
    }
    
    this->packetHistory[0] = new SensorPacket(nullptr, nullptr, this->motionModel->GetRealX(), this->motionModel->GetRealY(), this->motionModel->GetRealTheta(), 0);

    this->sensorSemaphore = CreateSemaphore(NULL, 0, 1, NULL);
    this->sensorThread = std::thread(MainScanLoop, this);
}

void SensorModel::GiveMap(Map* map) {
    this->map = map;
}

void SensorModel::GiveMotionModel(MotionModel* motionModel) {
    this->motionModel = motionModel;
}

void SensorModel::MainScanLoop() {
    //TODO add flag to stop for destruction
    for(;;) {
        WaitForSingleObject(this->sensorSemaphore, INFINITE);
        if(this->destroyFlag) {
            break;
        }

        SensorPacket* newPacket = this->GetScan(this->motionModel->GetRealX(), this->motionModel->GetRealY(), this->motionModel->GetRealTheta());

        this->AddPacket(newPacket);
    }
}

// for now, sensor model isn't accurate; doesn't account for acceleration/deceleration. May come back to this
// acceleration and deceleration difference turn into noise.
// with default values, that's at most 2.025 mm, and an average of 1.35 mm.
SensorPacket* SensorModel::GetScan(double currX, double currY, double currTheta) {
    PointCloud* pointCloud = new PointCloud();
    OPoint** renderCloud = new OPoint*[SENSOR_MODEL_POINTS_PER_SCAN];
    for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; i++) {
        renderCloud[i] = nullptr;
    }
    int currRenderPoints = 0;

    double pi = MathUtilities::PI;

    SensorPacket* lastPacket = this->GetLatestPacket();

    double deltaX = (currX - lastPacket->x) / SENSOR_MODEL_POINTS_PER_SCAN;
    double deltaY = (currY - lastPacket->y) / SENSOR_MODEL_POINTS_PER_SCAN;
    double deltaTheta = (currTheta - lastPacket->theta) / SENSOR_MODEL_POINTS_PER_SCAN;
    double deltaRadian = ((pi*2) / SENSOR_MODEL_POINTS_PER_SCAN);

    double startX = lastPacket->x + (deltaX / 2.0);
    double startY = lastPacket->y + (deltaY / 2.0);
    double startTheta = lastPacket->theta + (deltaTheta / 2.0);
    double startRadian = (pi*2) - (deltaRadian / 2.0);

    double range, dx, dy, resolution;

    for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; i++) {
        range = this->GetCollisionDistance(startX, startY, startTheta + startRadian, dx, dy);
        if(range < SENSOR_MODEL_MEASURE_MIN || range > SENSOR_MODEL_MEASURE_MAX) {
            // out of range
            continue;
        }

        for(int tier = 0; tier < SENSOR_MODEL_ACCURACY_TIERS; tier++) {
            if(range < SENSOR_MODEL_ACCURACY[tier].second) {
                range += Utilities::GetRandomNoise(range, SENSOR_MODEL_ACCURACY[tier].first);
                break;
            }
        }

        resolution = range * SENSOR_MODEL_RANGE_RESOLUTION;
        range = std::round(range / resolution) * resolution;

        pointCloud->Add(range, startRadian);

        // For rendering purposes
        renderCloud[currRenderPoints] = new OPoint(
            startX + (dx * range),
            startY + (dy * range)
        );
        (currRenderPoints)++;

        startX += deltaX;
        startY += deltaY;
        startTheta += deltaTheta;
        startRadian -= deltaRadian;
    }

    SensorPacket* newPacket = new SensorPacket(pointCloud, renderCloud, currX, currY, currTheta, this->kickTimeStamp);

    return newPacket;
}

double SensorModel::GetKickTimeStamp() {
    std::lock_guard<std::mutex> lock(this->guardTimestamp);
    return this->kickTimeStamp;
}

void SensorModel::SetKickTimeStamp(double timestamp) {
    std::lock_guard<std::mutex> lock(this->guardTimestamp);
    this->kickTimeStamp = timestamp;
}

HANDLE SensorModel::GetSensorSemaphore() {
    return this->sensorSemaphore;
}

SensorPacket* SensorModel::GetLatestPacket() {
    std::lock_guard<std::mutex> lock(this->guardPacketHistory);
    return this->packetHistory[0];
}

void SensorModel::AddPacket(SensorPacket* newPacket) {
    std::lock_guard<std::mutex> lock(this->guardPacketHistory);

    delete this->packetHistory[SENSOR_NUM_TRACKED_PACKETS - 1];

    for(int i = SENSOR_NUM_TRACKED_PACKETS - 1; i > 0; i--) {
        this->packetHistory[i] = this->packetHistory[i - 1];
    }

    this->packetHistory[0] = newPacket;
}

double SensorModel::GetCollisionDistance(double x, double y, double theta, double& dx, double& dy) {
    double minDistance = FLT_MAX;
    OLine** lines = this->map->GetLines();

    double d; //discriminate
    double u; // partial derivative in respect to line length? must be 0 <= u <= 1
             // ...idk it's the percent of the line between intersection point and each end.
             // I didn't take calc 3. ¯\_(ツ)_/¯
    double t; // distance to intersection point

    double ax; // point ax
    double ay; // point ay
    double bx; // point bx. a/b can be switched with no difference
    double by; // point by

    dx = cos(theta); // direction x
    dy = sin(theta); // direction y

    for(int i = 0; i < this->map->GetLinesSize(); i++) {
        ax = lines[i]->point1->x;
        ay = lines[i]->point1->y;
        bx = lines[i]->point2->x;
        by = lines[i]->point2->y;

        d = ((bx - ax) * (-dy)) - ((by - ay) * (-dx));

        u = (((x - ax) * (-dy)) - ((y - ay) * (-dx))) / d;
        if(u < 0 || u > 1) {
            // outside the line.
            // essentially, we're seeing where a ray intersects with
            // an infinite version of our line.
            // 'u' represents whether it's within the line segment.
            continue;
        }

        t = (((bx - ax) * (y - ay)) - ((by - ay) * (x - ax))) / d;
        if(t < minDistance && t > 0) {
            // looking for closest intersection above 0 (below is behind ray)
            minDistance = t;
        }
    }

    return minDistance;
}

void SensorModel::PrintRenderCloud(SensorPacket* packet) {
    std::cout << "Render cloud: " << std::endl;
    for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; i++) {
        if(packet->renderCloud[i] == nullptr) {
            break;
        }
        packet->renderCloud[i]->Print();
        std::cout << std::endl;
    }
}
