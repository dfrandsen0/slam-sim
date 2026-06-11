#include <cmath>
#include <thread>
#include <mutex>

#include "motionModel.h"
#include "..\Utilities\mathUtilities.h"
#include "..\World\map.h"
#include "..\config.h"

MotionModel::MotionModel() {

}

MotionModel::~MotionModel() {
    //do not delete map here.
}

void MotionModel::GiveMap(Map* map) {
    this->map = map;
}

double MotionModel::GetRealX() {
    return this->realX;
}

double MotionModel::GetRealY() {
    return this->realY;
}

double MotionModel::GetRealTheta() {
    return this->realTheta;
}

void MotionModel::ChangeRealX(double x) {
    this->realX += x;
}

void MotionModel::ChangeRealY(double y) {
    this->realY += y;
}

void MotionModel::ChangeRealTheta(double theta) {
    this->realTheta += theta;
}

void MotionModel::SetStartPosition(double x, double y, double theta) {
    this->realX = x;
    this->realY = y;
    this->realTheta = theta;
}

void MotionModel::GetMovement(RobotCommand initialCommand, double& changeDist, double& changeTheta) {
    MathUtilities::SampleCommand(initialCommand, this->realTheta, this->velocity, changeDist, changeTheta);
}

void MotionModel::UpdateRobotPosition(double changeX, double changeY, double changeTheta) {
    this->ChangeRealX(changeX);
    this->ChangeRealY(changeY);
    this->ChangeRealTheta(changeTheta);
}
