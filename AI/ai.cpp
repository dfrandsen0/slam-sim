#include <iostream>
#include <cmath>

#include "ai.h"
#include "..\Listener\userInput.h"
#include "..\Models\robotModel.h"
#include "..\Utilities\mathUtilities.h"
#include "..\config.h"

AIModule::AIModule() {
    this->commandSequence.resize(4);
    this->currInstruction = 0;
    this->commandSequence[0] = {RobotCommand::STOP, 0};
}

//must NOT delete robotModel or userInput
AIModule::~AIModule() {

}

void AIModule::GiveUserInput(UserInput* userInput) {
    this->userInput = userInput;
}

void AIModule::GiveRobotModel(RobotModel* robotModel) {
    this->robotModel = robotModel;
}

//point and click first, then you can feed "point" to self-discovery module
//this version requires a bit of cheating to get distance; motion model is provided
//"close enough" model
void AIModule::UpdateAI() {
    if(!(this->userInput->GetClicked())) {
        return;
    }

    this->userInput->SetUnclicked();

    double x = (double)(this->userInput->GetX());
    double y = (double)(this->userInput->GetY());

    if((x < (CLIENT_SCREEN_WIDTH / 2.0)) || (y > (CLIENT_SCREEN_HEIGHT / 2.0))) {
        return;
    }

    x = (x - (((double)CLIENT_SCREEN_WIDTH)* 0.75)) * MM_PER_DIP;
    y = (y - (((double)CLIENT_SCREEN_HEIGHT) * 0.25)) * -MM_PER_DIP;

    double distX = x - this->robotModel->GetRealX();
    double distY = y - this->robotModel->GetRealY();
    double distance = std::sqrt(distX*distX + distY*distY);

    if(distance <= ROBOT_REAL_RADIUS) {
        return;
    }

    distance -= (MOTION_MODEL_DISTANCE_TO_STOP*2);
    int timeForward = (int)std::floor(distance / MOTION_MODEL_MAX_VELOCITY);

    double rotation = std::atan2(distY, distX) - this->robotModel->GetRealTheta();
    rotation = std::remainder(rotation, 2.0 * MathUtilities::PI);

    RobotCommand rotateDirection = RobotCommand::LEFT;
    if(rotation < 0) {
        rotateDirection = RobotCommand::RIGHT;
        rotation *= -1;
    }

    int timeRotate = (int)std::floor(rotation / MOTION_MODEL_ROTATION);

    // stop, rotate towards destination, move forward, stop
    this->commandSequence[3] = {RobotCommand::STOP, MOTION_MODEL_TIME_TO_STOP};
    this->commandSequence[2] = {rotateDirection, timeRotate};
    this->commandSequence[1] = {RobotCommand::FORWARD, timeForward};
    this->commandSequence[0] = {RobotCommand::STOP, MOTION_MODEL_TIME_TO_STOP};
    this->currInstruction = 3;
}

RobotCommand AIModule::GetCommand() {
    //prevents pre-rotation?
    if(!(this->started)) {
        this->started = true;
        return RobotCommand::STOP;
    }

    if(HARD_OVERRIDE_COMMANDS) {
        return OVERRIDE;
    }

    if(this->commandSequence[this->currInstruction].second <= 0) {
        if(this->currInstruction == 0) {
            return RobotCommand::STOP;
        } else {
            (this->currInstruction)--;
        }
    }

    (this->commandSequence[this->currInstruction].second)--;

    return this->commandSequence[this->currInstruction].first;
}

void AIModule::SetStarted() {
    this->started = true;
}