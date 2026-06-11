#include <iostream>
#include <cmath>
#include <utility>

#include "mathUtilities.h"
#include "..\World\Objects\opoint.h"
#include "utilities.h"

// All theta's are expected in radians.

double MathUtilities::PI = 3.1415926535898;

std::pair<double, double> MathUtilities::PolarToCartesian(double range, double theta, double cx, double cy) {
    return {
        cx + (range * std::cos(theta)),
        cy + (range * std::sin(theta))
    };
}

void MathUtilities::SampleCommand(RobotCommand command, double currTheta, double& velocity, double& changeDist, double& changeTheta) {
    double velocityFinal, timeHit;
    switch(command) {
        case RobotCommand::FORWARD:
            if((velocity + MOTION_MODEL_ACCELERATION) < MOTION_MODEL_MAX_VELOCITY) {
                velocityFinal = velocity + MOTION_MODEL_ACCELERATION;
                changeDist = velocity + (0.5 * MOTION_MODEL_ACCELERATION);
                velocity += MOTION_MODEL_ACCELERATION;
            } else {
                timeHit = (MOTION_MODEL_MAX_VELOCITY - velocity) / MOTION_MODEL_ACCELERATION;
                changeDist = (velocity * timeHit) + (0.5 * MOTION_MODEL_ACCELERATION * (timeHit*timeHit));
                changeDist += (1 - timeHit) * MOTION_MODEL_MAX_VELOCITY;
                velocity = MOTION_MODEL_MAX_VELOCITY;
            }

            changeTheta = 0;
            break;
            
        case RobotCommand::STOP:
            if((velocity - MOTION_MODEL_ACCELERATION) > 0) {
                velocityFinal = velocity - MOTION_MODEL_ACCELERATION;
                changeDist = velocity + (0.5 * -MOTION_MODEL_ACCELERATION);
                velocity = velocityFinal;
            } else {
                timeHit = (0 - velocity) / (-MOTION_MODEL_ACCELERATION);
                changeDist = (velocity * timeHit) - (0.5 * MOTION_MODEL_ACCELERATION * (timeHit*timeHit));
                velocity = 0;
            }
            changeTheta = 0;

            break;

        case RobotCommand::RIGHT:
            changeTheta = -MOTION_MODEL_ROTATION;
            changeDist = 0;
            break;

        case RobotCommand::LEFT:
            changeTheta = MOTION_MODEL_ROTATION;
            changeDist = 0;
            break;
    }
}

double MathUtilities::DistTwoPoints(OPoint* point1, OPoint* point2) {
    double deltaX = point1->x - point2->x;
    double deltaY = point1->y - point2->y;
    return std::sqrt(deltaX*deltaX + deltaY*deltaY);
}
