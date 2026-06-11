#ifndef UTILITIES_MATH_UTILITIES_H_
#define UTILITIES_MATH_UTILITIES_H_

#include <utility>

#include "..\World\Objects\opoint.h"
#include "..\config.h"

// All theta's are expected in radians.

class MathUtilities {
public:
    static double PI;

    static std::pair<double, double> PolarToCartesian(double range, double theta, double cx, double cy);
    static void SampleCommand(RobotCommand command, double currTheta, double& velocity, double& changeDist, double& changeTheta);
    static double DistTwoPoints(OPoint* point1, OPoint* point2);
private:
    MathUtilities() = default;
};

#endif