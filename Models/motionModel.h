#ifndef MODELS_MOTION_MODEL_H_
#define MODELS_MOTION_MODEL_H_

#include <mutex>

#include "..\World\map.h"
#include "..\config.h"

class MotionModel {
public:
private:
    Map* map = nullptr;

    double realX = 0.0;
    double realY = 0.0;
    double realTheta = 0.0;

    double velocity = 0.0;

public:
    MotionModel();
    ~MotionModel();

    void GiveMap(Map* map);

    double GetRealX();
    double GetRealY();
    double GetRealTheta();

    void ChangeRealX(double x);
    void ChangeRealY(double y);
    void ChangeRealTheta(double theta);
    
    void SetStartPosition(double x, double y, double theta);

    void GetMovement(RobotCommand initialCommand, double& changeChange, double& changeTheta);
    void UpdateRobotPosition(double changeX, double changeY, double changeTheta);
private:

};

#endif