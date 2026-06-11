#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include <windows.h>

#include "..\Graphics\graphics.h"
#include "..\Listener\userInput.h"
#include "..\Graphics\renderPacket.h"
#include "..\World\world.h"
#include "..\Models\robotModel.h"
#include "..\AI\ai.h"
#include "..\SLAMModels\Templates\slam.h"

class Simulator {
private:
    World* world = nullptr;
    RobotModel* robotModel = nullptr;
    AIModule* aiModule = nullptr;
    SLAMModule* slamModule = nullptr;
    GraphicsModule* graphicsModule = nullptr;
    
    UserInput* userInput = nullptr;

    bool paused = false;

public:
    Simulator(HINSTANCE hInstance, int nCmdShow);
    ~Simulator();
private:
public:
    void RunMainLoop();
private:

};

#endif