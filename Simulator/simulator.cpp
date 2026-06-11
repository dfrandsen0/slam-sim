#include <chrono>
#include <iostream>

#include "simulator.h"
#include "..\Graphics\graphics.h"
#include "..\Graphics\renderPacket.h"
#include "..\World\world.h"
#include "..\World\Objects\opoint.h"
#include "..\Models\robotModel.h"
#include "..\Models\Lidar\pointCloud.h"
#include "..\AI\ai.h"
#include "..\SLAMModels\Templates\slam.h"
#include "..\SLAMModels\EKF\ekf.h"
#include "..\SLAMModels\Gmapping\gmapping.h"
#include "..\SLAMModels\MapRepresentation\poseRenderPacket.h"
#include "..\config.h"

using SchedClock = std::chrono::high_resolution_clock;
using Duration = std::chrono::duration<double>;
using TimeStamp = std::chrono::time_point<std::chrono::high_resolution_clock>;

Simulator::Simulator(HINSTANCE hInstance, int nCmdShow) {
    this->graphicsModule = new GraphicsModule(hInstance, nCmdShow);
    
    this->world = new World();
    this->graphicsModule->CreateBackground(this->world->GetMap());

    this->robotModel = new RobotModel();
    this->aiModule = new AIModule();
    
    if(START_AI_WITH_STOP) {
        this->aiModule->SetStarted();
    }

    if(STARTING_SLAM == SLAM_OPTION_EKF) {
        this->slamModule = new EKF();
    } else {
        this->slamModule = new Gmapping();
    }
}

Simulator::~Simulator() {
    delete this->graphicsModule;
    delete this->world;
    delete this->robotModel;
}

void Simulator::RunMainLoop() {
    TimeStamp lastTime = SchedClock::now();
    TimeStamp now = SchedClock::now();

    Duration elapsed;
    double scanAccumulator = 0.0;
    const double scanPeriod = SENSOR_MODEL_TIME_PER_SCAN;

    double mainAccumulator = 0.0;
    const double mainPeriod = GRAPHICS_FPS;

    double motionAccumulator = 0.0;
    double motionPeriod = MOTION_PERIOD;

    double totalAccumulator = 0.0;
    double changeDist, changeTheta;

    MSG msg = {};

    this->robotModel->InitializeRobot(this->world->GetMap());
    this->slamModule->InitSlam(this->robotModel->GetRealX(), this->robotModel->GetRealY(), this->robotModel->GetRealTheta());
    this->graphicsModule->GiveRenderMapAddress(this->slamModule->GetRenderMapAddress());
    this->graphicsModule->GiveRenderMapGuard(this->slamModule->GetRenderMapGuard());
    this->userInput = this->graphicsModule->GetUserInput();
    this->aiModule->GiveUserInput(this->userInput);
    this->aiModule->GiveRobotModel(this->robotModel);

    RenderPacket* renderPacket = new RenderPacket(
        this->robotModel->GetRealX(),
        this->robotModel->GetRealY(),
        this->robotModel->GetRealTheta(),
        nullptr,
        nullptr,
        nullptr,
        GMAPPING_NUM_PARTICLES
    );
            
    this->graphicsModule->UpdateRenderInfo(renderPacket);

    bool running = true;

    while(running) {
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) running = false;
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        if(userInput->GetSpaced()) {
            userInput->SetUnspaced();
            this->paused = !(this->paused);
        }

        now = SchedClock::now();
        elapsed = now - lastTime;
        lastTime = now;

        if(!(this->paused)) {
            motionAccumulator += elapsed.count();
            scanAccumulator += elapsed.count();
            mainAccumulator += elapsed.count();
            totalAccumulator += elapsed.count();
        }

        while(motionAccumulator >= motionPeriod) {
            this->aiModule->UpdateAI();
            
            RobotCommand initialCommand = this->aiModule->GetCommand();
            this->robotModel->CommandRobot(initialCommand, changeDist, changeTheta);

            double pointCloudTimestamp;
            PointCloud* pointCloud = this->robotModel->CopyLatestScan(pointCloudTimestamp);
            this->slamModule->UpdateSlam(changeDist, changeTheta, totalAccumulator, pointCloudTimestamp, pointCloud);
            motionAccumulator -= motionPeriod;
        }

        while(scanAccumulator >= scanPeriod) {
            this->robotModel->KickOffScan(totalAccumulator); //what if it can't?
            scanAccumulator -= scanPeriod;
        }

        while(mainAccumulator >= mainPeriod) {
            PoseRenderPacket* poses = this->slamModule->GetPoses();
            PoseRenderPacket* extendedPoses = this->slamModule->GetExtendedPoses();

            // save cpu cycles by not making copy
            OPoint** renderPointCloud = this->robotModel->CopyLatestRenderScan();

            RenderPacket* renderPacket = new RenderPacket(
                this->robotModel->GetRealX(),
                this->robotModel->GetRealY(),
                this->robotModel->GetRealTheta(),
                renderPointCloud,
                poses,
                extendedPoses,
                this->slamModule->GetNeff()
            );
            
            this->graphicsModule->UpdateRenderInfo(renderPacket);

            mainAccumulator -= mainPeriod;
        }

        this->graphicsModule->RenderFrame();
    }
}