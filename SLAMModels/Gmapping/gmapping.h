#ifndef SLAM_MODELS_GMAPPING_H_
#define SLAM_MODELS_GMAPPING_H_

#include <windows.h>
#include <utility>
#include <atomic>
#include <unordered_set>
#include <vector>

#include "..\Templates\slam.h"
#include "particle.h"
#include "..\..\Models\Lidar\pointCloud.h"
#include "statePacket.h"
#include "..\MapRepresentation\occupancyGrid.h"
#include "logField.h"
#include "..\..\World\Objects\opoint.h"
#include "..\..\config.h"

class Gmapping : public SLAMModule {
public:
private:
    Particle** particles = nullptr;

    std::atomic<bool> refreshParticles;

    bool backUpdated = true;

    int* particlesToRefresh = nullptr;

    std::pair<double, double>* history;
    int historySize = 0;

    int ticksSinceLastUpdate = 0;

    StatePacket* currPacket = nullptr;

    // How far would the robot have gone if there was no noise?
    //   Used for determining if slam should run given min distance
    double accumulatedPoseSinceLastUpdate = 0.0;

    double lastScanExpDist = 0.0;
    double lastScanExpTheta = 0.0;

    double accumulatingExpDist = 0.0;
    double accumulatingExpTheta = 0.0;

    double lastScanTimestamp = 0.0;

public:
    Gmapping();
    ~Gmapping();

    void InitSlam(double startX, double startY, double startTheta);

    void UpdateSlam(double changeDist, double changeTheta, double commandTimestamp, double pointCloudTimestamp, PointCloud* pointCloud);
    void RunSlam();

private:
    void MoveParticles(double changeDist, double changeTheta);
    void BackUpdateParticles();

    void UpdateMaps();

    void UpdatePoses();
    int GetStrongestParticleIndex(Particle** particles);

    void CreateInverseSigmas(LogField* logField);
    void GetPoints(LogField* logField, int particleIndex);
    void GetSectorRange(LogField* logField);
    void CreateGrid(LogField* logField);
    void PopulateLikelihoodField(LogField* logField);
    void DistanceTransform1D(const std::vector<double>& f, std::vector<double>& d, int n);
    void NudgeParticle(LogField* logField);
    void SampleParticles(LogField* logField);
    double ScoreParticlePose(LogField* logField, double deltaX, double deltaY, double deltaTheta);
    double ScoreRelativePosition(LogField* logField, double deltaX, double deltaY, double deltaTheta);

    void FillParticlesToRefresh();
    void FlushRefreshedParticles();
};

#endif