#ifndef SLAM_MODELS_EKF_H_
#define SLAM_MODELS_EKF_H_

#include <windows.h>
#include <thread>
#include <atomic>

#include "..\Templates\slam.h"
#include "..\..\Models\Lidar\pointCloud.h"
#include "..\MapRepresentation\occupancyGrid.h"
#include "..\..\World\Objects\opoint.h"
#include "..\..\config.h"

/*
 * EKF
 *
 * Not finished. Not even close.
 * It hasn't even been fully refactored.
 * Perhaps one day I'll come back to this.
 * 
*/

class EKF : public SLAMModule {
public:
private:
    double lastScanTimestamp = 0.0;
    HANDLE slamSemaphore;
    std::thread slamThread;
    std::atomic<bool> slamFinished;

    double startX = 0.0;
    double startY = 0.0;
    double startTheta = 0.0;

    double lastScanX = 0.0;
    double lastScanY = 0.0;
    double lastScanTheta = 0.0;

    double currScanX = 0.0;
    double currScanY = 0.0;
    double currScanTheta = 0.0;

    double poseX = 0.0;
    double poseY = 0.0;
    double poseTheta = 0.0;

    OccupancyGrid* map = nullptr;
    PointCloud* lastPointCloud = nullptr;

public:
    EKF();
    ~EKF();

    void InitSlam(double startX, double startY, double startTheta);
    void UpdateSlam(double changeDist, double changeTheta, double commandTimestamp, double pointCloudTimestamp, PointCloud* pointCloud);
    void RunSlam();

private:
    void UpdateScanPoses();

    void RefineEstimates();

    std::vector<OPoint*> GenerateLandmarks(); //what to return?
    std::vector<OPoint*> GetCartesianPoints();
    std::vector<std::vector<OPoint*>> MakeClusters(std::vector<OPoint*>& cartesianPoints);
    std::vector<std::vector<OPoint*>> SplitIntoLines(const std::vector<OPoint*>& cluster);
    void MergeLines(std::vector<std::vector<OPoint*>>& lines);
    void RunLeastSquares(std::vector<std::vector<OPoint*>>& lines);
    void FindIntersections(std::vector<std::vector<OPoint*>>& lines, std::vector<OPoint*>& intersections);
};

#endif