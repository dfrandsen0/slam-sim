#ifndef SLAM_MODELS_TEMPLATES_SLAM_H_
#define SLAM_MODELS_TEMPLATES_SLAM_H_

#include <windows.h>
#include <vector>
#include <mutex>
#include <memory>
#include <unordered_set>
#include <atomic>
#include <thread>

#include "..\..\Models\Lidar\pointCloud.h"
#include "..\MapRepresentation\occupancyGrid.h"
#include "..\MapRepresentation\poseRenderPacket.h"
#include "..\..\config.h"

class SLAMModule {
protected:
    HANDLE slamSemaphore;
    std::thread slamThread;
    std::atomic<bool> slamFinished;

    std::vector<float>* renderMap = nullptr;
    std::shared_ptr<std::mutex> guardRenderMap;

    PoseRenderPacket* poses = nullptr;
    PoseRenderPacket* extendedPoses = nullptr;
    std::mutex guardPoses;

    std::mutex guardNeff;
    double neff = GMAPPING_NUM_PARTICLES;

    //for render purposes ONLY. No other operation will access this!
    double startX = 0.0;
    double startY = 0.0;
    double startTheta = 0.0;

public:
    explicit SLAMModule();
    virtual ~SLAMModule();

    virtual void InitSlam(double startX, double startY, double startTheta) = 0;
    virtual void UpdateSlam(double changeDist, double changeTheta, double commandTimestamp, double pointCloudTimestamp, PointCloud* pointCloud) = 0;
    virtual void RunSlam() = 0;

    PoseRenderPacket* GetPoses();
    PoseRenderPacket* GetExtendedPoses();
    void ReplacePoses(PoseRenderPacket* newPacket, PoseRenderPacket* newExtendedPacket);

    double GetNeff();

    std::vector<float>** GetRenderMapAddress();
    std::shared_ptr<std::mutex> GetRenderMapGuard();

protected:
    void RefineEstimates();

    void AddAffectedCells(int startX, int startY, int endX, int endY, std::unordered_set<std::pair<int, int>, pair_hash>& misses, std::unordered_set<std::pair<int, int>, pair_hash>& hits);

    void UpdateNeff(double newNeff);

    void CreateRenderCopy(std::unordered_map<std::pair<int, int>, Sector*, pair_hash>* model);
    std::pair<float, float> XYToDips(double x, double y);

};

#endif