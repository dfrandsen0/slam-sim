#include <iostream>
#include <thread>
#include <vector>
#include <utility>

#include "ekf.h"
#include "..\Templates\slam.h"
#include "..\..\Models\Lidar\pointCloud.h"
#include "..\MapRepresentation\sector.h"
#include "..\MapRepresentation\occupancyGrid.h"
#include "..\..\World\Objects\opoint.h"
#include "..\..\Utilities\mathUtilities.h"
#include "..\..\config.h"

EKF::EKF() : SLAMModule() {
    this->guardRenderMap = std::make_shared<std::mutex>();
}

EKF::~EKF() {
    //delete occupancy grid;
    
}

void EKF::InitSlam(double startX, double startY, double startTheta) {
    //TODO refactor

    this->startX = startX; //remove
    this->startY = startY; //remove
    this->startTheta = startTheta; //remove

    this->slamFinished = true; //remove
    this->slamSemaphore = CreateSemaphore(NULL, 0, 1, NULL); //remove
    this->slamThread = std::thread(RefineEstimates, this); //remove

    Sector* startingSector = new Sector();
    OccupancyGrid* startingGrid = new OccupancyGrid();

    startingGrid->AddSector(0, 0, startingSector);
    startingSector->AddReference();
    this->map = startingGrid;
}

void EKF::UpdateSlam(double changeDist, double changeTheta, double commandTimestamp, double pointCloudTimestamp, PointCloud* pointCloud) {
    // Check if pointcloud is nullptr
    // Estimate and update pose
    // Estimate delta xytheta and send to landmark
    //    extractor, with point cloud
    // - Convert polar coordinates to cartesian and account
    //    for motion blur
    // - Cluster according to distance and cluster threshold
    // - For each cluster, use line split and merge
    // - For each cluster, determine which lines should be one
    // - Determine intersection points
    // - Pass intersection points back to EKF

    if((pointCloud == nullptr) || (pointCloud->cloudSize == 0)) {
        delete pointCloud;
        return;
    }

    // update current point cloud
    if(pointCloudTimestamp != this->lastScanTimestamp) {
        this->lastScanTimestamp = pointCloudTimestamp;
        delete this->lastPointCloud;
        this->lastPointCloud = pointCloud->Copy();

        this->UpdateScanPoses();
    }

    if(!(this->slamFinished)) {
        return;
    }

    this->slamFinished = false;
    
    ReleaseSemaphore(this->slamSemaphore, 1, NULL);
    delete pointCloud;
}

void EKF::UpdateScanPoses() {
    this->lastScanX = this->currScanX;
    this->lastScanY = this->currScanY;
    this->lastScanTheta = this->currScanTheta;

    this->currScanX = this->poseX;
    this->currScanY = this->poseY;
    this->currScanTheta = this->poseTheta;
}

void EKF::RunSlam() {
    //TODO refactor
}

void EKF::RefineEstimates() {
    //TODO refactor
    std::vector<OPoint*> intersections;

    for(;;) {
        WaitForSingleObject(this->slamSemaphore, INFINITE);

        //add way to destroy
        intersections = this->GenerateLandmarks();

        //generate landmarks
        // augment state space or close loop
        // reestimate P, Q, K, and pose

        for(int i = 0; i < ((int)(intersections.size())); i++) {
            delete intersections[i];
        }

        //UpdateMap()
        //CreateRenderCopy()
        this->slamFinished = true;
    }
}

std::vector<OPoint*> EKF::GenerateLandmarks() {
    std::vector<OPoint*> cartesianPoints = this->GetCartesianPoints();
    std::vector<std::vector<OPoint*>> clusters = this->MakeClusters(cartesianPoints);
    std::vector<OPoint*> intersections;

    if(clusters.size() != 0) {
        std::vector<std::vector<OPoint*>> lines;

        for(const std::vector<OPoint*>& cluster : clusters) {
            lines = this->SplitIntoLines(cluster);
            this->MergeLines(lines);
            this->RunLeastSquares(lines);

            this->FindIntersections(lines, intersections);
        }
    }

    for(int i = 0; i < ((int)(cartesianPoints.size())); i++) {
        delete cartesianPoints[i];
    }

    return intersections;
}

std::vector<OPoint*> EKF::GetCartesianPoints() {
    std::vector<OPoint*> cartesianPoints;
    cartesianPoints.resize(this->lastPointCloud->cloudSize);

    PolarPoint** cloud = this->lastPointCloud->cloud;

    double pi = MathUtilities::PI;
    double deltaRadian = ((pi*2) / SENSOR_MODEL_POINTS_PER_SCAN);

    double poseStartX = this->lastScanX;
    double poseStartY = this->lastScanY;
    double poseStartTheta = this->lastScanTheta;

    double deltaX = (this->currScanX - poseStartX) / SENSOR_MODEL_POINTS_PER_SCAN;
    double deltaY = (this->currScanY - poseStartY) / SENSOR_MODEL_POINTS_PER_SCAN;
    double deltaTheta = (this->currScanTheta - poseStartTheta) / SENSOR_MODEL_POINTS_PER_SCAN;

    double startRadian = ((pi*2) - (deltaRadian / 2));

    int cloudPoint = 0;

    std::pair<double, double> ends;
    for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; ++i) {
        if(cloud[cloudPoint]->theta == startRadian) {
            ends = MathUtilities::PolarToCartesian(cloud[cloudPoint]->range, cloud[cloudPoint]->theta + poseStartTheta, poseStartX, poseStartY);

            cartesianPoints[cloudPoint] = new OPoint(ends.first, ends.second);

            cloudPoint++;
            if(cloudPoint == this->lastPointCloud->cloudSize) {
                break;
            }
        }

        poseStartX += deltaX;
        poseStartY += deltaY;
        poseStartTheta += deltaTheta;
        startRadian -= deltaRadian;
    }

    return cartesianPoints;
}

std::vector<std::vector<OPoint*>> EKF::MakeClusters(std::vector<OPoint*>& cartesianPoints) {
    std::vector<std::vector<OPoint*>> clusters;

    clusters.push_back({cartesianPoints[0]});

    for(int i = 1; i < ((int)(cartesianPoints.size())); i++) {
        if(MathUtilities::DistTwoPoints(cartesianPoints[i - 1], cartesianPoints[i]) <= EKF_CLUSTERING_THRESHOLD) {
            clusters.back().push_back(cartesianPoints[i]);
        } else {
            if(clusters.back().size() < EKF_MIN_CLUSTER_SIZE) {
                clusters.pop_back();
            }
            clusters.push_back({cartesianPoints[i]});
        }
    }

    return clusters;
}

std::vector<std::vector<OPoint*>> EKF::SplitIntoLines(const std::vector<OPoint*>& cluster) {
    // for()


    return {{nullptr}};
}

void EKF::MergeLines(std::vector<std::vector<OPoint*>>& lines) {

}

void EKF::RunLeastSquares(std::vector<std::vector<OPoint*>>& lines) {
    // double sumX, sumY, sumBoth, sumXSquared;
    // double m, b;
    // for(std::vector<OPoint*>& line : lines) {
    //     sumX = 0.0;
    //     sumY = 0.0;
    //     sumBoth = 0.0;
    //     sumXSquared = 0.0;

    //     for(OPoint* point : line) {
    //         sumX += point->x;
    //         sumY += point->y;
    //         sumBoth += (point->x * point->y);
    //         sumXSquared += (point->x * point->x);
    //     }

    //     if(sumX == 0) {
    //         continue; // what to do here?
    //     }

    //     m = ((line.size() * sumBoth) - (sumX * sumY)) / ((line.size() * sumXSquared) - (sumX * sumX));
    //     b = (sumY - (m * sumX)) / line.size();
    // }
}

void EKF::FindIntersections(std::vector<std::vector<OPoint*>>& lines, std::vector<OPoint*>& intersections) {

}
