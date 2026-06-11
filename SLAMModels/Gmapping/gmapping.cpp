#include <windows.h>
#include <thread>
#include <utility>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <mutex>
#include <algorithm>
#include <cfloat>
#include <chrono>

#include "gmapping.h"
#include "logField.h"
#include "..\Templates\slam.h"
#include "particle.h"
#include "..\..\Models\Lidar\pointCloud.h"
#include "..\MapRepresentation\occupancyGrid.h"
#include "..\MapRepresentation\sector.h"
#include "..\MapRepresentation\poseRenderPacket.h"
#include "..\..\World\Objects\opoint.h"
#include "..\..\Utilities\utilities.h"
#include "..\..\Utilities\mathUtilities.h"
#include "..\..\config.h"

Gmapping::Gmapping() : SLAMModule() {
    this->particles = new Particle*[GMAPPING_NUM_PARTICLES];
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        this->particles[i] = nullptr;
    }

    this->guardRenderMap = std::make_shared<std::mutex>();
    this->history = new std::pair<double, double>[GMAPPING_HISTORY_SIZE];
}

Gmapping::~Gmapping() {
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        delete this->particles[i];
    }
    delete[] this->particles;

    //todo add way to destroy thread
}

// Initiate slam; give particles same map
void Gmapping::InitSlam(double startX, double startY, double startTheta) {
    SLAMModule::InitSlam(startX, startY, startTheta);

    this->refreshParticles = false;

    this->particlesToRefresh = new int[GMAPPING_NUM_PARTICLES];
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        this->particlesToRefresh[i] = i;
    }

    Sector* startingSector1 = new Sector();

    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        this->particles[i] = new Particle();
        OccupancyGrid* startingGrid = new OccupancyGrid();

        startingGrid->AddSector(0, 0, startingSector1);
        startingSector1->AddReference();

        this->particles[i]->map = startingGrid;
        this->particles[i]->weight = GMAPPING_STARTING_WEIGHT;
    }
}

// (Step 1)
// Check if algorithm finished; if so, clean up and resolve any lingering affects.
//   Move particles according to odometry. If minimum time/dist, run algorithm.
void Gmapping::UpdateSlam(double changeDist, double changeTheta, double commandTimestamp, double pointCloudTimestamp, PointCloud* pointCloud) {

    if(!(this->slamFinished)) {
        //track history
        this->history[this->historySize].first = changeDist;
        this->history[this->historySize].second = changeTheta;
        (this->historySize)++;
    } else if(this->slamFinished && !(this->backUpdated)) {
        this->backUpdated = true;

        this->BackUpdateParticles();
    }

    this->MoveParticles(changeDist, changeTheta);
    this->UpdatePoses();

    this->accumulatingExpDist += changeDist;
    this->accumulatingExpTheta += changeTheta;

    this->accumulatedPoseSinceLastUpdate += std::abs(changeDist);
    this->accumulatedPoseSinceLastUpdate += (std::abs(changeTheta) * MOTION_MODEL_ROTATION_AMP);

    //check time stamps and update particle history if required
    if(pointCloudTimestamp != this->lastScanTimestamp) {
        this->lastScanTimestamp = pointCloudTimestamp;
        
        for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
            this->particles[i]->UpdateHistory();
        }

        this->lastScanExpDist = this->accumulatingExpDist;
        this->lastScanExpTheta = this->accumulatingExpTheta;

        this->accumulatingExpDist = 0.0;
        this->accumulatingExpTheta = 0.0;
    }

    (this->ticksSinceLastUpdate)++;

    if((pointCloud == nullptr) || (pointCloud->cloudSize == 0)) {
        delete pointCloud;
        return;
    }

    if(!(this->slamFinished)) {
        delete pointCloud;
        return;
    }

    // Make a separate copy of every particle so the thread can be running and the particles on the
    //   screen don't freeze.
    if((this->ticksSinceLastUpdate >= SLAM_MAXIMUM_PERIOD_COUNT) ||
       ((this->ticksSinceLastUpdate >= SLAM_MINIMUM_PERIOD_COUNT) &&
        (this->accumulatedPoseSinceLastUpdate >= SLAM_MINIMUM_DISTANCE))) {

        this->accumulatedPoseSinceLastUpdate = 0.0;
        this->ticksSinceLastUpdate = 0;
        this->slamFinished = false;
        this->backUpdated = false;

        delete this->currPacket;
        StatePacket* packet = new StatePacket();
        packet->CopyInfo(this->particles);
        packet->pointCloud = pointCloud;

        packet->commandTimestamp = commandTimestamp;
        packet->pointCloudTimestamp = pointCloudTimestamp;

        packet->expDist = this->lastScanExpDist;
        packet->expTheta = this->lastScanExpTheta;

        this->currPacket = packet;

        ReleaseSemaphore(this->slamSemaphore, 1, NULL);
    } else {
        delete pointCloud;
    }
}

// (Step 2)
// Update particles according to odometry. Add noise appropriately.
void Gmapping::MoveParticles(double changeDist, double changeTheta) {
    double noisyDist = 0;
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        if(changeDist == 0) {
            if(changeTheta == 0) {
                break;
            }
            this->particles[i]->theta += changeTheta + Utilities::GetFixedNoise(MOTION_MODEL_ROTATION_FIXED);
            continue;
        }

        this->particles[i]->theta += (changeTheta + Utilities::GetFixedNoise(MOTION_MODEL_FORWARD_ROTATION_DEVIATION));
        noisyDist = changeDist + Utilities::GetRandomNoise(changeDist, MOTION_MODEL_FORWARD_DEVIATION);

        this->particles[i]->x += (noisyDist * std::cos(this->particles[i]->theta));
        this->particles[i]->y += (noisyDist * std::sin(this->particles[i]->theta));
    }
}


// The gmapping algorithm slightly "nudges" particles; move the particles back to the "nudged"
//   position, then refresh if needed and update according to history. Includes reset of history
void Gmapping::BackUpdateParticles() {
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        this->particles[i]->x = this->currPacket->particles[i]->x;
        this->particles[i]->y = this->currPacket->particles[i]->y;
        this->particles[i]->theta = this->currPacket->particles[i]->theta;
        this->particles[i]->weight = this->currPacket->particles[i]->weight;
    }

    if(this->refreshParticles) {
        this->FlushRefreshedParticles();
    }

    for(int i = 0; i < this->historySize; i++) {
        this->MoveParticles(this->history[i].first, this->history[i].second);
    }
    
    this->historySize = 0;
}

// Run the gmapping algorithm. See Resources/Notes/gmapping.txt for algorithm steps and example.
void Gmapping::RunSlam() {

    // ====================
    //      steps 3-7
    // ====================

    LogField* logField = new LogField();
    logField->cloud = this->currPacket->pointCloud->cloud;
    logField->numPoints = this->currPacket->pointCloud->cloudSize;
    this->CreateInverseSigmas(logField);

    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        this->GetPoints(logField, i);
        logField->currParticle = this->currPacket->particles[i];

        //num points will not be 0.
        this->GetSectorRange(logField);
        logField->likelihoodField.reserve(((logField->sectorMaxX)-(logField->sectorMinX)) * ((logField->sectorMaxY)-(logField->sectorMinY)) * (GMAPPING_SECTOR_SIZE*GMAPPING_SECTOR_SIZE));

        this->CreateGrid(logField);
        this->PopulateLikelihoodField(logField);

        this->NudgeParticle(logField);

        //check local samples and build L(k)
        this->SampleParticles(logField);

        for(int i = 0; i < logField->numPoints; i++) {
            delete logField->cartesianPoints[i];
            delete logField->poses[i];
        }

        delete[] logField->cartesianPoints;
        delete[] logField->poses;
    }

    delete logField;
    // ^^^^ steps 3-7 ^^^^


    // ====================
    //   step 8 (sorta 9)
    // ====================

    double totalWeight = 0.0;
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        totalWeight += this->currPacket->particles[i]->weight;
    }

    double neffWeight = 0.0;
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        this->currPacket->particles[i]->weight /= totalWeight; // (Step 8)
        neffWeight += (this->currPacket->particles[i]->weight)*(this->currPacket->particles[i]->weight);
    }

    // particle efficiency; Neff = 1 / ( sum(final_weights^2))
    double newNeff = 1.0 / neffWeight;

    this->UpdateNeff(newNeff);
    // ^^^^ step 8 (-9) ^^^^

    // ====================
    //     steps 12/13
    // ====================

    this->UpdateMaps();

    int strongestIndex = this->GetStrongestParticleIndex(this->currPacket->particles);
    std::unordered_map<std::pair<int, int>, Sector*, pair_hash>* model = 
        this->currPacket->particles[strongestIndex]->map->GetCells();

    // (Step 13, defined in slam.cpp)
    this->CreateRenderCopy(model);

    // ====================
    //     steps 9-11
    // ====================

    if(newNeff < (GMAPPING_NEFF_THRESHOLD)) {
        this->FillParticlesToRefresh();
    }
}

// (Step 12)
// For each particle's map, ray trace from start of laser to end, marking cells as
//   hit or miss.
void Gmapping::UpdateMaps() {
    double pi = MathUtilities::PI;
    double deltaRadian = ((pi*2) / SENSOR_MODEL_POINTS_PER_SCAN);
    double inverseCellSize = 1 / GMAPPING_GRID_CELL_SIZE;
    if(this->currPacket->pointCloud == nullptr) {
        return;
    }

    PolarPoint** cloud = this->currPacket->pointCloud->cloud;
    if(this->currPacket->pointCloud->cloudSize == 0) {
        return;
    }

    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        Particle* particle = this->currPacket->particles[i];
        if(particle->map == nullptr) {
            continue;
        }

        std::unordered_set<std::pair<int, int>, pair_hash> misses;
        std::unordered_set<std::pair<int, int>, pair_hash> hits;

        // If we are not accounting for motion blur, we simply "0 out" the delta variables, and
        //    act like the scan started at the final pose.
        double poseStartX = particle->currScanX;
        double poseStartY = particle->currScanY;
        double poseStartTheta = particle->currScanTheta;
        double deltaX = 0.0;
        double deltaY = 0.0;
        double deltaTheta = 0.0;

        if(ACCOUNT_FOR_MOTION_BLUR) {
            poseStartX = particle->oldScanX;
            poseStartY = particle->oldScanY;
            poseStartTheta = particle->oldScanTheta;

            deltaX = (particle->currScanX - poseStartX) / SENSOR_MODEL_POINTS_PER_SCAN;
            deltaY = (particle->currScanY - poseStartY) / SENSOR_MODEL_POINTS_PER_SCAN;
            deltaTheta = (particle->currScanTheta - poseStartTheta) / SENSOR_MODEL_POINTS_PER_SCAN;
        }

        double startRadian = ((pi*2) - (deltaRadian / 2));

        int cloudPoint = 0;
        int totalPoints = this->currPacket->pointCloud->cloudSize;

        for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; ++i) {
            if(cloud[cloudPoint]->theta == startRadian) {
                std::pair<double, double> ends = MathUtilities::PolarToCartesian(cloud[cloudPoint]->range, cloud[cloudPoint]->theta + poseStartTheta, poseStartX, poseStartY);

                int startX = (int)std::floor(poseStartX * inverseCellSize);
                int startY = (int)std::floor(poseStartY * inverseCellSize);
                int endX = (int)std::floor(ends.first * inverseCellSize);
                int endY = (int)std::floor(ends.second * inverseCellSize);

                this->AddAffectedCells(startX, startY, endX, endY, misses, hits);

                cloudPoint++;
                if(cloudPoint == totalPoints) {
                    break;
                }
            }

            poseStartX += deltaX;
            poseStartY += deltaY;
            poseStartTheta += deltaTheta;
            startRadian -= deltaRadian;
        }

        // Hits trump misses
        for(const std::pair<int, int>& hit : hits) {
            if(misses.find(hit) != misses.end()) {
                misses.erase(hit);
            }

            particle->map->ChangeCell(hit.first, hit.second, GMAPPING_LOG_ODDS_HIT);
        }

        for(const std::pair<int, int>& miss : misses) {
            particle->map->ChangeCell(miss.first, miss.second, GMAPPING_LOG_ODDS_MISS);
        }
    }
}

// Updates the positions the graphics will use to draw particles
void Gmapping::UpdatePoses() {
    std::lock_guard<std::mutex> lock(this->guardPoses);

    PoseRenderPacket* newPacket = new PoseRenderPacket(GMAPPING_NUM_PARTICLES, 3);
    PoseRenderPacket* newExtendedPacket = new PoseRenderPacket(GMAPPING_NUM_PARTICLES, 3);
    int strongestIndex = this->GetStrongestParticleIndex(this->particles);

    std::pair<double, double> screenCords;
    double cosTheta = std::cos(this->startTheta);
    double sinTheta = std::sin(this->startTheta);

    //rotate, translate, then convert to dips
    double worldX = ((this->particles[strongestIndex]->x)*cosTheta)-((this->particles[strongestIndex]->y)*sinTheta) + this->startX;
    double worldY = ((this->particles[strongestIndex]->x)*sinTheta)+((this->particles[strongestIndex]->y)*cosTheta) + this->startY;
    screenCords = this->XYToDips(worldX, worldY);

    newPacket->AddPose({screenCords.first, screenCords.second, this->particles[strongestIndex]->theta + this->startTheta});
    newExtendedPacket->AddPose({worldX, worldY, this->particles[strongestIndex]->theta + this->startTheta});

    double worldXTotal = worldX;
    double worldYTotal = worldY;

    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        if(i == strongestIndex) {
            continue;
        }

        worldX = ((this->particles[i]->x)*cosTheta)-((this->particles[i]->y)*sinTheta) + this->startX;
        worldY = ((this->particles[i]->x)*sinTheta)+((this->particles[i]->y)*cosTheta) + this->startY;
        screenCords = this->XYToDips(worldX, worldY);

        worldXTotal += worldX;
        worldYTotal += worldY;

        newPacket->AddPose({screenCords.first, screenCords.second, this->particles[i]->theta + this->startTheta});
        newExtendedPacket->AddPose({worldX, worldY, this->particles[i]->theta + this->startTheta});
    }

    worldXTotal /= GMAPPING_NUM_PARTICLES;
    worldYTotal /= GMAPPING_NUM_PARTICLES;

    double baseX = CLIENT_SCREEN_WIDTH * 0.75;
    double baseY = CLIENT_SCREEN_HEIGHT * 0.75;

    for(int i = 0; i < newExtendedPacket->numValues; i += 3) {
        newExtendedPacket->poses[i] = baseX + (newExtendedPacket->poses[i] - worldXTotal);
        newExtendedPacket->poses[i + 1] = baseY - (newExtendedPacket->poses[i + 1] - worldYTotal);
    }

    this->ReplacePoses(newPacket, newExtendedPacket);
}

// Returns the particle with the largest weight. Call before normalizing
int Gmapping::GetStrongestParticleIndex(Particle** particles) {
    int index = 0;
    double maxWeight = -1;
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        if(this->particles[i]->weight > maxWeight) {
            maxWeight = this->particles[i]->weight;
            index = i;
        }
    }

    return index;
}

// Creates an array for computational speed. Part of the scoring equation for P(Z)
//   requires a sigma for that laser; but it's based on distance. Sigma is found
//   per laser point, and capped at half a grid cell size. Calculates that entire
//   piece of the equation, -1/(2*sigma^2).
void Gmapping::CreateInverseSigmas(LogField* logField) {
    double* newSigmas = new double[logField->numPoints]();
    std::fill_n(newSigmas, logField->numPoints, GMAPPING_SCAN_MATCHING_DEFAULT_SIGMA);

    double sigmaMM;
    for(int i = 0; i < logField->numPoints; i++) {
        for(int tier = 0; tier < SENSOR_MODEL_ACCURACY_TIERS; tier++) {
            if(logField->cloud[i]->range < SENSOR_MODEL_ACCURACY[tier].second) {
                sigmaMM = (std::max)(SENSOR_MODEL_ACCURACY[tier].first * logField->cloud[i]->range, GMAPPING_GRID_CELL_SIZE / 2.0);
                newSigmas[i] = -1.0 / (2.0*sigmaMM*sigmaMM);
                break;
            }
        }
    }

    logField->inverseSigmas = newSigmas;
}

// Given a point cloud and particle, create a list of cartesian points corresponding
//   to the transformation of those polar points. Accounts for noise using that
//   particle's pose. Creates a second list of all laser starting points, or, the pose
//   of that particle at every given laser firing point.
void Gmapping::GetPoints(LogField* logField, int particleIndex) {
    logField->currParticle = this->currPacket->particles[particleIndex];
    Particle* particle = logField->currParticle;
    PolarPoint** cloud = this->currPacket->pointCloud->cloud;

    if(logField->numPoints == 0) {
        return;
    }

    // If we are not accounting for motion blur, we simply "0 out" the delta variables, and
    //    act like the scan started at the final pose.
    double poseStartX = particle->currScanX;
    double poseStartY = particle->currScanY;
    double poseStartTheta = particle->currScanTheta;
    double deltaX = 0.0;
    double deltaY = 0.0;
    double deltaTheta = 0.0;

    if(ACCOUNT_FOR_MOTION_BLUR) {
        poseStartX = particle->oldScanX;
        poseStartY = particle->oldScanY;
        poseStartTheta = particle->oldScanTheta;

        deltaX = (particle->currScanX - poseStartX) / SENSOR_MODEL_POINTS_PER_SCAN;
        deltaY = (particle->currScanY - poseStartY) / SENSOR_MODEL_POINTS_PER_SCAN;
        deltaTheta = (particle->currScanTheta - poseStartTheta) / SENSOR_MODEL_POINTS_PER_SCAN;
    }

    double pi = MathUtilities::PI;
    double deltaRadian = ((pi*2) / SENSOR_MODEL_POINTS_PER_SCAN);
    double startRadian = ((pi*2) - (deltaRadian / 2));

    int cloudPoint = 0;

    OPoint** returnCartPoints = new OPoint*[logField->numPoints];
    OPoint** returnPoses = new OPoint*[logField->numPoints];

    for(int i = 0; i < SENSOR_MODEL_POINTS_PER_SCAN; ++i) {
        if(cloud[cloudPoint]->theta == startRadian) {
            std::pair<double, double> ends = MathUtilities::PolarToCartesian(cloud[cloudPoint]->range, cloud[cloudPoint]->theta + poseStartTheta, poseStartX, poseStartY);

            returnCartPoints[cloudPoint] = new OPoint(ends.first, ends.second);
            returnPoses[cloudPoint] = new OPoint(poseStartX, poseStartY);

            cloudPoint++;
            if(cloudPoint == logField->numPoints) {
                break;
            }
        }

        poseStartX += deltaX;
        poseStartY += deltaY;
        poseStartTheta += deltaTheta;
        startRadian -= deltaRadian;
    }

    logField->cartesianPoints = returnCartPoints;
    logField->poses = returnPoses;
}

// Finds the minimum and maximum cell for the point cloud, and pulls the
//   sectors associated with those cells. Used to determine what sectors
//   should be referenced when scan matching. Include a sector safety of
//   +- 1 in both x/y.
void Gmapping::GetSectorRange(LogField* logField) {
    double minX = DBL_MAX;
    double maxX = -DBL_MAX;
    double minY = DBL_MAX;
    double maxY = -DBL_MAX;

    if(logField->cartesianPoints == nullptr) {
        return;
    }

    for(int i = 0; i < this->currPacket->pointCloud->cloudSize; i++) {
        minX = (std::min)(minX, logField->cartesianPoints[i]->x);
        maxX = (std::max)(maxX, logField->cartesianPoints[i]->x);
        minY = (std::min)(minY, logField->cartesianPoints[i]->y);
        maxY = (std::max)(maxY, logField->cartesianPoints[i]->y);
    }

    double inverseCellSize = 1.0 / GMAPPING_GRID_CELL_SIZE;
    logField->sectorMinX = ((int)std::floor(((double)std::floor(minX * inverseCellSize)) / (double)GMAPPING_SECTOR_SIZE)) - 1;
    logField->sectorMinY = ((int)std::floor(((double)std::floor(minY * inverseCellSize)) / (double)GMAPPING_SECTOR_SIZE)) - 1;
    logField->sectorMaxX = ((int)std::floor(((double)std::floor(maxX * inverseCellSize)) / (double)GMAPPING_SECTOR_SIZE)) + 1;
    logField->sectorMaxY = ((int)std::floor(((double)std::floor(maxY * inverseCellSize)) / (double)GMAPPING_SECTOR_SIZE)) + 1;

    logField->offsetX = logField->sectorMinX * -GMAPPING_SECTOR_MM_SIZE;
    logField->offsetY = logField->sectorMinY * -GMAPPING_SECTOR_MM_SIZE;
}

// Creates a 2D grid to hold a likelihood field based on the sectors associated with
//   the full range of cells found in the point cloud. Empty sectors are treated as
//   misses; if they haven't been generated, no laser has been there.
void Gmapping::CreateGrid(LogField* logField) {
    logField->sectorsWidth = logField->sectorMaxX - logField->sectorMinX + 1;
    logField->sectorsHeight = logField->sectorMaxY - logField->sectorMinY + 1;
    int totalCells = logField->sectorsWidth * logField->sectorsHeight * GMAPPING_SECTOR_SIZE * GMAPPING_SECTOR_SIZE;
    logField->likelihoodField.assign(totalCells, GMAPPING_INF);

    int lfIndex = 0;
    int cellIndex;

    for(int y = logField->sectorMinY; y <= logField->sectorMaxY; ++y) {
        std::vector<Sector*> rowSectors;
        rowSectors.reserve(logField->sectorsWidth);

        for(int x = logField->sectorMinX; x <= logField->sectorMaxX; ++x) {
            rowSectors.push_back(logField->currParticle->map->GetSector(x, y));
        }

        for(int row = 0; row < GMAPPING_SECTOR_SIZE; row++) {
            for(Sector* currSector : rowSectors) {
                if(currSector == nullptr) {
                    lfIndex += GMAPPING_SECTOR_SIZE;
                } else {
                    cellIndex = row * GMAPPING_SECTOR_SIZE;
                    for(int i = 0; i < GMAPPING_SECTOR_SIZE; i++) {
                        if(currSector->cells[cellIndex] >= GMAPPING_LOG_ODDS_WALL_VALUE) {
                            logField->likelihoodField[lfIndex] = 0;
                        }
                        cellIndex++;
                        lfIndex++;
                    }
                }
            }
        }
    }
}

// Euclidean Distance Transform using Felzenszwalb-Huttenlocher (FH) algorithm.
// Do 1D distance transform first on rows, then on cols. Expects a grid of all INF
//   except obstacle cells, which have 0.
void Gmapping::PopulateLikelihoodField(LogField* logField) {
    logField->cellsWidth = logField->sectorsWidth * GMAPPING_SECTOR_SIZE;
    logField->cellsHeight = logField->sectorsHeight * GMAPPING_SECTOR_SIZE;
    int width = logField->cellsWidth;
    int height = logField->cellsHeight;

    // Pass 1: Rows
    std::vector<double> row(width);
    std::vector<double> resultRows(width);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            row[x] = logField->likelihoodField[(y * width) + x];
        }
        
        this->DistanceTransform1D(row, resultRows, width);
        
        for (int x = 0; x < width; ++x) {
            logField->likelihoodField[(y * width) + x] = resultRows[x];
        }
    }

    double cellSizeSquared = GMAPPING_GRID_CELL_SIZE*GMAPPING_GRID_CELL_SIZE;

    // Pass 2: Columns
    std::vector<double> col(height);
    std::vector<double> resultCols(height);
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            col[y] = logField->likelihoodField[(y * width) + x];
        }
        
        this->DistanceTransform1D(col, resultCols, height);
        
        for (int y = 0; y < height; ++y) {
            logField->likelihoodField[(y * width) + x] = (std::min)(resultCols[y] * cellSizeSquared, GMAPPING_SCAN_MATCHING_MAX_DIST);
        }
    }
}

// Get's the lower envelope of a set of parabolas, each representing the closeness of that index's obstacle.
// - inDists represents distances gained during previous pass
// - result is output
// - n is size of arrays
// 
// For every y value within the lower envelope, it's index's closest obstacle is found
//    through the parabola associated with that part of the lower envelope. 
void Gmapping::DistanceTransform1D(const std::vector<double>& inDists, std::vector<double>& result, int n) {
    std::vector<int> v(n);        // locations of parabolas in lower envelope
    std::vector<double> z(n + 1); // locations of boundaries between parabolas, one extra for upper bound
    
    int k = 0; // parabola num

    v[0] = 0; // init 
    z[0] = -GMAPPING_INF; // init starting lower envelope
    z[1] = +GMAPPING_INF;

    // intersection of two parabolas, s = (f(q) + q^2 - (f(p) + p^2)) / (2q - 2p)
    for (int q = 1; q < n; ++q) {
        // Calculate intersection of parabola q and v[k]
        double s = ((inDists[q] + q * q) - (inDists[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
        
        while (s <= z[k]) {
            k--; // "pop off" occluded parabolas, or parabolas not involved in lower envelope
            s = ((inDists[q] + q*q) - (inDists[v[k]] + (v[k]*v[k]))) / ((2.0*q) - (2.0*v[k]));
        }
        
        k++;
        v[k] = q;
        z[k] = s;
        z[k + 1] = +GMAPPING_INF;
    }

    // Fill in the distance values from the lower envelope
    int j = 0;
    for (int q = 0; q < n; ++q) {
        while (z[j + 1] < q) {
            j++;
        }
        result[q] = ((q - v[j])*(q - v[j])) + inDists[v[j]];
    }
}

// (Step 3)
// Check each position just around the particle along 1D dimensions; if the scan would
//   better "match" that position, move the particle in that direction. Nudge amount is
//   based on noise. Note: position will be further fine tuned via SampleParticles().
void Gmapping::NudgeParticle(LogField* logField) {

    // original, +x, -x, +y, -y, +t, -t
    std::vector<double> scores(7);
    for(int round = 0; round < GMAPPING_SCAN_MATCHING_MAX_NUDGES; round++) {
        scores[0] = this->ScoreRelativePosition(logField, 0, 0, 0);
        scores[1] = this->ScoreRelativePosition(logField, GMAPPING_SCAN_MATCHING_NUDGE_JUMP, 0, 0);
        scores[2] = this->ScoreRelativePosition(logField, -GMAPPING_SCAN_MATCHING_NUDGE_JUMP, 0, 0);
        scores[3] = this->ScoreRelativePosition(logField, 0, GMAPPING_SCAN_MATCHING_NUDGE_JUMP, 0);
        scores[4] = this->ScoreRelativePosition(logField, 0, -GMAPPING_SCAN_MATCHING_NUDGE_JUMP, 0);
        scores[5] = this->ScoreRelativePosition(logField, 0, 0, GMAPPING_SCAN_MATCHING_NUDGE_TWIST);
        scores[6] = this->ScoreRelativePosition(logField, 0, 0, -GMAPPING_SCAN_MATCHING_NUDGE_TWIST);

        int index = 0;
        double currMax = scores[0];
        for(int i = 1; i < 7; i++) {
            if(scores[i] > currMax) {
                currMax = scores[i];
                index = i;
            }
        }

        if(index == 0) {
            break;
        }

        // Nudge kept separate from pose for laser adjustment
        switch(index) {
            case 0:
                break;
            case 1:
                logField->currParticle->nudgeX += GMAPPING_SCAN_MATCHING_NUDGE_JUMP;
                break;
            case 2:
                logField->currParticle->nudgeX -= GMAPPING_SCAN_MATCHING_NUDGE_JUMP;
                break;
            case 3:
                logField->currParticle->nudgeY += GMAPPING_SCAN_MATCHING_NUDGE_JUMP;
                break;
            case 4:
                logField->currParticle->nudgeY -= GMAPPING_SCAN_MATCHING_NUDGE_JUMP;
                break;
            case 5:
                logField->currParticle->nudgeTheta += GMAPPING_SCAN_MATCHING_NUDGE_TWIST;
                break;
            default:
                logField->currParticle->nudgeTheta -= GMAPPING_SCAN_MATCHING_NUDGE_TWIST;
                break;
        }
    }
}

// (Step 4)
// Finds covariance matrix L(k), representing the sums of:
//  - P(Zt | Xt, m)
//  - P(Xt | Xt-1, u)
//
// For every combination of +-x/y/theta. Each probability gives that sample a "weight".
//   Final pose for the particle is a weighted average between all samples probabilties.
//   Note: Calculating e^-450 is far beyond the width of a double. Scores are "normalized"
//   by adding the highest log amount to each sample. Particles are then given a final
//   weight based on their samples average weight. (Representing Mu)
void Gmapping::SampleParticles(LogField* logField) {
    //neighbors to sample
    double tTerms[3] = {0, GMAPPING_SCAN_MATCHING_NUDGE_JUMP, -GMAPPING_SCAN_MATCHING_NUDGE_JUMP};
    double rTerms[3] = {0, GMAPPING_SCAN_MATCHING_NUDGE_TWIST, -GMAPPING_SCAN_MATCHING_NUDGE_TWIST};

    //calc P(Xt | Xt-1, u)
    //calc P(Zt | Xt, m)
    double maxPower = -DBL_MAX;
    std::vector<double> motionScore(27);
    std::vector<double> scanScore(27);
    int index;
    for(int i = 0; i < 3; i++) { // theta
        for(int j = 0; j < 3; j++) { // y
            for(int k = 0; k < 3; k++) { // x
                index = (i * 9) + (j * 3) + k;
                motionScore[index] = this->ScoreParticlePose(logField, tTerms[k], tTerms[j], rTerms[i]);
                scanScore[index] = this->ScoreRelativePosition(logField, tTerms[k], tTerms[j], rTerms[i]);
                maxPower = (std::max)(maxPower, scanScore[index]);
            }
        }
    }

    //sorta normalize with maxPower; brings probabilities into calculable range
    for(int i = 0; i < 27; i++) {
        scanScore[i] -= maxPower;
    }

    double startTheta = logField->currParticle->theta + logField->currParticle->nudgeTheta;
    
    double totalWeight = 0.0;
    double sampleWeight = 0.0;
    double totalX = 0.0;
    double totalY = 0.0;
    double totalSinTheta = 0.0;
    double totalCosTheta = 0.0;

    for(int i = 0; i < 3; i++) { // theta
        for(int j = 0; j < 3; j++) { // y
            for(int k = 0; k < 3; k++) { // x
                index = (i * 9) + (j * 3) + k;

                scanScore[index] = std::exp(scanScore[index]);
                motionScore[index] = std::exp(motionScore[index]);

                sampleWeight = scanScore[index]*motionScore[index];
                totalWeight += sampleWeight;

                totalX += sampleWeight * (logField->currParticle->x + logField->currParticle->nudgeX + tTerms[k]);
                totalY += sampleWeight * (logField->currParticle->y + logField->currParticle->nudgeY + tTerms[j]);

                totalSinTheta += sampleWeight * std::sin(startTheta + rTerms[i]);
                totalCosTheta += sampleWeight * std::cos(startTheta + rTerms[i]);
            }
        }
    }

    if(totalWeight > 0) {
        // (Step 7)
        logField->currParticle->x = totalX / totalWeight;
        logField->currParticle->y = totalY / totalWeight;
        logField->currParticle->theta = std::atan2(totalSinTheta, totalCosTheta);

        // (Step 5)
        logField->currParticle->weight *= totalWeight;
    }
}

// Based on total odometry reading, how far it should have moved/twisted, calculates the
//   probability the particle could be at that particular pose. Based on Mahalanobis
//   distance, using equation e^(-0.5*(d^2 / (2*sigma^2))). Theta is turned into rotational error,
//   and compounded into both distance and sigma, so:
//
//   e^(-0.5*((d^2 / (2*sigma^2)) + (atan2(sin(thetaDiff), cos(thetaDiff)))))
double Gmapping::ScoreParticlePose(LogField* logField, double deltaX, double deltaY, double deltaTheta) {
    // based on mahalanobis distance: e^-(diff^2 / 2*sigma^2)
    double xDiff = logField->currParticle->currScanX - logField->currParticle->oldScanX + deltaX + logField->currParticle->nudgeX;
    double yDiff = logField->currParticle->currScanY - logField->currParticle->oldScanY + deltaY + logField->currParticle->nudgeY;
    double deltaDist = std::sqrt(xDiff*xDiff + yDiff*yDiff);

    double distDiff = deltaDist - this->currPacket->expDist;

    double thetaTravelled = logField->currParticle->currScanTheta - logField->currParticle->oldScanTheta + deltaTheta + logField->currParticle->nudgeTheta;
    double thetaDiff = thetaTravelled - (this->currPacket->expTheta);
    double thetaError = std::atan2(std::sin(thetaDiff), std::cos(thetaDiff));

    // sigmas have a floor of base_deviation. The model tends to converge better with at least some noise.
    double distSigma = deltaDist * MOTION_MODEL_FORWARD_DEVIATION + MOTION_MODEL_BASE_FORWARD_DEVIATION;
    double sigmaRotation = (thetaTravelled * MOTION_MODEL_ROTATION);
    double sigmaVeer = (deltaDist * MOTION_MODEL_FORWARD_ROTATION_DEVIATION);
    double thetaSigma = std::sqrt(sigmaRotation*sigmaRotation + sigmaVeer*sigmaVeer + MOTION_MODEL_BASE_ROTATION_DEVIATION*MOTION_MODEL_BASE_ROTATION_DEVIATION);

    return -0.5 * (((distDiff*distDiff) / (distSigma*distSigma)) + ((thetaError*thetaError) / (thetaSigma*thetaSigma)));
}

// For every point within a lidar cloud, alter it according to nudge amount. Count each
//   point who's cell represents a wall (above threshold). Only counts every Xth laser.
//   Based on Mahalanobis distance.
// Note: If a point isn't found on the grid, it isn't handled. I haven't run into an issue,
//   though you think it'd happen at some point.
double Gmapping::ScoreRelativePosition(LogField* logField, double deltaX, double deltaY, double deltaTheta) {
    //given a set of points and a set of changes,
    //   alter all the points in the cloud accordingly and count up the score

    // Based on mahalanobis distance: e^-(d^2 / (2*sigma^2)). Note, the e^ part isn't calculated.
    // The log of that can be added together to get the same idea. We just need direction, we save
    // the log calculations for elsewhere.

    double score = 0;
    double cosTheta = std::cos(deltaTheta + logField->currParticle->nudgeTheta);
    double sinTheta = std::sin(deltaTheta + logField->currParticle->nudgeTheta);
    double x, y, tempX;
    int invScanPercent = (int)(1.0 / GMAPPING_SCAN_MATCHING_PERCENT_LASERS_USED);
    double invCellSize = 1.0 / GMAPPING_GRID_CELL_SIZE;
    for(int i = 0; i < logField->numPoints; i += invScanPercent) {
        x = (logField->cartesianPoints[i]->x);
        y = (logField->cartesianPoints[i]->y);

        if((deltaTheta + logField->currParticle->nudgeTheta) != 0) {
            x -= (logField->poses[i]->x);
            y -= (logField->poses[i]->y);

            tempX = x;
            x = tempX*cosTheta - y*sinTheta;
            y = tempX*sinTheta + y*cosTheta;

            x += (logField->poses[i]->x);
            y += (logField->poses[i]->y);
        }


        x += deltaX + logField->currParticle->nudgeX;
        y += deltaY + logField->currParticle->nudgeY;

        //convert to likelihood field from cartesian world
        x += logField->offsetX;
        y += logField->offsetY;

        score += logField->likelihoodField[(int)((std::floor(y * invCellSize)*(logField->cellsWidth)) + std::floor(x * invCellSize))] * logField->inverseSigmas[i];
    }

    return score;
}

// (Step 9 pt. 1, Step 10)
// Choose with replacement (uses comb randomness; guarantees the best particles
//  get chosen at least once). Determines which particles are replaced and which are kept.
void Gmapping::FillParticlesToRefresh() {

    // Pick particles for refresh
    double offset = 1.0 / GMAPPING_NUM_PARTICLES;
    double position = Utilities::GetUniformEpsilon(offset);

    Particle** particles = this->currPacket->particles;
    double currWeightMass = particles[0]->weight;
    int particleNum = 0;

    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        while(currWeightMass <= position) {
            particleNum++;
            currWeightMass += particles[particleNum]->weight;
        }

        this->particlesToRefresh[i] = particleNum;
        position += offset;
    }

    // Determine particle assignment
    std::vector<int> unassigned; //particles not yet assigned a location
    std::vector<int> unfulfilled; //simply nums 0-numparticles that aren't in particlesToRefresh
    unassigned.reserve(8);
    unfulfilled.reserve(8);

    int checkPosition = 0;

    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        while(i > this->particlesToRefresh[checkPosition]) {
            unassigned.push_back(checkPosition);
            checkPosition++;
            if(checkPosition == GMAPPING_NUM_PARTICLES) {
                break;
            }
        }

        //edge case
        if(checkPosition == GMAPPING_NUM_PARTICLES) {
            while(i < GMAPPING_NUM_PARTICLES) {
                unfulfilled.push_back(i);
                i++;
            }
            break;
        }

        if(i < this->particlesToRefresh[checkPosition]) {
            unfulfilled.push_back(i);
        } else if(i == this->particlesToRefresh[checkPosition]) {
            checkPosition++;
        }
    }

    //edge case
    while(checkPosition < GMAPPING_NUM_PARTICLES) {
        unassigned.push_back(checkPosition);
        checkPosition++;
    }

    // Run through the other particles and clear their occupancy maps, as well as
    //  add the new one in. Just need to add maps; later, when we refresh, we
    //  change the other values
    for(int i = 0; i < ((int)(unfulfilled.size())); i++) {
        this->currPacket->particles[unfulfilled[i]]->map->ClearSectors();
        this->currPacket->particles[unfulfilled[i]]->map->FillSectors(this->currPacket->particles[this->particlesToRefresh[unassigned[i]]]->map->GetSectors());
    }

    // (Step 11)
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        this->currPacket->particles[i]->weight = GMAPPING_STARTING_WEIGHT;
    }

    this->refreshParticles = true;
}

// (Step 9 pt. 2)
// Finish changing values such that old particles replicate new particles data.
//   This is in a different function, different place for thread safety.
void Gmapping::FlushRefreshedParticles() {
    Particle* sourceParticle;
    Particle* destinationParticle;
    for(int i = 0; i < GMAPPING_NUM_PARTICLES; i++) {
        destinationParticle = this->particles[i];
        sourceParticle = this->particles[this->particlesToRefresh[i]];

        destinationParticle->oldScanX = sourceParticle->oldScanX;
        destinationParticle->oldScanY = sourceParticle->oldScanY;
        destinationParticle->oldScanTheta = sourceParticle->oldScanTheta;
        
        destinationParticle->currScanX = sourceParticle->currScanX;
        destinationParticle->currScanY = sourceParticle->currScanY;
        destinationParticle->currScanTheta = sourceParticle->currScanTheta;

        destinationParticle->x = sourceParticle->x;
        destinationParticle->y = sourceParticle->y;
        destinationParticle->theta = sourceParticle->theta;
    }
}
