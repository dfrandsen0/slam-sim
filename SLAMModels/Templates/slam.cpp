#include <iostream>
#include <vector>
#include <mutex>
#include <memory>
#include <thread>

#include "slam.h"
#include "..\MapRepresentation\poseRenderPacket.h"

SLAMModule::SLAMModule() {
    
}

SLAMModule::~SLAMModule() {
    
}

// sets start x/y/theta for rendering purposes only. Sets slam as waiting and starts thread.
void SLAMModule::InitSlam(double startX, double startY, double startTheta) {
    this->startX = startX;
    this->startY = startY;
    this->startTheta = startTheta;

    this->slamFinished = true;

    this->slamSemaphore = CreateSemaphore(NULL, 0, 1, NULL);
    this->slamThread = std::thread(RefineEstimates, this);
}

// Gets render poses lock. Does NOT lock mutex.
std::vector<float>** SLAMModule::GetRenderMapAddress() {
    return &(this->renderMap);
}

// Gets render poses map.
std::shared_ptr<std::mutex> SLAMModule::GetRenderMapGuard() {
    return this->guardRenderMap;
}

// Get regular top-left poses. Should be refactored to be more generalizable.
PoseRenderPacket* SLAMModule::GetPoses() {
    std::lock_guard<std::mutex> lock(this->guardPoses);
    if(this->poses == nullptr) {
        return nullptr;
    }
    
    PoseRenderPacket* packet = this->poses->Copy();
    return packet;
}

// Get extended poses. Should be changed to generalized slam. Extended are bottom right poses.
PoseRenderPacket* SLAMModule::GetExtendedPoses() {
    std::lock_guard<std::mutex> lock(this->guardPoses);
    if(this->extendedPoses == nullptr) {
        return nullptr;
    }

    PoseRenderPacket* packet = this->extendedPoses->Copy();
    return packet;
}

// Replace poses. Should be changed to render packet and made slam generalized.
void SLAMModule::ReplacePoses(PoseRenderPacket* newPacket, PoseRenderPacket* newExtendedPacket) {
    delete this->poses;
    this->poses = newPacket;

    delete this->extendedPoses;
    this->extendedPoses = newExtendedPacket;
}

// Gets particle efficiency. Should be factored out eventually.
double SLAMModule::GetNeff() {
    std::lock_guard<std::mutex> lock(this->guardNeff);
    return this->neff;
}

// Basic slam algorithm loop. Refine all estimations here.
void SLAMModule::RefineEstimates() {
    for(;;) {
        //TODO add way to destroy

        WaitForSingleObject(this->slamSemaphore, INFINITE);

        this->RunSlam();

        this->slamFinished = true;
    }
}

// From occupancy grid cell x, y to X, Y, add final cell to hits and all others to misses.
void SLAMModule::AddAffectedCells(int startX, int startY, int endX, int endY, std::unordered_set<std::pair<int, int>, pair_hash>& misses, std::unordered_set<std::pair<int, int>, pair_hash>& hits) {
    int dx = std::abs(endX - startX);
    int dy = std::abs(endY - startY);

    int stepX = 1;
    if(startX >= endX) {
        stepX = -1;
    }

    int stepY = 1;
    if(startY >= endY) {
        stepY = -1;
    }

    int error = dx - dy;
    int error2;

    int x = startX;
    int y = startY;

    for(;;) {
        if((x == endX) && (y == endY)) {
            hits.insert({x, y});
            break;
        }

        misses.insert({x, y});
        error2 = 2 * error;

        if(error2 > -dy) {
            error -= dy;
            x += stepX;
        }

        if(error2 < dx) {
            error += dx;
            y += stepY;
        }
    }
}

// Updates Neff after obtaining lock
void SLAMModule::UpdateNeff(double newNeff) {
    std::lock_guard<std::mutex> lock(this->guardNeff);
    this->neff = newNeff;
}

// (Step 13 of Gmapping)
// Given a set of sectors associated with a map, find the DIP location of each
//   and mark it as a pose to give to graphics for rendering.
void SLAMModule::CreateRenderCopy(std::unordered_map<std::pair<int, int>, Sector*, pair_hash>* model) {
    std::vector<float>* newRenderMap = new std::vector<float>();
    newRenderMap->reserve(6000);

    int sectorX, sectorY;
    double mapX, mapY, worldX, worldY;
    std::pair<float, float> screenCords;
    double cosTheta = std::cos(this->startTheta);
    double sinTheta = std::sin(this->startTheta);

    for(std::pair<std::pair<int, int>, Sector*> sectorInfo : *model) {
        sectorX = sectorInfo.first.first * GMAPPING_SECTOR_SIZE;
        sectorY = sectorInfo.first.second * GMAPPING_SECTOR_SIZE;

        for(int i = 0; i < GMAPPING_SECTOR_NUM_CELLS; i++) {
            if(sectorInfo.second->cells[i] <= 0) {
                continue;
            }

            mapX = (sectorX + (i % GMAPPING_SECTOR_SIZE)) * GMAPPING_GRID_CELL_SIZE;
            mapY = (sectorY + (i / GMAPPING_SECTOR_SIZE)) * GMAPPING_GRID_CELL_SIZE;
            worldX = (mapX*cosTheta) - (mapY*sinTheta) + this->startX;
            worldY = (mapX*sinTheta) + (mapY*cosTheta) + this->startY;

            screenCords = this->XYToDips(worldX, worldY);

            newRenderMap->push_back(screenCords.first);
            newRenderMap->push_back(screenCords.second);
            newRenderMap->push_back((float)(sectorInfo.second->cells[i]));
        }
    }

    std::lock_guard<std::mutex> lock(*(this->guardRenderMap));
    delete this->renderMap;
    this->renderMap = newRenderMap;
}

// Converts world x/y to screen space. Note: y-axis in screen space is flipped.
std::pair<float, float> SLAMModule::XYToDips(double x, double y) {
    return {
        (CLIENT_SCREEN_WIDTH * 0.25) + (x / MM_PER_DIP) + 1,
        (CLIENT_SCREEN_HEIGHT * 0.25) + (y / MM_PER_DIP * -1) + 1
    };
}