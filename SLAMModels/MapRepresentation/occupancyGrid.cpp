#include <unordered_map>
#include <iostream>
#include <cmath>
#include <vector>

#include "occupancyGrid.h"
#include "sector.h"

OccupancyGrid::OccupancyGrid() {

}

OccupancyGrid::~OccupancyGrid() {
    //TODO what goes here?
}

void OccupancyGrid::AddSector(int x, int y, Sector* sector) {
    this->grid[{x, y}] = sector;
}

void OccupancyGrid::RemoveSector(int x, int y) {
    this->grid.erase({x, y});
}

bool OccupancyGrid::SectorExists(int x, int y) {
    if(this->grid.find({x, y}) != this->grid.end()) {
        return true;
    } else {
        return false;
    }
}

//returns nullptr if it doesn't exist
Sector* OccupancyGrid::GetSector(int x, int y) {
    if(this->grid.find({x, y}) != this->grid.end()) {
        return this->grid[{x, y}];
    } else {
        return nullptr;
    }
}

void OccupancyGrid::ClearSectors() {
    for(const std::pair<std::pair<int, int>, Sector*> match : this->grid) {
        match.second->RemoveReference();
    }

    this->grid.clear();
}

std::vector<std::pair<std::pair<int, int>, Sector*>> OccupancyGrid::GetSectors() {
    std::vector<std::pair<std::pair<int, int>, Sector*>> references;
    references.reserve(this->grid.size());

    for(const std::pair<const std::pair<int, int>, Sector*>& match : this->grid) {
        references.push_back({{match.first.first, match.first.second}, match.second});
    }

    return references;
}

// make sure previous sectors are already cleared!
void OccupancyGrid::FillSectors(const std::vector<std::pair<std::pair<int, int>, Sector*>>& incomingSectors) {
    for(const std::pair<std::pair<int, int>, Sector*>& match : incomingSectors) {
        this->grid[{match.first.first, match.first.second}] = match.second;
        match.second->AddReference();
    }
}

void OccupancyGrid::GetRowWalls(std::vector<double>& values, int sectorX, int sectorY, int row) {
    values.clear();

    if(this->grid.find({sectorX, sectorY}) == this->grid.end()) {
        for(int i = 0; i < GMAPPING_SECTOR_SIZE; i++) {
            values.push_back(GMAPPING_INF);
        }
        return;
    }

    Sector* thisSector = this->grid[{sectorX, sectorY}];

    int cellIndex = GMAPPING_SECTOR_SIZE * row;
    for(int i = 0; i < GMAPPING_SECTOR_SIZE; i++) {
        if(thisSector->cells[cellIndex] >= GMAPPING_LOG_ODDS_WALL_VALUE) {
            values.push_back(0);
        } else {
            values.push_back(GMAPPING_INF);
        }

        cellIndex++;
    }
}

//either log odds hit or miss
void OccupancyGrid::ChangeCell(int cellX, int cellY, double value) {
    int sectorX = (int)std::floor((double)cellX / (double)GMAPPING_SECTOR_SIZE);
    int sectorY = (int)std::floor((double)cellY / (double)GMAPPING_SECTOR_SIZE);
    std::unordered_map<std::pair<int, int>, Sector*>::iterator it = this->grid.find({sectorX, sectorY});

    if(it == this->grid.end()) {
        Sector* newSector = new Sector();
        this->AddSector(sectorX, sectorY, newSector);
        newSector->AddReference();
        newSector->ChangeCellValue(cellX % GMAPPING_SECTOR_SIZE, cellY % GMAPPING_SECTOR_SIZE, value);

        return;
    }

    if(it->second->GetReferenceCount() == 1) {
        it->second->ChangeCellValue(cellX % GMAPPING_SECTOR_SIZE, cellY % GMAPPING_SECTOR_SIZE, value);
    } else {
        Sector* oldSector = it->second;
        Sector* newSector = oldSector->Copy();
        newSector->AddReference();
        newSector->ChangeCellValue(cellX % GMAPPING_SECTOR_SIZE, cellY % GMAPPING_SECTOR_SIZE, value);
        this->RemoveSector(sectorX, sectorY);
        oldSector->RemoveReference();
        this->AddSector(sectorX, sectorY, newSector); //this line was missing?
    }
}

std::unordered_map<std::pair<int, int>, Sector*, pair_hash>* OccupancyGrid::GetCells() {
    return &(this->grid);
}

