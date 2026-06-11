#ifndef SLAM_MODELS_GMAPPING_MAP_REPRESENTATION_OCCUPANCY_GRID_H_
#define SLAM_MODELS_GMAPPING_MAP_REPRESENTATION_OCCUPANCY_GRID_H_

#include <unordered_map>
#include <utility>
#include <vector>

#include "sector.h"
#include "..\..\config.h"

struct pair_hash {
    size_t operator()(const std::pair<int, int>& v) const {
        size_t seed = 0;
        seed ^= std::hash<int>{}(v.first) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>{}(v.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

class OccupancyGrid {
public:
private:

public:
    OccupancyGrid();
    ~OccupancyGrid();

    void AddSector(int x, int y, Sector* sector);
    void RemoveSector(int x, int y);
    bool SectorExists(int x, int y);
    Sector* GetSector(int x, int y);

    void ClearSectors();
    std::vector<std::pair<std::pair<int, int>, Sector*>> GetSectors();
    void FillSectors(const std::vector<std::pair<std::pair<int, int>, Sector*>>& incomingSectors);

    void GetRowWalls(std::vector<double>& values, int sectorX, int sectorY, int row);

    std::unordered_map<std::pair<int, int>, Sector*, pair_hash>* GetCells();

    void ChangeCell(int cellX, int cellY, double value);


private:
    std::unordered_map<std::pair<int, int>, Sector*, pair_hash> grid;


};

#endif