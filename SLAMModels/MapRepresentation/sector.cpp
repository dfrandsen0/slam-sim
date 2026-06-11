#include <cstring>
#include <cmath>

#include "sector.h"
#include "..\..\config.h"

Sector::Sector() {
    this->refCount = 0;
    this->cells = new double[GMAPPING_SECTOR_NUM_CELLS]();
}

Sector::~Sector() {
    delete[] this->cells;
}

void Sector::AddReference() {
    this->refCount += 1;
}

// do not call if you still need access to this sector!
void Sector::RemoveReference() {
    this->refCount -= 1;
    if(this->refCount == 0) {
        delete this;
    }
}

int Sector::GetReferenceCount() {
    return this->refCount;
}

void Sector::ChangeCellValue(int cellX, int cellY, double value) {
    if(cellX < 0) {
        cellX += GMAPPING_SECTOR_SIZE;
    }

    if(cellY < 0) {
        cellY += GMAPPING_SECTOR_SIZE;
    }

    int cellIndex = (GMAPPING_SECTOR_SIZE * cellY) + cellX;

    this->cells[cellIndex] += value;
    if(this->cells[cellIndex] > GMAPPING_MAX_LOG_ODDS) {
        this->cells[cellIndex] = GMAPPING_MAX_LOG_ODDS;
    } else if(this->cells[cellIndex] < -GMAPPING_MAX_LOG_ODDS) {
        this->cells[cellIndex] = -GMAPPING_MAX_LOG_ODDS;
    }
}

double Sector::GetCellValue(int cellX, int cellY) {
    return this->cells[(GMAPPING_SECTOR_SIZE * std::abs(cellY)) + std::abs(cellX)];
}

// Does not change reference counts (for copy or original)
Sector* Sector::Copy() {
    Sector* newSector = new Sector();
    memcpy(newSector->cells, this->cells, GMAPPING_SECTOR_NUM_CELLS*sizeof(double));
    return newSector;
}
