#ifndef SLAM_MODELS_GMAPPING_MAP_REPRESENTATION_SECTOR_H_
#define SLAM_MODELS_GMAPPING_MAP_REPRESENTATION_SECTOR_H_

struct Sector {
public:
    Sector();
    ~Sector();

    int refCount = 0;
    double* cells = nullptr;

    void AddReference();
    void RemoveReference();
    int GetReferenceCount();
    
    void ChangeCellValue(int cellX, int cellY, double value);
    double GetCellValue(int cellX, int cellY);

    Sector* Copy();
};

#endif