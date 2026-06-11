#ifndef SLAM_MODELS_GMAPPING_LOG_FIELD_H_
#define SLAM_MODELS_GMAPPING_LOG_FIELD_H_

#include <vector>

#include "..\..\Models\Lidar\polarPoint.h"
#include "..\..\World\Objects\opoint.h"
#include "particle.h"

struct LogField {
public:
    LogField();
    ~LogField();

    std::vector<double> likelihoodField;
    PolarPoint** cloud = nullptr;
    double* inverseSigmas = nullptr;
    Particle* currParticle = nullptr;

    OPoint** cartesianPoints = nullptr;
    OPoint** poses = nullptr;

    int numPoints = 0;

    int sectorMinX = 0.0;
    int sectorMinY = 0.0;
    int sectorMaxX = 0.0;
    int sectorMaxY = 0.0;

    int sectorsWidth = 0.0;
    int sectorsHeight = 0.0;
    int cellsWidth = 0.0;
    int cellsHeight = 0.0;

    int offsetX = 0.0;
    int offsetY = 0.0;

    double expDist = 0.0;
    double expTheta = 0.0;
};

#endif