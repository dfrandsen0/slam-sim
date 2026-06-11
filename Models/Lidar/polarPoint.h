#ifndef MODELS_LIDAR_POLAR_POINT_H_
#define MODELS_LIDAR_POLAR_POINT_H_

struct PolarPoint {
public:
    double range;
    double theta;

    PolarPoint(double range, double theta);
    ~PolarPoint();

    void Print();
    PolarPoint* Copy();
};

#endif