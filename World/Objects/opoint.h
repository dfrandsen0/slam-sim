#ifndef WORLD_OBJECTS_OPOINT_H_
#define WORLD_OBJECTS_OPOINT_H_

struct OPoint {
public:
    OPoint(double x, double y);
    ~OPoint();

    double x;
    double y;

    void Print();
    OPoint* Copy();
};

#endif