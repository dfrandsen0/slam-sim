#ifndef WORLD_OBJECTS_OLINE_H_
#define WORLD_OBJECTS_OLINE_H_

#include "opoint.h"

struct OLine {
public:
    OLine(OPoint* point1, OPoint* point2);
    ~OLine();

    OPoint* point1 = nullptr;
    OPoint* point2 = nullptr;

    void Print();
};

#endif