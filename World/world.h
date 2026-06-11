#ifndef WORLD_H_
#define WORLD_H_

#include "map.h"

class World {
private:
    Map* map = nullptr;
public:
    World();
    ~World();

    Map* GetMap();
};

#endif