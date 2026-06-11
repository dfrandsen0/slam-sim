#include "world.h"
#include "map.h"
#include "..\config.h"

World::World() {
    this->map = new Map(STARTING_MAP);
}

World::~World() {
    delete this->map;
}

Map* World::GetMap() {
    return this->map;
}