#ifndef WORLD_MAP_H_
#define WORLD_MAP_H_

#include "Objects\oline.h"
#include "Objects\opoint.h"

class Map {
public:

private:
    int id = 0;

    int linesSize = -1;
    int startsSize = -1;

    OLine** lines = nullptr;
    OPoint** starts = nullptr;

public:
    Map(int id);
    ~Map();

    int GetLinesSize();
    OLine** GetLines();

    int GetStartsSize();
    OPoint** GetStarts();
    OPoint* GetStart(int index);

    void ReadMap(int id);
    void TestMap();
};

#endif