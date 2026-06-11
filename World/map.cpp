#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "map.h"
#include "Objects\oline.h"
#include "Objects\opoint.h"
#include "..\config.h"

Map::Map(int id) {
    this->lines = new OLine*[MAX_MAP_LINES];
    this->linesSize = 0;

    for(int i = 0; i < MAX_MAP_LINES; i++) {
        this->lines[i] = nullptr;
    }

    this->starts = new OPoint*[MAX_MAP_STARTS];
    this->startsSize = 0;

    for(int i = 0; i < MAX_MAP_STARTS; i++) {
        this->starts[i] = nullptr;
    }

    this->ReadMap(id);
}

Map::~Map() {
    if(this->linesSize > 0) {
        for(int i = 0; i < this->linesSize; i++) {
            delete this->lines[i];
        }
    }

    if(this->linesSize != -1) {
        delete[] this->lines;
    }
}

int Map::GetLinesSize() {
    return this->linesSize;
}

OLine** Map::GetLines() {
    return this->lines;
}

int Map::GetStartsSize() {
    return this->startsSize;
}

OPoint** Map::GetStarts() {
    return this->starts;
}

OPoint* Map::GetStart(int index) {
    return this->starts[index];
}

void Map::ReadMap(int id) {
    if(this->id != 0) {
        for(int i = 0; i < this->linesSize; i++) {
            delete this->lines[i];
            this->lines[i] = nullptr;
        }

        this->linesSize = 0;

        for(int i = 0; i < this->startsSize; i++) {
            delete this->starts[i];
            this->starts[i] = nullptr;
        }

        this->startsSize = 0;
    }

    this->id = id;
    std::string filename = "Resources\\Maps\\map" + std::to_string(id) + ".txt";

    std::ifstream file(filename, std::ios::binary);
    if(!file.is_open()) {
        std::cout << "couldn't open file " << filename << std::endl;
        exit(1);
    }

    std::string line;
    OPoint* tempPoint1, * tempPoint2;
    OLine* tempLine;
    double tempVals[4];

    while(std::getline(file, line)) {
        line.erase(line.find_last_not_of(" \t\n\r") + 1);
        
        if(line == "starts") {
            break;
        }

        if(line.empty()) {
            continue;
        }

        std::stringstream ss(line);
        ss >> tempVals[0] >> tempVals[1] >> tempVals[2] >> tempVals[3];

        tempPoint1 = new OPoint(tempVals[0], tempVals[1]);
        tempPoint2 = new OPoint(tempVals[2], tempVals[3]);
        tempLine = new OLine(tempPoint1, tempPoint2);
        this->lines[this->linesSize] = tempLine;
        (this->linesSize)++;
    }

    while(std::getline(file, line)) {
        if(line.empty()) {
            continue;
        }

        std::stringstream ss(line);
        ss >> tempVals[0] >> tempVals[1];

        tempPoint1 = new OPoint(tempVals[0], tempVals[1]);
        this->starts[this->startsSize] = tempPoint1;
        (this->startsSize)++;
    }

    file.close();
    // this->TestMap();
}

void Map::TestMap() {
    std::cout << "testing map..." << std::endl;
    std::cout << "num lines: " << this->linesSize << std::endl;

    for(int i = 0; i < this->linesSize; i++) {
        this->lines[i]->Print();
    }

    for(int i = 0; i < this->startsSize; i++) {
        this->starts[i]->Print();
        std::cout << std::endl;
    }
}