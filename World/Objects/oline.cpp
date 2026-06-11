#include <iostream>

#include "oline.h"
#include "opoint.h"

OLine::OLine(OPoint* point1, OPoint* point2) {
    this->point1 = point1;
    this->point2 = point2;
}

OLine::~OLine() {
    delete this->point1;
    delete this->point2;
}

// includes new line
void OLine::Print() {
    this->point1->Print();
    std::cout << ", ";
    this->point2->Print();
    std::cout << std::endl;
}