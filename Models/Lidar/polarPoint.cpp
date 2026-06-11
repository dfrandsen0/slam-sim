#include <iostream>

#include "polarPoint.h"

PolarPoint::PolarPoint(double range, double theta) {
    this->range = range;
    this->theta = theta;
}

PolarPoint::~PolarPoint() {

}

// doesn't include newline.
// (r, theta)
void PolarPoint::Print() {
    std::cout << "(" << this->range << ", " << this->theta << ")";
}

PolarPoint* PolarPoint::Copy() {
    return new PolarPoint(this->range, this->theta);
}