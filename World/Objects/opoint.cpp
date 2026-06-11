#include <iostream>

#include "opoint.h"

OPoint::OPoint(double x, double y) {
    this->x = x;
    this->y = y;
}

OPoint::~OPoint() {

}

// No new line
void OPoint::Print() {
    std::cout << "(" << this->x << ", " << this->y << ")";
}

OPoint* OPoint::Copy() {
    return new OPoint(this->x, this->y);
}