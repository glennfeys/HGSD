#include "Location.h"

Location::Location(int id, float x, float y) {
    this->id = id;
    this->x = x;
    this->y = y;
}

Location::~Location(){}

float Location::distance(Location* b) {
    return sqrt(pow(b->x - this->x, 2.0) + pow(b->y - this->y, 2.0));
}

void Location::print(ostream& file) {
    file << "(" << this->x << ", " << this->y << ")";
}