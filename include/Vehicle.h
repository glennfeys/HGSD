#ifndef VEHICLE_H
#define VEHICLE_H

#include <stdlib.h>
#include <stdio.h>

#define VEHICLE_SPEED 30 // 30 km/h
#define VEHICLE_CAPACITY 150
#define VEHICLE_LOAD_TIME 0.5 // 900 seconds = 15 min
#define DROP_TIME 0.03 // 1.8 min
#define VEHICLE_MAX_TIME 8 // 7200 seconds = 2 hours 
// TODO drop time, ...

using namespace std;

class Vehicle
{
private:
    /* data */
    
public:
    static const unsigned int maxCapacity = VEHICLE_CAPACITY;
    static const unsigned int speed = VEHICLE_SPEED;
    static const unsigned int maxT = VEHICLE_MAX_TIME;

    static const unsigned int dropTime = DROP_TIME;
    static const unsigned int loadTime = VEHICLE_LOAD_TIME;

    Vehicle();
    ~Vehicle();
};

#endif