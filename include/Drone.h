#ifndef DRONE_H
#define DRONE_H

#include <stdlib.h>
#include <stdio.h>

#define DRONE_SPEED 50 // 15 m/s = 54 km/h
#define DRONE_CAPACITY 1
#define DRONE_LOAD_TIME 0.75 // = 45 min (charging included)
#define DRONE_MAX_TIME 24 // = 24 hours
#define LOAD_TIME 0.05 // 900 seconds = 15 min
#define DROP_TIME 0.03 // 1.8 min
#define DRONE_SPEED_FACTOR 2
// TODO drop time, ...

using namespace std;

class Drone
{
private:
    
public:
    static const int maxCapacity = DRONE_CAPACITY;
    static const int speed = DRONE_SPEED;
    static const int maxT = DRONE_MAX_TIME;
    static const int loadTime = LOAD_TIME;
    static const int dropTime = DROP_TIME;
    static const int speedFactor = DRONE_SPEED_FACTOR;

    Drone();
    ~Drone();
};

#endif