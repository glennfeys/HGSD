#ifndef DVRPD_H
#define DVRPD_H

#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include "Drone.h"
#include "Vehicle.h"
#include "Location.h"
#include <bits/stdc++.h>
#include <unordered_set>

#define MAP_WIDTH 10
#define MAP_HEIGHT 10

#define MAX_X 1000
#define MAX_Y 1000

#define DRONE_ELIGIBLE 80

using namespace std;

class DVRPD
{
private:
    vector<Drone*> drones;
    vector<Vehicle*> vehicles;
    unordered_set<int> drone_unelgible;

    vector<Location*> locations;
    vector<float> capacities;
    
public:
    static const int droneEligible = DRONE_ELIGIBLE;

    unordered_set<int> free_ids;
    vector<float*> distance_matrix;
    Location* depot;
    unordered_set<int>* location_map;

    DVRPD(unsigned int drones, unsigned int vehicles);
    DVRPD(string path, unsigned int drones, unsigned int vehicles);
    ~DVRPD();

    Location* addLocation(float x, float y);
    float addCapacity(unsigned int index, float capacity);
    void removeLocation(int loc);
    vector<Location*> getLocations();
    vector<Vehicle*> getVehicles();
    vector<Drone*> getDrones();
    void createRandomLocations(unsigned int amount, float max_x, float max_y);
    void add_rand_location(float max_x, float max_y);
    unordered_set<int> getNeighbours(int index, int range);
    int getRandomNeighbour(int loc, int range);
    float distance(int a, int b);
    unordered_set<int> getDroneUneligible();
    unordered_set<int> getDroneUneligibleCapacity();
};

#endif
