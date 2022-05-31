#ifndef LOCATION_H
#define LOCATION_H

#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <bits/stdc++.h>

using namespace std;

class Location
{
private:
    
    
    
public:
    float x;
    float y;
    int id;
    //int index;
    Location(int id, float x, float y);
    ~Location();

    float distance(Location* b);
    float distance(int b, vector<float*>& distanceMatrix);
    static float distance(int a, int b, vector<float*>& distanceMatrix) {
        if (a == b) return 0.0;
        return distanceMatrix[max(a, b)][min(a, b)];
    }
    void print(ostream& file);
};

#endif