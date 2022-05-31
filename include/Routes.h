#ifndef ROUTES_H
#define ROUTES_H

#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <bits/stdc++.h>
#include "Vehicle.h"
#include "Drone.h"
#include "DVRPD.h"
#include <map>
#include <chrono>

using namespace std;

struct RouteNode
{
    unsigned int location;
    int next;
    int prev;
};

struct Route {
    int start;
    float fitness;
    unsigned int length;
};

class Routes
{
    typedef Routes* (*mutationFunc)(Routes* mutation);

    private:
        
        
    public:
        vector<RouteNode> nodes;
        vector<int> free_locations;
        // todo kan dit geen array worden?
        vector<Route> routes;
        Routes* best1;
        Routes* best2;
        Routes* best3;
        DVRPD* vrp;
        int id;

        int last_mutation;

        float fitness;

        vector<int> location_route_map;

        Routes(DVRPD* vrp);
        ~Routes();

        float getBiasedFitness();
        float getFitness();
        float getFitnessRatio();
        float getFitnessRatio2();
        float getFitnessRatio3();
        float getFitnessRatio4();
        float getFitnessMakespan();
        float getBiasedFitnessRatio();
        float getFitnessDistance();
        float getFitnessDistMin();
        float getFitnessDistMinRatio();
        float getFitnessDistMin2();
        float getFitnessDistMinRatio2();
        int getFitnessAmount();
        Routes* cross_over(Routes* routes, Routes* mutation);
        Routes* mutate(Routes* mutation, bool clone);
        void remove_locations(unordered_set<int> locations);
        //void remove_locations2(unordered_set<int> locations);
        void clone_to(Routes* mutation);
        void print(ostream& file);
        bool check_routes();
        bool check_inf_loop();
        //bool check_doubles();
        bool check_locs(string name);
        bool check_maxT();
        float get_diversity(Routes* routes);

        void make_rnd_routes();
        void make_heuristic1_routes();
        void make_nn_routes();
        void make_empty_routes();
        void make_routes(vector<int>* rs);
        void insertAt(Route* route, int pos, RouteNode* begin, RouteNode* end);
        void addLocation(unsigned int loc);
        int next(int node);
        int prev(int node);
        int getPos(int n, Route& route);
        float getRouteLength(Route* route);
        int getCOprev(int n, unordered_set<int> locations, int end_sec, int end_sec_d);
        int getCOnext(int n, unordered_set<int> locations, int start_sec);
        int* routeToArray(Route& route);
        int getRouteLevenshteinDist(Route& route1, Route& route2, Routes* mutation);
        void getRouteIndex(int loc, int* route, int* index);

        void op_swap_free(unsigned int r_index, unsigned int index, unsigned int fl_index);
        void op_swap_pos(unsigned int r_index, unsigned int index, unsigned int len, unsigned int insert_pos);
        void op_cross_over_m(unsigned int r_index1, unsigned int index1, unsigned int len1, unsigned int r_index2, unsigned int index2, unsigned int len2);
        void op_add(unsigned int r_index, unsigned int index, unordered_set<int> locs);
        void op_remove(unsigned int r_index, unsigned int index, unsigned int amount);
        void op_reverse_part(unsigned int r_index, unsigned int start_index, unsigned int end_index);
        void op_cross(unsigned int r_index1, unsigned int index1, unsigned int r_index2, unsigned int index2);
        void op_cross_over(Routes* donor, Routes* acceptor, unsigned int r_index, unsigned int index, unsigned int len);

        void fill_location_route_map();
        void routes_from_file(string file);
        
};

#endif