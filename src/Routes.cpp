#include "Routes.h"

Routes::Routes(DVRPD* vrp) {
    this->vrp = vrp;
    for (unsigned int i=0; i<vrp->getLocations().size(); i++) {
        this->nodes.push_back({i, -1, -1});
    }
    for (unsigned int i=0; i<vrp->getVehicles().size(); i++) {
        this->routes.push_back({0, -1.0f, 0});
    }

    //this->location_route_map = new int[this->vrp->getLocations().size()];
    for (int i=0; i< this->vrp->getLocations().size(); i++) {
        this->location_route_map.push_back(-1);
    }
}

void Routes::routes_from_file(string file) {
    string line;
    ifstream myfile(file);

    vector<vector<int>> rs;

    if (myfile.is_open()) {
        while (getline(myfile,line)){
            if(line.substr(0,7) == "Route #") {

                stringstream ss(line);
                string word;
                
                vector<int> r;

                int i = 0;

                while (ss >> word) {
                    if (i>1) {
                        r.push_back(stoi(word));
                    }
                    i++;
                }

                rs.push_back(r);
            }
        }
        myfile.close();
    } else cout << "Unable to open file"; 

    vector<int>* routes = &rs[0];

    this->make_routes(routes);

}

Routes::~Routes() {}

float Routes::getFitness() {
    float fit = getFitnessDistMinRatio2();
    this->fitness = fit;
    return fit;
}

float manhatten(Location* l1, Location* l2) {
    return abs(l1->x - l2->x) + abs(l1->y - l2->y);
}

float Routes::getFitnessDistMin() {
    float res = 0;

    Location* depot = vrp->depot;
    vector<Location*>& locs = vrp->getLocations();

    for (Route r : this->routes) {
        float fitness = 0.0;

        int curr = r.start;

        if(curr == 0) continue;

        fitness += manhatten(locs[curr], depot);
        

        int next = this->next(curr);

        while (next != 0) {
            if (next == -1) {
                cerr << "next can not be -1" << endl;
                exit(1);
            }
            fitness += manhatten(locs[curr], locs[next]);
            curr = next;
            next = this->next(curr);
        }

        fitness += manhatten(locs[curr], depot);

        //cout << "V: " << fitness << endl;
        if (fitness > res) {
            res = fitness;
        }
    }

    int drones = this->vrp->getDrones().size();

    if (drones != 0) {
        vector<float> dists;
        for (int fl : this->free_locations) {
            dists.push_back(2*this->vrp->distance(fl,0));
        }
        sort(dists.begin(), dists.end());

        

        float ds[drones] = {};

        while (dists.size() > 0) {
            int sm = 0;
            for (int i=0; i<drones; i++) {
                if (ds[i] < ds[sm]) {
                    sm = i;
                }
            }
            ds[sm] += dists.back()/Drone::speedFactor;
            dists.pop_back();
        }

        for (int i=0; i<drones; i++) {
            //cout << "D: " << ds[i] << endl;
            if (ds[i] > res) {
                res = ds[i];
            }
        }
    }

    if (DVRPD::droneEligible < 100) {
        unordered_set<int>& uneligible = this->vrp->getDroneUneligible();

        for (int fl : this->free_locations) {
            if (uneligible.find(fl) != uneligible.end()) {
                res += 1000;
            }    
        }
    }
    

    return res; 
}

float Routes::getFitnessDistMinRatio() {
    float longest_route = 0.0;
    float fitness = 0.0;

    vector<pair<int, float>> ts;

    Location* depot = vrp->depot;
    vector<Location*>& locs = vrp->getLocations();

    for (Route r : this->routes) {
        float r_length = 0.0;

        int curr = r.start;

        if(curr == 0) continue;

        r_length += manhatten(locs[curr], depot);
        

        int next = this->next(curr);

        while (next != 0) {
            if (next == -1) {
                cerr << "next can not be -1" << endl;
                exit(1);
            }
            r_length += manhatten(locs[curr], locs[next]);
            curr = next;
            next = this->next(curr);
        }

        r_length += manhatten(locs[curr], depot);

        ts.push_back({r.length, r_length});
        

        if (r_length > longest_route) {
            longest_route = r_length;
        }
    }

    int drones = this->vrp->getDrones().size();

    if (drones != 0) {

        vector<float> dists;
        for (int fl : this->free_locations) {
            dists.push_back(2*this->vrp->distance(fl,0));
        }
        sort(dists.begin(), dists.end());

        float ds[drones] = {};
        int da[drones] = {};

        while (dists.size() > 0) {
            int sm = 0;
            for (int i=0; i<drones; i++) {
                if (ds[i] < ds[sm]) {
                    sm = i;
                }
            }
            ds[sm] += dists.back()/Drone::speedFactor;
            da[sm]++;
            dists.pop_back();
        }

        for (int i=0; i<drones; i++) {
            ts.push_back({da[i], ds[i]});
            if (ds[i] > longest_route) {
                longest_route = ds[i];
            }
        }
    } else {
        fitness -= 100 * free_locations.size();
    }

    for (pair<int, float> p : ts) {
        fitness += p.first / (p.second/1 +  longest_route);
    }

    float res = 0.0;

    if (longest_route != 0.0) {
        res = -fitness*1000000/longest_route;
    }

    if (DVRPD::droneEligible < 100) {
        unordered_set<int>& uneligible = this->vrp->getDroneUneligible();

        for (int fl : this->free_locations) {
            if (uneligible.find(fl) != uneligible.end()) {
                res += 1000;
            }    
        }
    }
    
    return  res;
}

float Routes::getFitnessDistMin2() {
    float res = 0;

    for (Route r : this->routes) {
        float fitness = 0.0;

        int curr = r.start;

        if(curr == 0) continue;

        fitness += this->vrp->distance(curr,0);
        

        int next = this->next(curr);

        while (next != 0) {
            if (next == -1) {
                cerr << "next can not be -1" << endl;
                exit(1);
            }
            fitness += this->vrp->distance(curr,next);
            curr = next;
            next = this->next(curr);
        }

        fitness += this->vrp->distance(curr,0);

        //cout << "V: " << fitness << endl;
        if (fitness > res) {
            res = fitness;
        }
    }

    int drones = this->vrp->getDrones().size();

    if (drones != 0) {
        vector<float> dists;
        for (int fl : this->free_locations) {
            dists.push_back(2*this->vrp->distance(fl,0));
        }
        sort(dists.begin(), dists.end());

        float ds[drones] = {};

        while (dists.size() > 0) {
            int sm = 0;
            for (int i=0; i<drones; i++) {
                if (ds[i] < ds[sm]) {
                    sm = i;
                }
            }
            ds[sm] += dists.back()/Drone::speedFactor;
            dists.pop_back();
        }

        for (int i=0; i<drones; i++) {
            //cout << "D: " << ds[i] << endl;
            if (ds[i] > res) {
                res = ds[i];
            }
        }
    }

    if (DVRPD::droneEligible < 100) {
        unordered_set<int>& uneligible = this->vrp->getDroneUneligibleCapacity();

        for (int fl : this->free_locations) {
            if (uneligible.find(fl) != uneligible.end()) {
                res += 1000;
            }    
        }
    }
    

    return res; 
}

float Routes::getFitnessDistMinRatio2() {
    float longest_route = 0.0;
    float fitness = 0.0;

    vector<pair<int, float>> ts;

    for (Route r : this->routes) {
        float r_length = 0.0;

        int curr = r.start;

        if(curr == 0) continue;

        r_length += this->vrp->distance(curr,0);
        

        int next = this->next(curr);

        while (next != 0) {
            if (next == -1) {
                cerr << "next can not be -1" << endl;
                exit(1);
            }
            r_length += this->vrp->distance(curr,next);
            curr = next;
            next = this->next(curr);
        }

        r_length += this->vrp->distance(curr,0);

        ts.push_back({r.length, r_length});
        

        if (r_length > longest_route) {
            longest_route = r_length;
        }
    }

    int drones = this->vrp->getDrones().size();

    if (drones == 0) {

        fitness -= 100 * free_locations.size();

    } else {

        vector<float> dists;
        for (int fl : this->free_locations) {
            dists.push_back(2*this->vrp->distance(fl,0));
        }
        sort(dists.begin(), dists.end());


        float ds[drones] = {};
        int da[drones] = {};

        while (dists.size() > 0) {
            int sm = 0;
            for (int i=0; i<drones; i++) {
                if (ds[i] < ds[sm]) {
                    sm = i;
                }
            }
            ds[sm] += dists.back()/Drone::speedFactor;
            da[sm]++;
            dists.pop_back();
        }

        for (int i=0; i<drones; i++) {
            ts.push_back({da[i], ds[i]});
            if (ds[i] > longest_route) {
                longest_route = ds[i];
            }
        }

    } 

    

    int count = 0;
    for (pair<int, float> p : ts) {
        /*if (count < routes.size()) {
            fitness += p.first / (p.second + 2 * longest_route);
        } else {
            fitness += p.first / (p.second + 2 * longest_route);
        }*/
        fitness += p.first / (p.second + 500 * longest_route);
        

        count++;
    }

    float res = 0.0;

    if (longest_route != 0.0) {
        res = -fitness*1000000/longest_route;
    }

    if (DVRPD::droneEligible < 100) {
        unordered_set<int>& uneligible = this->vrp->getDroneUneligibleCapacity();

        for (int fl : this->free_locations) {
            if (uneligible.find(fl) != uneligible.end()) {
                res += 1000;
            }    
        }
    }
    
    return  res;
}

float Routes::getRouteLength(Route* route) {
    if (route->fitness != -1.0f) {
        return route->fitness;
    }
    float fitness = 0.0;

    int curr = route->start;

    if(curr == 0) return fitness;

    
    fitness += this->vrp->distance(0, curr);

    int next = this->next(curr);

    while (next != 0) {
        if (next == -1) {
            cerr << "next can not be -1" << endl;
            exit(1);
        }
        fitness += vrp->distance(curr, next);
        curr = next;
        next = this->next(curr);
    }

    fitness += vrp->distance(0, curr);

    route->fitness = fitness;
    
    return fitness;
}

int* Routes::routeToArray(Route& route) {
    int* res = new int[route.length];
    RouteNode* curr = &this->nodes[route.start];
    for (unsigned int i=0; i<route.length; i++) {
        res[i] = curr->location;
        curr = &this->nodes[curr->next];
    }
    return res;
}

int Routes::getRouteLevenshteinDist(Route& route1, Route& route2, Routes* mutation) {
    // d is een tabel met lenStr1+1 rijen and lenStr2+1 kolommen
    int s1= route1.length + 1;
    int s2 = route2.length + 1;

    int* r1 = this->routeToArray(route1);
    int* r2 = mutation->routeToArray(route2);

    int* d = new int[s1*s2];
 
    for (int i=0; i<s1; i++) {
        d[i*s2] = i;
    }
        
    for (int j=0; j<s2; j++) {
        d[j] = j;
    }

    int cost = 0;
 
    for (int i = 1; i < s1; i++) {
        for (int j = 1; j < s2; j++) {
            if (r1[i-1] == r2[j-1]){
                cost = 0;
            } else {
                cost = 1;
            }
            d[i*s2 + j] = min(min(d[(i-1)*s2 + j] + 1, d[i*s2 + j-1] + 1),  d[(i-1)*s2 + j-1] + cost);
        }
            
    }

    int res = d[s1* s2 - 1];

    delete [] r1;
    delete [] r2;
    delete [] d;
       
 
    return res;
}

float Routes::getBiasedFitnessRatio() {

    float fitness = getFitnessRatio();

    if (this == this->best1)
        return fitness;

    float bf = this->best1->getFitnessRatio();

    if (fitness < bf)
        return fitness;
    
    //if (fitness > 60) cout << max_time << endl;
    float diversity = this->get_diversity(this->best1);

    //float diversity = (diversity1 + diversity2 + diversity3) / 3;

    // TODO tweak & dont make hardcoded
    return 0.9 * fitness + 0.1 * fitness * diversity;
}

float Routes::getFitnessRatio2() {
    int drones = vrp->getDrones().size();

    float longest_route = 0.0f;

    vector<pair<int, float>> route_times;
    
    float fitness = 0.0;
    for(auto route = this->routes.begin(); route < this->routes.end(); route++) {
        int route_len = route->length;

        float route_time = this->getRouteLength(&(*route)) / (100 * Vehicle::speed) + Vehicle::loadTime + route_len * Vehicle::dropTime;

        if (route->length > Vehicle::maxCapacity) {
            route_len -= route->length - Vehicle::maxCapacity;
        }
        
        if (route_time > longest_route) {
            longest_route = route_time;
        }
        route_times.push_back({route_len, route_time});
    }

    if (drones == 0) {
        fitness -= 10 * this->free_locations.size();
    } else {
        float d_time = 0.0;
        for (int loc : this->free_locations) {
            d_time += 2 * this->vrp->distance(0, loc) / (100*Drone::speed) + Drone::loadTime + Drone::dropTime;
        }

        d_time /= drones;
        //d_time = (4/3 - 1/(3*drones)) * d_time;

        if (d_time > longest_route) {
            longest_route = d_time;
        }

        if (d_time != 0.0) {
            fitness += this->free_locations.size() / (drones*longest_route);
        }

        
    }

    for (pair<int, float> p : route_times) {
        if (p.second == 0.0f) 
            continue;

        fitness += p.first / (p.second + longest_route);
    }
    
    
    //if (fitness > 60) cout << max_time << endl;

    return -fitness;
}

float Routes::getFitnessRatio4() {
    int drones = vrp->getDrones().size();

    float longest_route = 0.0f;

    vector<pair<int, float>> route_times;

    float avg_time = 0.0f;
    
    float fitness = 0.0;
    for(auto route = this->routes.begin(); route < this->routes.end(); route++) {
        int route_len = route->length;

        float route_time = this->getRouteLength(&(*route)) / (100 * Vehicle::speed) + Vehicle::loadTime + route_len * Vehicle::dropTime;

        if (route->length > Vehicle::maxCapacity) {
            route_len = 0;
        }
        
        if (route_time > longest_route) {
            longest_route = route_time;
        }
        route_times.push_back({route_len, route_time});
    }

    if (drones == 0) {
        fitness -= 10 * this->free_locations.size();
    } else {
        float d_time = 0.0;
        for (int loc : this->free_locations) {
            d_time += 2 * this->vrp->distance(0, loc) / (100*Drone::speed) + Drone::loadTime + Drone::dropTime;
        }

        d_time /= drones;
        //d_time = (4/3 - 1/(3*drones)) * d_time;

        if (d_time > longest_route) {
            longest_route = d_time;
        }

        if (d_time != 0.0) {
            avg_time += d_time;
            fitness += this->free_locations.size() / (drones*longest_route);
        }

        
    }

    for (pair<int, float> p : route_times) {
        avg_time += p.second;
        if (p.second == 0.0f) 
            continue;
        
        fitness += p.first / (p.second + longest_route);
    }

    avg_time /=  route_times.size() + 1;

    return  -fitness/longest_route;
}

float Routes::getFitnessMakespan() {
    int drones = vrp->getDrones().size();

    float longest_route = 0.0f;
    
    for(auto route = this->routes.begin(); route < this->routes.end(); route++) {
        int route_len = route->length;

        float route_time = this->getRouteLength(&(*route)) / (100 * Vehicle::speed) + Vehicle::loadTime + route_len * Vehicle::dropTime;

        if (route->length > Vehicle::maxCapacity) {
            return -1.0f;
        }
        
        if (route_time > longest_route) {
            longest_route = route_time;
        }
    }

    if (drones == 0 && this->free_locations.size() > 0) {
        return -1.0f;
    } else {
        float d_time = 0.0;
        for (int loc : this->free_locations) {
            d_time += 2 * this->vrp->distance(0, loc) / (100*Drone::speed) + Drone::loadTime + Drone::dropTime;
        }

        d_time /= drones;
        //d_time = (4/3 - 1/(3*drones)) * d_time;

        if (d_time > longest_route) {
            longest_route = d_time;
        }   
    }

    return longest_route;
}

float Routes::getFitnessRatio3() {
    int drones = vrp->getDrones().size();

    float longest_route = 0.0f;

    vector<pair<int, float>> route_times;
    
    float fitness = 0.0;
    for(auto route = this->routes.begin(); route < this->routes.end(); route++) {
        int route_len = route->length;

        float route_time = this->getRouteLength(&(*route)) / (100 * Vehicle::speed) + Vehicle::loadTime + route_len * Vehicle::dropTime;

        if (route->length > Vehicle::maxCapacity) {
            route_len -= route->length - Vehicle::maxCapacity;
        }
        
        if (route_time > longest_route) {
            longest_route = route_time;
        }
        route_times.push_back({route_len, route_time});
    }

    if (drones == 0) {
        fitness -= 10 * this->free_locations.size();
    } else {
        float d_time = 0.0;
        for (int loc : this->free_locations) {
            d_time += 2 * this->vrp->distance(0, loc) / (100*Drone::speed) + Drone::loadTime + Drone::dropTime;
        }

        d_time /= drones;

        if (d_time > longest_route) {
            longest_route = d_time;
        }

        if (d_time != 0.0) {
            fitness += this->free_locations.size() / (drones*longest_route);
        }

        
    }

    for (pair<int, float> p : route_times) {
        if (p.second == 0.0f) 
            continue;

        fitness += p.first / (longest_route);
    }
    
    
    //if (fitness > 60) cout << max_time << endl;

    return -fitness;
}

float Routes::getFitnessRatio() {
    int drones = vrp->getDrones().size();

    const float min_time = Vehicle::maxT;
    
    float fitness = 0.0;
    for(auto route = this->routes.begin(); route < this->routes.end(); route++) {
        float route_time = this->getRouteLength(&(*route)) / (100 * Vehicle::speed);
        int route_len = route->length;

        if (route->length > Vehicle::maxCapacity) {
            route_len -= route->length - Vehicle::maxCapacity;
        }
        
        if (route_time > Vehicle::maxT) {
            route_time += route_time - Vehicle::maxT;
        }

        if (route_time != 0) {
            fitness += route_len / (route_time + min_time);
        }
    }
    
    
    //TODO use something like bin packing here
    
    float d_time = 0.0;
    unsigned int j = 0;

    while (j < this->free_locations.size()) {
        d_time += 2 * this->vrp->distance(0, this->free_locations[j]) / (100*Drone::speed);

        if (d_time > drones * min_time) {
            break;
        }
        
        j++;
    }
    
    if (d_time != 0) {
        fitness += drones * j/(d_time + drones * min_time);
    }
    
    //if (fitness > 60) cout << max_time << endl;

    return -fitness;
}

float Routes::getFitnessDistance() {
    //int drones = this->vrp->getDrones().size();
    
    float fitness = 0.0;
    for(Route route : this->routes) {
        float route_len = this->getRouteLength(&route);
        fitness += route_len;
        if (route.length == 0) {
            fitness += 1000;
        }
    }

    for (int loc : this->free_locations) {
        fitness += 2 * this->vrp->distance(loc, 0);
    }

    return fitness;
    /*
    DVRPD* vrp = this->vrp;

    sort(
        this->free_locations.begin(), 
        this->free_locations.end(), 
        [vrp](int a, int b) { return vrp->distance(0,a) < vrp->distance(0,b); }
    );
    
    //TODO use something like bin packing here
    for (int i=0; i<drones; i++) {
        float d_time = 0.0;
        unsigned int j = 0;
        while(j < this->free_locations.size()) {
            d_time += 2 * this->vrp->distance(0, this->free_locations[j]) / (100*Drone::speed);
            if (d_time > Vehicle::maxT) {
                fitness += d_time;
                
            }
            j++;
        }
    }

    return fitness;
    */
}

// sum travel time of all routes combined of drones and vehicles
float Routes::getBiasedFitness(){    
    float fitness = 0.0;
    for(Route route : this->routes) {
        float route_time = this->getRouteLength(&route) / (100 * Vehicle::speed);
        fitness += route_time;
        //cout << "rt: " << route_time << endl;
        if (route_time > Vehicle::maxT) {
            fitness += (route_time - Vehicle::maxT)*5;
        }
    }

    

    for(int loc : this->free_locations) {
        fitness += 2 * this->vrp->distance(0, loc) / (100 * Drone::speed);
    }

    float diversity = this->get_diversity(this->best1);

    if (diversity == 0.0f)
        return fitness;
    
    // TODO tweak & dont make hardcoded
    return fitness + (sqrt(10/diversity)*5);
}

// sum travel time of all routes combined of drones and vehicles
int Routes::getFitnessAmount(){
    int drones = this->vrp->getDrones().size();
    int fitness = 0;
    for(Route route : this->routes) {
        fitness += route.length;
        float route_duration = this->getRouteLength(&route) / (100*Vehicle::speed);
        
        float parcelTime = route.length / route_duration;
        if (route_duration > Vehicle::maxT) {
            fitness -= (route_duration - Vehicle::maxT) * parcelTime * 2;
        }
    }

    int fs_size = this->free_locations.size();
    float* free_time = new float[fs_size];

    for (int i=0; i<fs_size; i++) {
        free_time[i] = this->vrp->distance(0,this->free_locations[i]) / (100*Drone::speed);
    }

    sort(free_time, free_time+fs_size);
    
    //TODO use something like bin packing here
    int j=0;
    float d_time = 0.0;
    while(j < fs_size && d_time + free_time[j] < drones * Drone::maxT) {
        d_time += free_time[j];
        j++;
        fitness++;
    }

    //cout << "fitness2: " << fitness << endl;
    
    return -fitness;
}

// get random number between start and end, not including end
int random_num(int start, int end)
{
    int range = end-start;
    if (range <= 0) return start;
    int random_int = start+(rand()%range);
    return random_int;
}

int Routes::getCOprev(int n, unordered_set<int>& locations, int end_sec, int end_sec_d) {
    RouteNode curr = this->nodes[n];

    int prev = curr.prev;
    while(prev != 0) {
        if (locations.find(prev) == locations.end() && prev != end_sec) {
            break;
        }
        if (prev == end_sec) {
            return end_sec_d;
        }
        curr = this->nodes[curr.prev];
        prev = curr.prev;
    }
    return prev;
}

int Routes::getCOnext(int n, unordered_set<int>& locations, int start_sec) {
    RouteNode curr = this->nodes[n];

    int next=curr.next;
    while(next != 0) {
        if (locations.find(next) == locations.end() || next == start_sec) {
            break;
        }
        curr = this->nodes[curr.next];
        next=curr.next;
    }

    return next;
}

// cross over curent routes with given routes and store in mutation
// TODO bekijken impact grotere cross-over
Routes* Routes::cross_over(Routes* donor, Routes* acceptor){

    //cout << "cross_over" << endl;

    int rs_size = donor->routes.size();
    int rand_route_index = random_num(0,rs_size);
    Route route = donor->routes[rand_route_index];
    int r_size = route.length;

    if (r_size <= 1) 
        return NULL;

    int rand_index = random_num(0,r_size-1);
    int curr = donor->getPos(rand_index, route);

    // get random node from donor route that is also in a route in the acceptor
    int i = 0;
    while (this->nodes[curr].next == -1 && i < r_size) {
        i++;
        rand_index = (rand_index+1) % (r_size-1);
        curr = donor->getPos(rand_index, route);
    }

    if (this->nodes[curr].next == -1)
        return NULL;

    int rand_len = random_num(2, r_size-rand_index+1);

    this->op_cross_over(donor, acceptor, rand_route_index, rand_index, rand_len);

    /*if (!acceptor->check_locs("cross-over")) {
        cout << "BUG";
    }*/

    return acceptor;
}

void Routes::op_cross_over(Routes* donor, Routes* acceptor, unsigned int r_index, unsigned int index, unsigned int len) {
    if (r_index >= this->routes.size()) {
        cerr << "CO r_index out of bounds";
        exit(-1);
    }

    Route* route = &donor->routes[r_index];

    if (index + len > route->length) {
        cerr << "index out of bounds";
        exit(-1);
    }

    acceptor->free_locations = this->free_locations;
    acceptor->routes = this->routes;

    int curr = donor->getPos(index, *route);

    if (this->nodes[curr].next == -1) {
        cerr << "node not in acceptor route";
        exit(-1);
    }


    this->fill_location_route_map();

    unordered_set<int> locations;

    // heuristiek: we doen cross over op plaats van originele start node

    RouteNode node_d = donor->nodes[curr];
    int start_sec = curr;

    // get locations & insert piece
    for (unsigned int i=0; i<len; i++) {
        locations.insert(curr);
        node_d = donor->nodes[curr];
        acceptor->nodes[curr] = node_d;
        curr = node_d.next;
    }

    int end_sec_d = node_d.location;

    //remove inserted nodes from free locations

    for (unsigned int i=0; i<acceptor->free_locations.size(); i++) {
        if (locations.find(acceptor->free_locations[i]) != locations.end()) {
            acceptor->free_locations.erase(acceptor->free_locations.begin()+i);
            i--;
        }
    }

    // push overwritten locations to free_locations if needed
    RouteNode* node_t = &this->nodes[start_sec];
    for(unsigned int i=1; i< len; i++) {
        if (node_t->next == 0) {
            break;
        }
        node_t = &this->nodes[node_t->next];
        if (locations.find(node_t->location) == locations.end()) {
            acceptor->nodes[node_t->location] = {node_t->location, -1, -1};
            acceptor->free_locations.push_back(node_t->location);
            locations.insert(node_t->location);
        }
    }

    int end_sec = node_t->location;
    
    // get new prev and next for nodes, skipping nodes in locations
    for (unsigned int i=1; i<this->nodes.size(); i++) {
        if (locations.find(i) == locations.end()) {
            int next = getCOnext(i, locations, start_sec);
            int prev = getCOprev(i, locations, end_sec, end_sec_d);
            if (prev == 0) 
                acceptor->routes[this->location_route_map[i]].start = i;
            acceptor->nodes[i]={i, next, prev};
        } else if ((int)i == start_sec) {
            int prev = getCOprev(i, locations, end_sec, end_sec_d);
            acceptor->nodes[i].prev=prev;
            if (prev == 0)
                acceptor->routes[this->location_route_map[i]].start = i;
        } else if ((int)i == end_sec_d) {
            int next = getCOnext(end_sec, locations, start_sec);
            acceptor->nodes[i].next=next;
        }
    }

    // fix empty routes
    for (unsigned int i=0 ; i<acceptor->routes.size(); i++) {
        int st = acceptor->routes[i].start;
        if (acceptor->nodes[st].prev != 0) {
            acceptor->routes[i].start = 0;
        }
    }

    // recalc lengths and fitness
    for (unsigned int i=0; i<acceptor->routes.size(); i++) {
        int count = 0;
        Route* r = &acceptor->routes[i];
        curr = r->start;
        while (curr != 0) {
            curr = acceptor->next(curr);
            count++;
        }
        r->length = count;
        r->fitness = -1.0f;
    }
}

/*
bool Routes::check_inf_loop() {
    for (Route r : this->routes) {
        int curr = r.start;
        for (size_t i = 0; i < Vehicle::maxCapacity; i++)
        {
            curr = this->next(curr);
        }
        if (curr != 0) return false;
    }
    return true;
}*/

bool Routes::check_locs(string name) {
    for (Route r : this->routes) {
        int curr = r.start;
        int prev = 0;
        for (size_t i = 0; i < Vehicle::maxCapacity*2; i++)
        {
            if (curr == 0) {
                if (i < r.length) {
                    cout << name << ": Route to short!" << endl;
                    return false;
                }
                if (i > r.length) {
                    cout << name << ": Route to long!" << endl;
                    return false;
                }
                break;
            }
            if (find(this->free_locations.begin(), this->free_locations.end(), curr) != this->free_locations.end()) {
                cout << name << ": location in free locations!" << endl;
                return false;
            }
            if (this->nodes[curr].prev != prev) {
                cout << name << ": wrong prev!" << endl;
                return false;
            }
            prev = curr;
            curr = this->next(curr);
        }
    }
    for (int i : this->free_locations) {
        RouteNode* n = &this->nodes[i];
        if (n->next != -1 || n->prev != -1 || (int)n->location != i) {
            cout << name << ": free location not empty!" << endl;
            return false;
        }
    }
    return true;
}

// remove locations out of the routes except for the sequence starting from exclude with length exclude_len
void Routes::remove_locations(unordered_set<int>& locations) {
    // remove locations from free_locations
    for (int location : locations) {
        vector<int>::iterator position = find(this->free_locations.begin(), this->free_locations.end(), location);
        if (position != this->free_locations.end()) {
            this->free_locations.erase(position);
        } else {
            RouteNode node = this->nodes[location];
            this->nodes[node.prev].next = node.next;
            this->nodes[node.next].prev = node.prev;
            node.next = -1;
            node.prev = -1;
        }
    }
}

void Routes::LPT(vector<int>* result) {
    vector<pair<float, int>> dists;
    for (int fl : this->free_locations) {
        dists.push_back({2*this->vrp->distance(fl,0), fl});
    }
    sort(dists.begin(), dists.end());

    int drones = this->vrp->getDrones().size();

    float ds[drones] = {};

    while (dists.size() > 0) {
        int sm = 0;
        for (int i=0; i<drones; i++) {
            if (ds[i] < ds[sm]) {
                sm = i;
            }
        }
        ds[sm] += dists.back().first/Drone::speedFactor;
        result[sm].push_back(dists.back().second);
        dists.pop_back();
    }

}

void Routes::print(ostream& file) {
    int route_index = 1;

    for (Route route : this->routes) {
        int curr = route.start;
        
        file << "Truck Route #" << route_index << ":";
        route_index++;

        while (curr != 0) {
            file << " " << curr;
            curr = this->next(curr);
        }
        file << endl;
    }

    int drones = this->vrp->getDrones().size();

    vector<int> drone_routes[drones] = {};
    this->LPT(drone_routes);

    for (int i=0; i<drones; i++) {
        file << "Drone Route #" << route_index << ":";
        route_index++;
        for(int location : drone_routes[i]) {
            file << " " << location;
        }
        file << endl;
    }
    file << "Makespan " << this->getFitnessDistMin2();
}

// clone this routes to given pointer
void Routes::clone_to(Routes* mutation) {
    // copy free_locations
    mutation->free_locations = this->free_locations;
    mutation->nodes = this->nodes;
    mutation->routes = this->routes;
}

bool SF_random(Routes* mutation, int* r_index, int* loc, int* fl_index) {
    int fl_size = mutation->free_locations.size();
    int r_size = mutation->routes.size();
    int rand_route_index = random_num(0,r_size);

    Route* rand_route = &mutation->routes[rand_route_index];

    int route_size = rand_route->length;

    if (route_size == 0 || mutation->free_locations.size() == 0) {
        return false;
    }

    *loc = mutation->getPos(random_num(0,route_size), *rand_route);
    *fl_index = random_num(0,fl_size);
    *r_index = rand_route_index;

    return true;
}

bool SF_heuristic1(Routes* mutation, int* r_index, int* loc, int* fl_index) {
    int fl_size = mutation->free_locations.size();
    int random_fl_index = random_num(0,fl_size);

    unordered_set<int> neighbours = mutation->vrp->getNeighbours(random_fl_index, 0);

    auto it = neighbours.begin();
    int rand_it = random_num(0,neighbours.size());

    advance(it, rand_it);

    unsigned int i=0;
    while (i<neighbours.size()) {
        if (it == neighbours.end())
            it = neighbours.begin();
        
        if (mutation->nodes[*it].next != -1) 
            break;
        it++;
        i++;
    }

    if (i == neighbours.size() || mutation->nodes[*it].prev == -1) {
        return false;
    }

    *loc = *it;
    *fl_index = random_fl_index;

    int route;
    int index;
    mutation->getRouteIndex(*it, &route, &index);

    *r_index = route;

    return true;
}

// swap random route node with free location
Routes* mutate_swap_free(Routes* mutation) {
    /*
        HEURISTIEK:
            - Swap met dichte free values probeer random 3-10 zoals bij add
            - omgekeerd zoals bij heuristiek 2 add, start met free_loc en zoek swap
    */

    //cout << "mutate_swap_free" << endl;

    int r_index;
    int loc;
    int fl_index;

    if (mutation->free_locations.size() == 0)
        return NULL;

    if (!SF_heuristic1(mutation, &r_index, &loc, &fl_index))
        return NULL;

    if (r_index == -1 || loc == -1) {
        return NULL;
    }

    mutation->op_swap_free(r_index, loc, fl_index);

    /*if (!mutation->check_locs("mutate_swap_free")) {
        cout << "BUG";
    }*/
    return mutation;
}

void Routes::op_swap_free(unsigned int r_index, unsigned int loc, unsigned int fl_index) {
    if (r_index >= this->routes.size()) {
        cerr << "SF r_index out of bounds";
        exit(-1);
    }

    Route* route = &this->routes[r_index];

    RouteNode* node = &this->nodes[loc];

    if (fl_index >= this->free_locations.size()) {
        cerr << "fl_index out of bounds " << fl_index << ", " << this->free_locations.size();
        exit(-1);
    }

    int fl = this->free_locations[fl_index];
    RouteNode* acc = &this->nodes[fl];

    if (node->prev == 0) {
        this->routes[r_index].start = fl;
        acc->prev = 0;
        acc->next = node->next;
        if (acc->next != 0)
            this->nodes[acc->next].prev = acc->location;
    } else {
        acc->next = node->next;
        acc->prev = node->prev;
        if (acc->next != 0)
            this->nodes[acc->next].prev = acc->location;
        if (acc->prev != 0)
            this->nodes[acc->prev].next = acc->location;
    }

    node->next = -1;
    node->prev = -1;

    this->free_locations[fl_index] = loc;
    //TODO kan beter door verschil in fitness te berekenen hier
    route->fitness = -1.0f;
}

bool SP_random(Routes* mutation, int* route, int* index, int* len, int* insert_pos) {
    int r_size = mutation->routes.size();

    int rand_route_index = random_num(0,r_size);

    Route* rand_route = &mutation->routes[rand_route_index];

    int route_size = rand_route->length;

    if (route_size <= 1)
        return false;

    *index = random_num(0,route_size);
    *len = random_num(1,min(route_size-*index+1, route_size-1));
    *insert_pos = random_num(0,route_size-*len);
    *route = rand_route_index;

    return true;
}

bool SP_heuristic1(Routes* mutation, int* route, int* index, int* len, int* insert_pos) {
    int r_size = mutation->routes.size();

    int rand_route_index = random_num(0,r_size);

    Route* rand_route = &mutation->routes[rand_route_index];

    int route_size = rand_route->length;

    if (route_size <= 1)
        return false;

    *index = random_num(0,route_size);
    *len = random_num(1,min(route_size-*index+1, route_size-1));

    RouteNode* start = &mutation->nodes[mutation->getPos(*index, *rand_route)];
    // get end
    int e = start->location;
    for (int i=0; i<*len-1; i++) {
        e = mutation->next(e);
    }

    RouteNode* end = &mutation->nodes[e];

    RouteNode* curr = &mutation->nodes[rand_route->start];
    int best = *index;
    float best_d = mutation->vrp->distance(start->prev, start->location) + mutation->vrp->distance(end->next, end->location);
    int ip = 0;
    while (curr->next != 0) {
        if (curr->location == start->location) {
            ip++;
            RouteNode* en = &mutation->nodes[end->next];
            if (en->location == 0)
                break;
            curr = &mutation->nodes[en->next];
        }
        
        if (curr->location == 0) {
            break;
        }
        
        float dist = mutation->vrp->distance(start->location, curr->prev) + mutation->vrp->distance(end->location, curr->location);

        if (dist < best_d) {
            best = ip;
            best_d = dist;
        }
        ip++;
        curr = &mutation->nodes[curr->next];
    }

    //TODO laatste plaats zit hier nog niet in!

    *insert_pos = best;
    *route = rand_route_index;

    return true;
}

// move sequence of nodes in routes to different position
// e.g: A-B-C-D-E => D-E-A-B-C
Routes* mutate_swap_pos(Routes* mutation) {
    /*
        HEURISTIEK:
            - swap stukken waar start en end dicht bij elkaar
            - swap met pos waar start van seq en end van seq dichtst staat
                1. bepaal stuk voor swap
                2. loop over route, en bepaal distance met start en end. 
                3. pak degene die het dichtste ligt (maar wel ver van de initiële pos?)
    */

    //cout << "mutate_swap_pos" << endl;
    
    int route;
    int index;
    int len;
    int insert_pos;

    if (!SP_heuristic1(mutation, &route, &index, &len, &insert_pos))
        return NULL;

    mutation->op_swap_pos(route, index, len, insert_pos);

    /*if(!mutation->check_locs("mutate_swap_pos")){
        cout << "bug";
    }*/

    return mutation;
}

void Routes::op_swap_pos(unsigned int r_index, unsigned int index, unsigned int len, unsigned int insert_pos) {
    if (r_index >= this->routes.size()) {
        cerr << "SP r_index out of bounds";
        exit(-1);
    }

    Route* route = &this->routes[r_index];

    if (index+len > route->length) {
        cerr << "index out of bounds";
        exit(-1);
    }

    if (insert_pos > route->length - len) {
        cerr << "insert_pos out of bounds";
        exit(-1);
    }

    int gap_start = this->getPos(index, *route);
    int gap_end = this->getPos(index+len-1, *route);

    RouteNode* start_node = &this->nodes[gap_start];
    RouteNode* end_node = &this->nodes[gap_end];

    if (index == 0) {
        route->start = end_node->next;
        if (end_node->next != 0)
            this->nodes[end_node->next].prev = 0;
    } else {
        this->nodes[start_node->prev].next = end_node->next;
        if (end_node->next != 0)
            this->nodes[end_node->next].prev = start_node->prev;
    }

    this->insertAt(route, insert_pos, start_node, end_node);
    route->fitness = -1.0f;
}

bool MCO_random(Routes* mutation, int* route1, int* index1, int* len1, int* route2, int* index2, int* len2) {
    int r_size = mutation->routes.size();
    int rand_route_index1 = random_num(0,r_size);
    int rand_route_index2 = random_num(0,r_size-1);

    if (rand_route_index2 >= rand_route_index1)
        rand_route_index2++;

    Route* rand_route1 = &mutation->routes[rand_route_index1];
    Route* rand_route2 = &mutation->routes[rand_route_index2];

    int route_size1 = rand_route1->length;
    int route_size2 = rand_route2->length;

    // if one of the routes is empty we can not do cross over
    if (route_size1 == 0 && route_size2 == 0)
        return false;

    // get index and length of cross-over
    *index1 = random_num(0, route_size1);
    *index2 = random_num(0, route_size2);
    *len1 = random_num(0, route_size1 - *index1 + 1);
    *len2 = random_num(0, route_size2 - *index2 + 1);

    if (*len1 == 0 && *len2 == 0) 
        return false;

    *route1 = rand_route_index1;
    *route2 = rand_route_index2;

    return true;
}

bool MCO_heuristic1(Routes* mutation, int* route1, int* index1, int* len1, int* route2, int* index2, int* len2) {
    // get 2 locs close to start and end from different route and replace parts.

    mutation->fill_location_route_map();

    int r_size = mutation->routes.size();
    *route1 = random_num(0,r_size);

    Route* rand_route1 = &mutation->routes[*route1];

    if (rand_route1->length == 0)
        return false;

    *index1 = random_num(0, rand_route1->length);
    *len1 = random_num(0, rand_route1->length - *index1 + 1);
    

    RouteNode* start = &mutation->nodes[mutation->getPos(*index1, *rand_route1)];
    RouteNode* end = start;
    for (int i=0; i<*len1-1; i++) {
        end = &mutation->nodes[end->next];
    }

    unordered_set<int> start_n = mutation->vrp->getNeighbours(start->prev, 0);
    unordered_set<int> end_n = mutation->vrp->getNeighbours(end->next, 0);

    auto it_s = start_n.begin();
    auto it_e = end_n.begin();

    if (start_n.size() == 0 || end_n.size() == 0)
        return false;

    advance(it_s, random_num(0, start_n.size()));
    advance(it_e, random_num(0, end_n.size()));

    int it_ss = *it_s;
    int it_es = *it_e;

    int r1;
    int r2;

    //TODO loop from random start position.
    do {
        r1 = mutation->location_route_map[*it_s];
        do {
            r2 = mutation->location_route_map[*it_e];
            if (r1 == r2 && r1 != *route1 && r1 != -1)
                break;
            it_e++;
            if (it_e == end_n.end())
                it_e = end_n.begin();
        } while(*it_e != it_es);
        if (r1 == r2)
            break;
        it_s++;
        if (it_s == start_n.end())
            it_s = start_n.begin();
    } while (*it_s != it_ss);

    if (r1 != r2 || (int)r1 == *route1 || r1 >= (int)mutation->routes.size() || it_s == start_n.end() || it_e == end_n.end() || r1 < 0)
        return false;

    *route2 = r1;

    // search if it_s before it_e or after
    RouteNode* n1 = &mutation->nodes[*it_s];
    RouteNode* n2 = &mutation->nodes[*it_e];

    Route* route = &mutation->routes[*route2];

    RouteNode* curr = &mutation->nodes[route->start];

    bool reverse;
    int index = 0;
    int len = 0;

    while (curr->location != 0) {
        if (curr->location == n1->location || curr->location == n2->location) {
            if (len == 0) {
                reverse = curr->location != n1->location;
                // TODO implement met reverse erbij!
                if (reverse)
                    return false;
                *index2 = index;
                len = 1;
                if (n1->location == n2->location) {
                    *len2 = len;
                    return true;
                }
            } else {
                *len2 = len;
                return true;
            }
        }
        if (len == 0) {
            index++;
        } else {
            len++;
        }
        
        curr = &mutation->nodes[curr->next];
    }

    return false;
}

// take 2 routes and do a cross-over between them
Routes* mutate_cross_over(Routes* mutation) {
    /*
        HEURISTIEK:
            - zoek start en stop stukken die dicht bij elkaar liggen.
                1. neem random route, start en end
                2. getNeighbourhood(start) en getNeighbourhood(end)
                3. zoek in de neighbourhoods naar neighbours met eenzelfde route verschillend van de random route
                4. maak hiervan de cross over. (let op welke is start/stop in LocationInfo?)
    */

   //cout << "mutate_cross_over" << endl;

   int route1;
   int index1;
   int len1;
   int route2;
   int index2;
   int len2;

    if (!MCO_heuristic1(mutation, &route1, &index1, &len1, &route2, &index2, &len2))
        return NULL;

    mutation->op_cross_over_m(route1, index1, len1, route2, index2, len2);
    
    /*if (!mutation->check_locs("mutate_cross_over")) {
        cout << "BUG";
    }*/
    return mutation;
}

void Routes::op_cross_over_m(unsigned int r_index1, unsigned int index1, unsigned int len1, unsigned int r_index2, unsigned int index2, unsigned int len2) {
    if (r_index1 >= this->routes.size() || r_index1 >= this->routes.size()) {
        cerr << "COM r_index out of bounds";
        exit(-1);
    }

    Route* route1 = &this->routes[r_index1];
    Route* route2 = &this->routes[r_index2];

    if (index1 + len1 > route1->length) {
        cerr << "index1 out of bounds";
        exit(-1);
    }

    if (index2 + len2 > route2->length) {
        cerr << "index2 out of bounds";
        exit(-1);
    }

    unsigned int route_size1 = route1->length;
    unsigned int route_size2 = route2->length;

    RouteNode* s1;
    RouteNode* s2;

    RouteNode* e1;
    RouteNode* e2;

    if (len1 > 0) {

        s1 = &this->nodes[this->getPos(index1, *route1)];
        e1 = &this->nodes[this->getPos(index1+len1-1, *route1)];

        if (index1 == 0) {
            route1->start = e1->next;
            if (e1->next != 0)
                this->nodes[e1->next].prev = 0;
        } else if (index1+len1 == route_size1) {
            this->nodes[s1->prev].next = 0;
        } else {
            this->nodes[e1->next].prev = s1->prev;
            this->nodes[s1->prev].next = e1->next;
        }   
    }

    if (len2 > 0) {
        s2 = &this->nodes[this->getPos(index2, *route2)];
        e2 = &this->nodes[this->getPos(index2+len2-1, *route2)];

        if (index2 == 0) {
            route2->start = e2->next;
            if (e2->next == -1) {
                cout << "BUG";
            }
            if (e2->next != 0)
                this->nodes[e2->next].prev = 0;
        } else if (index2+len2 == route_size2) {
            this->nodes[s2->prev].next = 0;
        } else {
            this->nodes[e2->next].prev = s2->prev;
            this->nodes[s2->prev].next = e2->next;
        }
    }

    
    if (len1 > 0) 
        this->insertAt(route2, index2, s1, e1);

    if (len2 > 0) 
        this->insertAt(route1, index1, s2, e2);
    

    route1->length = route_size1 - len1 + len2;
    route2->length = route_size2 - len2 + len1;
    route1->fitness = -1.0f;
    route2->fitness = -1.0f;
}

bool MA_random(Routes* mutation, int* route, int* index, unordered_set<int>* locs) {
    int r_size = mutation->routes.size();
    int rand_route_index = random_num(0,r_size);

    Route* rand_route = &mutation->routes[rand_route_index];
    unsigned int route_size = rand_route->length;

    unsigned int add_amount = random_num(1, Vehicle::maxCapacity-route_size+1);
    add_amount = min((int)add_amount, (int)mutation->free_locations.size());
    
    //TODO kijk of zonder route_size >= Vehicle::maxCapacity beter is
    if (route_size >= Vehicle::maxCapacity || mutation->free_locations.size() == 0)
        return false;

    int rand_index = random_num(0,route_size+1);
    
    for (size_t i = 0; i < add_amount; i++)
    {
        int fl_size = mutation->free_locations.size();

            
        int rand_fl_index = random_num(0,fl_size-i);
        int fl = mutation->free_locations[rand_fl_index];

        locs->insert(fl);
    }

    *route = rand_route_index;
    *index = rand_index;

    return true;
}

bool MA_heuristic1(Routes* mutation, int* route, int* index, unordered_set<int>* locs) {
    /*
        Add add_amount locations to route by getting random freelocs 
        randomly chosen that are the closest to the random insert location

        ideas: add_amount moet meer 1 zijn
    */
    int r_size = mutation->routes.size();
    int rand_route_index = random_num(0,r_size);

    Route* rand_route = &mutation->routes[rand_route_index];
    unsigned int route_size = rand_route->length;

    unsigned int add_amount = random_num(1, Vehicle::maxCapacity-route_size+1);
    add_amount = min((int)add_amount, (int)mutation->free_locations.size());
    
    //TODO kijk of zonder route_size >= Vehicle::maxCapacity beter is
    if (route_size >= Vehicle::maxCapacity || mutation->free_locations.size() == 0)
        return false;

    int rand_index = random_num(0,route_size+1);

    RouteNode* comp;

    if (rand_index == 0) {
        comp = &mutation->nodes[rand_route->start];
    } else {
        comp = &mutation->nodes[mutation->getPos(rand_index-1, *rand_route)];
    }
    
    for (size_t i = 0; i < add_amount; i++)
    {
        const int max_add_tries = 5;

        int best_loc;
        int fl_size = mutation->free_locations.size();
        float best_dist = FLT_MAX;

        for (int j=0; j<max_add_tries; j++) {
            
            int rand_fl_index = random_num(0,fl_size-i);
            int fl = mutation->free_locations[rand_fl_index];

            //TODO kijken of hier best comp->location of comp_l
            if (fl == -1 || comp->location >= mutation->vrp->getLocations().size()) {
                cout << "BUG";
            }
            float dist = mutation->vrp->distance(fl, comp->location);

            if (dist < best_dist) {
                best_dist = dist;
                best_loc = fl;
            }
        }

        locs->insert(best_loc);
    }

    *route = rand_route_index;
    *index = rand_index;

    return true;
}

vector<int> nearest_neighbour(DVRPD* vrp, int start, unordered_set<int> locs) {
    vector<int> result;
    int prev = start;
    for(unsigned int i=0; i<locs.size(); i++) {
        int best;
        float best_d = FLT_MAX;
        for (int loc: locs) {
            float dist = vrp->distance(prev, loc);
            if (dist < best_d) {
                best_d = dist;
                best = loc;
            }
        }
        result.push_back(best);
        locs.erase(best);
        prev = best;
    }

    return result;
}

bool MA_heuristic2(Routes* mutation, int* route, int* index, unordered_set<int>* locs) {
    /*
        get random insert location
        get neighbours of random insert location

        randomly chosen that are the closest to the random insert location

        ideas: add_amount moet meer 1 zijn
    */
    int r_size = mutation->routes.size();
    int rand_route_index = random_num(0,r_size);

    Route* rand_route = &mutation->routes[rand_route_index];

    int rand_index = random_num(0,rand_route->length+1);

    RouteNode* comp;

    if (rand_index == 0) {
        comp = &mutation->nodes[rand_route->start];
    } else {
        comp = &mutation->nodes[mutation->getPos(rand_index-1, *rand_route)];
    }

    mutation->fill_location_route_map();

    //TODO test what value is best here (seems 2 or 3)
    unordered_set<int> neighbours = mutation->vrp->getNeighbours(comp->location, 3);

    vector<int> fls;
    
    // add all 
    for (int n: neighbours) {
        if (mutation->location_route_map[n] == -1 && n != 0)
            fls.push_back(n);
    }

    int max_add = min((int)fls.size(), (int)(Vehicle::maxCapacity - rand_route->length));

    if (max_add < 1)
        return false;

    int add_amount = random_num(1, max_add);

    shuffle(fls.begin(), fls.end(), default_random_engine(rand()));

    for (int i=0; i< add_amount; i++) {
        locs->insert(fls[i]);
    }
    //TODO instead of adding them randomly maybe try adding by heuristic like nearest neighbour ?

    *route = rand_route_index;
    *index = rand_index;

    return true;
}

bool MA_heuristic3(Routes* mutation, int* route, int* index, unordered_set<int>* locs) {
    /*
        from random free loc get neighbours and add to closest ?
    */

   if (mutation->free_locations.size() == 0) {
       return false;
   }
    
    int rand_fl_index = random_num(0,mutation->free_locations.size());

    int fl = mutation->free_locations[rand_fl_index];

    mutation->fill_location_route_map();

    //TODO test what value is best here (seems 2 or 3)
    unordered_set<int> neighbours = mutation->vrp->getNeighbours(fl, 1);

    vector<int> ls;
    
    // add all 
    for (int n: neighbours) {
        if (mutation->location_route_map[n] != -1 && n != 0)
            ls.push_back(n);
    }

    if (ls.size() == 0)
        return false;

    pair<float, int> best = {FLT_MAX, -1};

    for (int loc : ls) {
        int d = mutation->vrp->distance(loc, fl);
        if (best.first > d) {
            best = {d, loc};
        }
    }

    int r, i;
    mutation->getRouteIndex(best.second, &r, &i);
    
    locs->insert(fl);

    if (r < 0) {
        cout << "bug";
    }

    *route = r;
    *index = i;

    return true;
}

// Add random free location to random route
Routes* mutate_add(Routes* mutation) {
    /*
    HEURISTIEK: 
        - ipv random route meer kans aan routes met kortste lengte?
        - neem 3*add_amount en hou enkel de add_amount dichtste over
        - Bij grote lengte kleine adds bij kleine lengte grote adds ???

    HEURISTIEK 2:
        omgekeerd werken starten met free_location en dan route zoeken waar die geadd kan worden.
        Free location ver van depot meer kans genomen
    */

   //cout << "mutate_add" << endl;

    int route, index;
    unordered_set<int> locs;

    if(!MA_heuristic2(mutation, &route, &index, &locs)) {
        return NULL;
    }

    mutation->op_add(route, index, locs);
    
    /*if(!mutation->check_locs("mutate_add")) {
        cout << "bug";
    }*/
    return mutation;
}

void Routes::op_add(unsigned int r_index, unsigned int index, unordered_set<int>& locs) {
    if (r_index >= this->routes.size()) {
        cerr << "MA r_index out of bounds: " << r_index;
        exit(-1);
    }

    Route* route = &this->routes[r_index];

    if (index > route->length) {
        cerr << "index out of bounds";
        exit(-1);
    }

    RouteNode* start;
    RouteNode* end;

    int i=0;

    for (int loc : locs) {


        this->free_locations.erase(find(this->free_locations.begin(), this->free_locations.end(), loc));

        RouteNode* node = &this->nodes[loc];

        /* 
            Heuristiek adden volgens beste route tussen het stuk
            of e.g. 10 random configs en de beste adden
        */

        if (i==0) {
            node->prev = 0;
            node->next = 0;
            start = node;
            end = node;
        } else {
            end->next = node->location;
            node->prev = end->location;
            node->next = 0;
            end = node;
        }
        i++;
    }

    this->insertAt(route, index, start, end);

    route->length += i;
    route->fitness = -1.0f;
}

bool MR_random(Routes* mutation, int* route, int* index, int* amount) {
    int r_size = mutation->routes.size();
    *route = random_num(0,r_size);

    Route* rand_route = &mutation->routes[*route];
    int route_size = rand_route->length;

    if (route_size <= 0)
        return NULL;

    *index = random_num(0,route_size);

    *amount = random_num(1,route_size-*index);

    return true;
}

// choose random x indices and remove the one that has the most different nodes from different route amongst it
bool MR_heuristic1(Routes* mutation, int* route, int* index, int* amount) {

    mutation->fill_location_route_map();

    const int tries = 5;

    int best = 0;
    int best_score = 0;

    for (int i=0; i< tries; i++) {
        int curr;
        // get random node in route
        do {
            curr = random_num(1, mutation->nodes.size());
        } while (mutation->vrp->free_ids.find(curr) != mutation->vrp->free_ids.end() || mutation->location_route_map[curr] == -1);

        int curr_route = mutation->location_route_map[curr];

        unordered_set<int> neighbours = mutation->vrp->getNeighbours(curr, 0);

        int score = 0;

        for (int loc : neighbours) {
            if (mutation->location_route_map[loc] != curr_route && mutation->location_route_map[loc] != -1)
                score++;
        }

        if (score > best_score) {
            best = curr;
            best_score = score;
        }
    }

    if (best == 0)
        return false;

    int ro, in;
    mutation->getRouteIndex(best, &ro, &in);

    *route = ro;
    *index= in;

    *amount = random_num(1,mutation->routes[*route].length -*index);

    return true;
}

// check for a route best node to remove
bool MR_heuristic2(Routes* mutation, int* route, int* index, int* amount) {

    int r_size = mutation->routes.size();
    *route = random_num(0,r_size);

    Route* rand_route = &mutation->routes[*route];
    int route_size = rand_route->length;

    set<pair<float, int>> dists; 

    *amount = random_num(1,route_size/2);

    RouteNode* start = &mutation->nodes[rand_route->start];
    RouteNode* stop = start;

    int i=0;

    float dps = 0.0f;

    dps += mutation->vrp->distance(stop->prev, stop->location);
    for (int i=0; i<*amount-1; i++) {
        dps += mutation->vrp->distance(stop->next, stop->location);
        stop = &mutation->nodes[stop->next];
    }
    dps += mutation->vrp->distance(stop->next, stop->location);
    
    while (stop->next != 0) {
        float d = dps - mutation->vrp->distance(start->prev, stop->next);
        dists.insert({-d, i});
        i++;
        dps -= mutation->vrp->distance(start->prev, start->location);
        start = &mutation->nodes[start->next];
        stop = &mutation->nodes[stop->next];
        dps += mutation->vrp->distance(stop->location, stop->next);
    }

    default_random_engine generator;
    normal_distribution<float> distribution(0,5);
    float number = distribution(generator);

    int num = (int) floor(abs(number)) % dists.size();

    auto it = dists.begin();
    advance(it, num);

    *index = it->second;

    return true;
}

// remove random location from route and make it a free location
Routes* mutate_remove(Routes* mutation) {
    /*
        HEURISTIEK:
            - remove degene die de fitness naar beneden halen
            - meer kans remove als dicht bij depot
            - removen als er veel rond een andere route hebben
            - niet removen als veel errond vrij zijn
    */

   //cout << "mutate_remove" << endl;

    int route;
    int index;
    int amount;

    if (!MR_random(mutation, &route, &index, &amount)) {
        return NULL;
    }

    mutation->op_remove(route, index, amount);

    /*if(!mutation->check_locs("mutate_remove")) {
        cout << "bug";
    }*/
    return mutation;
}

void Routes::op_remove(unsigned int r_index, unsigned int index, unsigned int amount) {
    if (r_index >= this->routes.size()) {
        cerr << "MR r_index out of bounds";
        exit(-1);
    }

    Route* route = &this->routes[r_index];

    if (index+amount > route->length) {
        cerr << "index out of bounds";
        exit(-1);
    }

    RouteNode* start = &this->nodes[this->getPos(index, *route)]; // -1
    RouteNode* end = &this->nodes[this->getPos(index+amount-1, *route)];

    // close gap
    if (index == 0) {
        route->start = end->next;
        if (end->next != 0)
            this->nodes[end->next].prev = 0;
    } else if (index+amount == route->length) {
        this->nodes[start->prev].next = 0;
    } else {
        this->nodes[start->prev].next = end->next;
        this->nodes[end->next].prev = start->prev;
    }

    RouteNode* temp;

    // remove nodes
    for (unsigned int i=0; i<amount; i++) {
        this->free_locations.push_back(start->location);
        temp = start;
        start = &this->nodes[start->next];
        temp->next = -1;
        temp->prev = -1;
    }

    route->length -= amount;
    route->fitness = -1.0f;
}

bool RP_random(Routes* mutation, int* route, int* start, int* stop) {
    // get random route
    int r_size = mutation->routes.size();
    *route = random_num(0,r_size);

    Route* rand_route = &mutation->routes[*route];
    int route_size = rand_route->length;

    if (route_size <= 1)
        return false;

    // get start and end of reverse
    *start = random_num(0,route_size-1);
    *stop = random_num(*start+1,route_size);

    return true;
}

bool RP_2_opt(Routes* mutation, int* route, int* start, int* end) {
    // get random route
    int r_size = mutation->routes.size();
    *route = random_num(0,r_size);

    Route* rand_route = &mutation->routes[*route];
    int route_size = rand_route->length;

    if (route_size <= 1)
        return false;

    RouteNode* n_start = &mutation->nodes[rand_route->start];
    RouteNode* n_end = &mutation->nodes[n_start->next];
    *start = 0;
    *end = 0;

    while (n_start->location != 0) {
        while (n_end->location != 0) {
            float d = mutation->vrp->distance(n_start->prev, n_start->location) 
                + mutation->vrp->distance(n_end->location, n_end->next)
                - mutation->vrp->distance(n_start->prev, n_end->location)
                - mutation->vrp->distance(n_start->location, n_end->next);
            if (d > 0) {
                // TODO hier kunnen we eigl fitness al zetten!
                return true;
            }
            n_end = &mutation->nodes[n_end->next];
            *end += 1;
        }
        n_start = &mutation->nodes[n_start->next];
        *start += 1;
        *end = *start+1;
        n_end = &mutation->nodes[n_start->next];
    }

    return false;
}

// take random part of route and reverse it
Routes* mutate_reverse_part(Routes* mutation) {
    /*
        HEURISTIEK:
            - reverse part waar begin en einde dicht bij elkaar liggen
            - maak hier 2-op van!
    */

    //cout << "mutate_reverse_part" << endl;

    int route, start, stop;

    if(rand() % 2 == 0) {
        if(!RP_random(mutation, &route, &start, &stop)) 
            return NULL;
    } else {
        if(!RP_2_opt(mutation, &route, &start, &stop)) 
            return NULL;
    }

    

    mutation->op_reverse_part(route, start, stop);

    /*if (!mutation->check_locs("mutate_reverse_part")) {
        cout << "BUG";
    }*/
    return mutation;
}

void Routes::op_reverse_part(unsigned int r_index, unsigned int start_index, unsigned int end_index) {

    if (r_index >= this->routes.size()) {
        cerr << "RP r_index out of bounds";
        exit(-1);
    }

    Route* route = &this->routes[r_index];

    if (start_index >= route->length) {
        cerr << "start_index out of bounds";
        exit(-1);
    }

    if (end_index < start_index || end_index >= route->length) {
        cerr << "end_index out of bounds";
        exit(-1);
    }

    RouteNode* begin = &this->nodes[this->getPos(start_index, *route)];
    RouteNode* end = &this->nodes[this->getPos(end_index, *route)];

    // close gap
    if (start_index == 0) {
        route->start = end->next;
        if (end->next != 0)
            this->nodes[end->next].prev = 0;
    } else if (end_index == route->length-1) {
        this->nodes[begin->prev].next = 0;
    } else {
        this->nodes[begin->prev].next = end->next;
        this->nodes[end->next].prev = begin->prev;
    }

    // reverse
    begin->prev = 0;
    end->next = 0;

    RouteNode* curr = begin;
    RouteNode* temp;

    while (curr->location != 0) {
        temp = &this->nodes[curr->next];
        curr->next = curr->prev;
        curr->prev = temp->location;
        curr = &this->nodes[curr->prev];
    }

    //insert
    this->insertAt(route, start_index, end, begin);
    route->fitness = -1.0f;
}

bool MC_heuristic1(Routes* mutation, int* route1, int* pos1, int* route2, int* pos2) {
    // 2-opt*

    vector<vector<int>> solutions;

    // get random route
    for (size_t i=0; i<mutation->routes.size(); i++) {
        for (size_t j=i+1; j<mutation->routes.size(); j++) {
            Route* rand_route1 = &mutation->routes[i];
            Route* rand_route2 = &mutation->routes[j];

            RouteNode* l1 = &mutation->nodes[rand_route1->start];
            RouteNode* l2 = &mutation->nodes[rand_route2->start];

            *pos1 = 0;
            *pos2 = 0;

            while (l1->location != 0) {
                while (l2->location != 0) {
                    float d = mutation->vrp->distance(l1->prev, l1->location) 
                        + mutation->vrp->distance(l2->prev, l2->location)
                        - mutation->vrp->distance(l1->prev, l2->location)
                        - mutation->vrp->distance(l1->location, l2->prev);
                    if (d > 0 && abs(*pos1 - *pos2) < 5 ) {
                        solutions.push_back(vector<int> {(int)i, *pos1, (int)j, *pos2});
                    }
                    l2 = &mutation->nodes[l2->next];
                    *pos2 += 1;
                }
                l1 = &mutation->nodes[l1->next];
                *pos1 += 1;
                *pos2 = 0;
                l2 = &mutation->nodes[rand_route2->start];
            }


        }
    }

    if (solutions.size() == 0)
        return false;

    vector<int> sol = solutions[random_num(0,solutions.size())];
    *route1 = sol[0];
    *pos1 = sol[1];
    *route2 = sol[2];
    *pos2 = sol[3];

    return true;
}

bool MC_random(Routes* mutation, int* route1, int* pos1, int* route2, int* pos2) {

    int r_size = mutation->routes.size();

    if (r_size < 2)
        return false;

    *route1 = random_num(0,r_size);
    *route2 = random_num(0,r_size-1);

    if (*route2 >= *route1) {
        *route2 += 1;
    }

    Route* r1 = &mutation->routes[*route1];
    int r1_size = r1->length;

    Route* r2 = &mutation->routes[*route2];
    int r2_size = r2->length;

    if (r1_size == 0 || r2_size == 0)
        return false;

    *pos1 = random_num(0,r1_size);
    *pos2 = random_num(0,r2_size);

    return true;
}

Routes* mutate_cross(Routes* mutation) {
    /*
        HEURISTIEK:
            - cross met nodes dicht bij elkaar
            - cross met nodes met +/- dezelfde index
    */

   //cout << "mutate_cross" << endl;

    int route1, route2, pos1, pos2;
    bool res;
    if(mutation->vrp->getLocations().size() < 400) {
        res = MC_heuristic1(mutation, &route1, &pos1, &route2, &pos2);
    } else {
        res = MC_random(mutation, &route1, &pos1, &route2, &pos2);
    }

    if(!res)
        return NULL;

    mutation->op_cross(route1, pos1, route2, pos2);

    /*if(!mutation->check_locs("mutate_cross")) {
        cout << "bug";
    }*/
    return mutation;
}

void Routes::op_cross(unsigned int r_index1, unsigned int index1, unsigned int r_index2, unsigned int index2) {

    if (r_index1 >= this->routes.size() || r_index2 >= this->routes.size()) {
        cerr << "MC r_index out of bounds";
        exit(-1);
    }

    Route* r1 = &this->routes[r_index1];
    Route* r2 = &this->routes[r_index2];

    if (index1 >= r1->length || index2 >= r2->length) {
        cerr << "index out of bounds";
        exit(-1);
    }

    int r1_len = r1->length;
    int r2_len = r2->length;

    r1->length = index1 + (r2_len - index2);
    r2->length = index2 + (r1_len - index1);

    RouteNode* n1 = &this->nodes[this->getPos(index1, *r1)];
    RouteNode* n2 = &this->nodes[this->getPos(index2, *r2)];

    if (index1 == 0) {
        r1->start = n2->location;
    } else {
        this->nodes[n1->prev].next = n2->location;
    }

    if (index2 == 0) {
        r2->start = n1->location;
    } else {
        this->nodes[n2->prev].next = n1->location;
    }

    int temp = n1->prev;
    n1->prev = n2->prev;
    n2->prev = temp;

    r1->fitness = -1.0f;
    r2->fitness = -1.0f;
}

bool Routes::check_maxT() {
    for (Route r : this->routes) {
        if ( this->getRouteLength(&r) > Vehicle::maxT) {
            return false;
        }
    }
    return true;
}

float Routes::get_diversity(Routes* routes) {
    if (this == routes) return -1.0f;
    int  res = 0;
    for (RouteNode rn : routes->nodes) {
        RouteNode* trn = &this->nodes[rn.location];
        if (rn.next != trn->next && rn.next != trn->prev)
            res++;
        if (rn.prev != trn->prev && rn.prev != trn->next)
            res++;
    }
    return 1.0/(2.0 * this->nodes.size()) * res;
    
    /*for (Route route1 : this->routes) {
        int d_min = route1.length;
        for (Route route2 : routes->routes) {
            int dist = this->getRouteLevenshteinDist(route1, route2, routes);
            d_min = min(d_min, dist);
        }
        res += (route1.length - d_min) / route1.length;
    }
    return res/this->routes.size();
    */
}
/*
float get_diversity_levenstein(Routes* routes) {
    if (this == routes) return -1.0f;
    int  res = 0;
    for (Route route1 : this->routes) {
        int d_min = route1.length;
        for (Route route2 : routes->routes) {
            int dist = this->getRouteLevenshteinDist(route1, route2, routes);
            d_min = min(d_min, dist);
        }
        res += (route1.length - d_min) / route1.length;
    }
    return res/this->routes.size();
}*/

// Create routes_amount random routes of length route_length using the given locations
void Routes::make_rnd_routes() {
    size_t routes_amount = this->vrp->getVehicles().size();
    unsigned int mc = Vehicle::maxCapacity;
    size_t loc_amount = this->vrp->getLocations().size();
    unsigned int mc2 = (int)(loc_amount-1)/routes_amount;
    unsigned int route_length = min(mc, mc2);

    int arr[loc_amount-1];
    
    // create range(size)
    for (size_t i = 1; i < loc_amount; i++)  arr[i-1] = i;

    unsigned seed = rand();
    // shuffle range to get random indexes for routes
    shuffle(arr, arr+loc_amount-1, default_random_engine(seed));

    // assign every route_length random ints to a route and create it
    for (size_t i = 0; i < routes_amount; i++)
    {
        Route* route = &this->routes[i];
        route->start = arr[i*route_length];

        this->nodes[route->start].prev = 0;
        this->nodes[route->start].next = arr[i*route_length + 1];

        for (size_t j = 1; j < route_length-1; j++)
        {
            int curr = arr[i*route_length + j];
            this->nodes[curr].prev = arr[i*route_length + j - 1];
            this->nodes[curr].next = arr[i*route_length + j + 1];
        }
        // breaks if route_length < 2
        int curr  = arr[(i+1)*route_length - 1];
        this->nodes[curr].prev = arr[(i+1)*route_length - 2];
        this->nodes[curr].next = 0;

        route->length = route_length;
    }

    // the free locations consist off the remaining locations
    for (unsigned int i = routes_amount*route_length; i < loc_amount-1; i++) {
        this->free_locations.push_back(arr[i]);
    }
}

Location* depot;

struct comparator {
    comparator(Location* depot) { this->depot = depot; }
    bool operator () (Location* a, Location* b) {
        float arca = atan2(a->x - depot->x, a->y - depot->y);
        float arcb = atan2(b->x - depot->x, b->y - depot->y);

        return arca < arcb;
    }

    Location* depot;
};

/*bool comparator(Location* a, Location* b) {
    float arca = atan2(a->x - depot->x, a->y - depot->y);
    float arcb = atan2(b->x - depot->x, b->y - depot->y);

    return arca < arcb;
}*/

void Routes::make_nn_routes() {
    size_t routes_amount = this->vrp->getVehicles().size();
    unsigned int mc = Vehicle::maxCapacity;
    size_t loc_amount = this->vrp->getLocations().size();
    unsigned int mc2 = (int)(loc_amount-1)/routes_amount;
    unsigned int route_length = min(mc, mc2);

    vector<Location*> locs = this->vrp->getLocations();

    Location* depot = *locs.begin();

    locs.erase(locs.begin());

    sort(locs.begin(), locs.end(), comparator(depot));

    int shift = rand() % locs.size();
    int arr[loc_amount-1];

    for (unsigned int i = 0; i < locs.size(); i++) {
        arr[i] = locs[(shift+i) % locs.size()]->id;
    }

    // assign every route_length random ints to a route and create it
    for (size_t i = 0; i < routes_amount; i++)
    {
        Route* route = &this->routes[i];
        // length, start
        route->start = arr[i*route_length];

        unordered_set<int> locs;

        for (int j=0; j<route_length; j++) {
            locs.insert(arr[i*route_length+j]);
        }

        int prev = 0;

        for (int j=0; j<route_length; j++) {
            pair<float, int> sm = {FLT_MAX, -1};
            for (int loc : locs) {
                float dist = this->vrp->distance(prev, loc);
                if (dist < sm.first) {
                    sm = {dist, loc};
                }
            }
            if (prev == 0) {
                route->start = sm.second;
            } else {
                this->nodes[prev].next = sm.second;
                
            }
            this->nodes[sm.second].prev = prev;
            prev = sm.second;
            locs.erase(sm.second);
        }

        this->nodes[prev].next = 0;

        route->length = route_length;
    }

    // the free locations consist off the remaining locations
    for (unsigned int i = routes_amount*route_length; i < loc_amount-1; i++) {
        this->free_locations.push_back(arr[i]);
    }
}

// Create routes_amount routes of length route_length using the given locations
// partition routes by circle sector
void Routes::make_heuristic1_routes() {
    size_t routes_amount = this->vrp->getVehicles().size();
    unsigned int mc = Vehicle::maxCapacity;
    size_t loc_amount = this->vrp->getLocations().size();
    unsigned int mc2 = (int)(loc_amount-1)/routes_amount;
    unsigned int route_length = min(mc, mc2);

    vector<Location*> locs = this->vrp->getLocations();

    Location* depot = *locs.begin();

    locs.erase(locs.begin());

    sort(locs.begin(), locs.end(), comparator(depot));

    int shift = rand() % locs.size();
    int arr[loc_amount-1];

    for (unsigned int i = 0; i < locs.size(); i++) {
        arr[i] = locs[(shift+i) % locs.size()]->id;
    }

    // assign every route_length random ints to a route and create it
    for (size_t i = 0; i < routes_amount; i++)
    {
        Route* route = &this->routes[i];
        route->start = arr[i*route_length];

        this->nodes[route->start].prev = 0;
        this->nodes[route->start].next = arr[i*route_length + 1];

        for (size_t j = 1; j < route_length-1; j++)
        {
            int curr = arr[i*route_length + j];
            this->nodes[curr].prev = arr[i*route_length + j - 1];
            this->nodes[curr].next = arr[i*route_length + j + 1];
        }
        // breaks if route_length < 2
        int curr  = arr[(i+1)*route_length - 1];
        this->nodes[curr].prev = arr[(i+1)*route_length - 2];
        this->nodes[curr].next = 0;

        route->length = route_length;
    }

    // the free locations consist off the remaining locations
    for (unsigned int i = routes_amount*route_length; i < loc_amount-1; i++) {
        this->free_locations.push_back(arr[i]);
    }
}

void Routes::make_routes(vector<int>* rs) {
    size_t routes_amount = this->vrp->getVehicles().size();
    size_t loc_amount = this->vrp->getLocations().size();

    unordered_set<int> locs;

    // assign every route_length random ints to a route and create it
    for (size_t i = 0; i < routes_amount; i++)
    {
        Route* route = &this->routes[i];
        vector<int> r = rs[i];
        if (r.size() == 0) {
            route->length = 0;
            route->start = 0;
        } else {
            route->start = r[0];
            route->length = r.size();

            for (size_t i=0; i<r.size(); i++) {
                int curr = r[i];
                locs.insert(curr);
                if (i==0) {
                    this->nodes[curr].prev = 0;
                } else {
                    this->nodes[curr].prev = r[i-1];
                }
                if (i == r.size()-1) {
                    this->nodes[curr].next = 0;
                } else {
                    this->nodes[curr].next = r[i+1];
                }
            }   
        }
    }

    this->free_locations = {};

    // the free locations consist off the remaining locations
    for (size_t i = 1; i < loc_amount; i++) {
        if (locs.find(i) == locs.end()) {
            this->free_locations.push_back(i);
        }
    }
}
/*
void Routes::make_empty_routes() {
    int routes_amount = this->vrp->getVehicles().size();
    int loc_amount = this->vrp->getLocations().size();

    // assign every route_length random ints to a route and create it
    for (size_t i = 0; i < routes_amount; i++)
    {
        Route* route = new Route();
        this->routes.push_back(route);
    }

    for (int i=1; i < loc_amount; i++)
        this->free_locations.push_back(i);
}*/

void Routes::insertAt(Route* route, int pos, RouteNode* begin, RouteNode* end) {
    if (route->start == 0) {
        begin->prev = 0;
        route->start = begin->location;
        end->next = 0;
    }
    else if (pos == 0) {
        begin->prev = 0;
        end->next = route->start;
        this->nodes[end->next].prev = end->location;
        route->start = begin->location;
    } else {
        RouteNode* insert = &this->nodes[this->getPos(pos-1, *route)];
        if (insert->next == 0) {
            insert->next = begin->location;
            begin->prev = insert->location;
            end->next = 0;
        } else {
            end->next = insert->next;
            if (end->next == 0 || end->next == -1) {
                cout << "BUGG 69 " << end->next <<  endl;
            }
            this->nodes[end->next].prev = end->location;
            if (begin == 0) {
                cout << "BUG";
            }
            begin->prev = insert->location;
            this->nodes[begin->prev].next = begin->location;
        }
    } 
}

void Routes::addLocation(unsigned int loc) {
    size_t nodes_size = this->nodes.size();
    if (nodes_size == loc) {
        this->nodes.push_back({loc, -1, -1});
    } else if (loc < nodes_size && (this->nodes[loc].prev != -1 || this->nodes[loc].next != -1)) {
        cerr << "Error: added location was not set to NULL!" << endl;
        exit(-1);
    } else if (loc > nodes_size) {
        cerr << "Error: added location to high!" << endl;
        exit(-1);
    }

    this->free_locations.push_back(loc);
}

int Routes::next(int node) {
    return nodes[node].next;
}

int Routes::prev(int node) {
    return nodes[node].prev;
}

int Routes::getPos(int n, Route& route) {
    int curr = route.start;
    for (int i=0; i<n; i++) {
        curr = next(curr);
    }
    return curr;
}

void Routes::getRouteIndex(int loc, int* route, int* index) {

    RouteNode* node = &this->nodes[loc];

    if (node->prev == -1) {
        *route = -1;
        *index = -1;
        return;
    }
        
    int pos = 0;
    while (node->prev != 0) {
        node = &this->nodes[node->prev];
        pos++;
    }

    for (size_t i=0; i<this->routes.size(); i++) {
        if (this->routes[i].start == (int)node->location) {
            *index = pos;
            *route = i;
            return;
        }
    }
}

void Routes::fill_location_route_map() {
    for (unsigned int i=0; i< this->routes.size(); i++) {
        int c = this->routes[i].start;
        while (c != 0) {
            this->location_route_map[c] = i;
            c = this->next(c);
        }
    }
    for (int loc: this->free_locations) {
        this->location_route_map[loc] = -1;
    }
}

Routes *Routes::mutate(Routes *mutation, bool clone) {
    if (clone)
        this->clone_to(mutation);
    
    vector<mutationFunc> mutations = {
        mutate_swap_free, 
        mutate_swap_pos, 
        mutate_cross_over,
        mutate_add,
        mutate_remove,
        mutate_reverse_part,
        mutate_cross
    };
    Routes* result;
    int mut;

    //auto startt = std::chrono::system_clock::now();

    do {
        mut = rand() % mutations.size();
        result = mutations[mut](mutation);
        //cout << (result == NULL) << " " << check_maxT() << " " << mut << endl;
    } while (result == NULL);

    //mutation->check_locs("mutate3");
    //auto t = std::chrono::system_clock::now() - startt;
    //cout << mut << ": " << t.count() << endl;
    result->last_mutation = mut;

    return result;
}