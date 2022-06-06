#include "DVRPD.h"

// constructor for VRP with giveven amount of drones and vehicles
DVRPD::DVRPD(unsigned int drones, unsigned int vehicles) {
    this->location_map = new unordered_set<int>[MAP_WIDTH*MAP_HEIGHT];
    
    Location* depot = addLocation(0, 0);
    this->depot = depot;

    for (size_t i = 0; i < drones; i++)
    {
        this->drones.push_back(new Drone());
    }
    for (size_t i = 0; i < vehicles; i++)
    {
        this->vehicles.push_back(new Vehicle());
    }
}

// constructor for VRP from a vrp file with given amount of drones and vehicles
DVRPD::DVRPD(string path, unsigned int drones, unsigned int vehicles) {
    string line;
    ifstream myfile(path);

    this->location_map = new unordered_set<int>[MAP_WIDTH*MAP_HEIGHT];

    if (myfile.is_open()) {
        while (getline(myfile,line)){
            if(line.substr(0,18) == "NODE_COORD_SECTION") {

                // depot
                getline(myfile,line);
                stringstream ss(line);
                string word;
                int i = 0;
                float x;
                float y;
                while (ss >> word) {
                    if (i==1) {
                        x = stof(word);
                    } else if(i==2) {
                        y = stof(word);
                    }
                    i++;
                }
                Location* depot = addLocation(x, y);
                this->depot = depot;

                
                while (getline(myfile,line) && line.substr(0,14) != "DEMAND_SECTION"){
                    stringstream ss(line);
                    string word;
                    int i = 0;
                    float x;
                    float y;
                    while (ss >> word) {
                        if (i==1) {
                            x = stof(word);
                        } else if(i==2) {
                            y = stof(word);
                        }
                        i++;
                    }

                    addLocation(x, y);
                }
            }
            if (line.substr(0,14) == "DEMAND_SECTION") {
                while(getline(myfile,line) && line.substr(0,13) != "DEPOT_SECTION") {
                    stringstream ss(line);
                    string word;
                    int i = 0;
                    float index;
                    float capacity;
                    while (ss >> word) {
                        if (i==0) {
                            index = stoi(word);
                        } else if(i==1) {
                            capacity = stof(word);
                        }
                        i++;
                    }

                    addCapacity(index-1, capacity);
                }
            }
        }
        myfile.close();
    } else cout << "Unable to open file"; 

    for (size_t i = 0; i < drones; i++)
    {
        this->drones.push_back(new Drone());
    }
    for (size_t i = 0; i < vehicles; i++)
    {
        this->vehicles.push_back(new Vehicle());
    }
}

DVRPD::~DVRPD(){
    for (Drone* drone: this->drones) {
        delete drone;
    }
    for (Vehicle* vehicle : this->vehicles) {
        delete vehicle;
    }
    for (Location* location : this->locations) {
        delete location;
    }

    for (float* arr : this->distance_matrix) {
        delete [] arr;
    }

    delete [] this->location_map;
}

float randomFloat(float min, float max) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = max - min;
    float r = random * diff;
    return min + r;
}

// create random location between (0,0) and (max_x, max_y)
void DVRPD::add_rand_location(float max_x, float max_y) {
    addLocation(randomFloat(0, max_x), randomFloat(0, max_y));
}

// create amount random locations between (0,0) and (max_x, max_y)
void DVRPD::createRandomLocations(unsigned int amount, float max_x, float max_y) {
    for (size_t i = 0; i < amount; i++)
    {
        add_rand_location(max_x, max_y);
    }
}

// Add location to the VRP and fill distance matrix and location_map
Location* DVRPD::addLocation(float x, float y) {

    Location* location;

    int index;

    // fill distance matrix
    if(!this->free_ids.empty()) {
        index = *this->free_ids.begin();
        this->free_ids.erase(index);
        location = new Location(index, x, y);
        for (int i=0; i<index; i++) {
            if (this->free_ids.find(i) == this->free_ids.end()) {
                this->distance_matrix[index][i] = location->distance(this->locations[i]);
            }
        }
        for (int i=index+1; i<(int)this->distance_matrix.size(); i++) {
            if (this->free_ids.find(i) == this->free_ids.end()) {
                this->distance_matrix[i][index] = location->distance(this->locations[i]);
            }
        }
        this->locations[index] = location;
    } else {
        index = this->distance_matrix.size();
        location = new Location(index, x, y);
        this->locations.push_back(location);
        this->distance_matrix.push_back(new float[index+1]);
        for (int i=0; i<=index; i++) {
            this->distance_matrix[index][i] = location->distance(this->locations[i]);
        }
    }

    int m_x = min(location->x, MAX_X - 1.0f) / (MAX_X/MAP_WIDTH);
    int m_y = min(location->y, MAX_Y - 1.0f)  / (MAX_Y/MAP_HEIGHT);
    this->location_map[m_x + m_y*MAP_WIDTH].insert(index);
    
    return location;
}

// remove location from vrp
void DVRPD::removeLocation(int loc) {
    this->free_ids.insert(loc);
    delete this->locations[loc];
    this->locations[loc] = NULL;
}

vector<Location*>& DVRPD::getLocations() {
    return this->locations;
}

vector<Vehicle*>& DVRPD::getVehicles() {
    return this->vehicles;
}

vector<Drone*>& DVRPD::getDrones() {
    return this->drones;
}

// get neighbours of location with given index and range of blocks around the location
unordered_set<int> DVRPD::getNeighbours(int index, int range) {
    Location* loc = this->locations[index];
    int x = loc->x / (MAX_X/MAP_WIDTH);
    int y = loc->y / (MAX_Y/MAP_HEIGHT);

    unordered_set<int> result;

    for (int i=-range; i<=range; i++) {
        for (int j=-range; j<=range; j++) {
            int nx = x-i;
            int ny = y-j;
            if (nx >= 0 && ny >= 0 && nx < MAP_WIDTH && ny < MAP_HEIGHT) {
                result.insert(this->location_map[nx+ny*MAP_WIDTH].begin(), this->location_map[nx+ny*MAP_WIDTH].end());
            }
        }
    }

    return result;
}

// get random neighbour of location with given index within range of blocks around the location
int DVRPD::getRandomNeighbour(int index, int range) {
    Location* loc = this->locations[index];
    int x = loc->x / (MAX_X/MAP_WIDTH);
    int y = loc->y / (MAX_Y/MAP_HEIGHT);

    int rx = min(MAP_WIDTH, x+range) - max(0,x-range) + 1;
    int ry = min(MAP_HEIGHT, y+range) - max(0,y-range) + 1;

    int nx = (rand() % rx) + max(0,x-range);
    int ny = (rand() % ry) + max(0,y-range);

    unordered_set<int> neighbourhood = this->location_map[nx+ny*MAP_WIDTH];

    int size = neighbourhood.size();

    if (size == 0)
        return -1;
    
    int rand_index = rand() % size;
    auto curr = neighbourhood.begin();
    advance(curr, rand_index);
    return *curr;
}

// Euclidian distance between location a and b
float DVRPD::distance(int a, int b) {
    return this->distance_matrix[max(a, b)][min(a, b)];
}

// get drone uneligible locations based on index for 30% and 70% based on furthest location from depot 
unordered_set<int>& DVRPD::getDroneUneligible() {
    if (this->droneEligible == 100 || this->drone_unelgible.size() != 0) {
        return this->drone_unelgible;
    }

    int m = (int) floor(1/((1-this->droneEligible/100.0)*0.3));

    for (unsigned int i=m; i<locations.size(); i+=m) {
        this->drone_unelgible.insert(i);
    }

    set<pair<float, int>> dists;

    for (unsigned int i=0; i<locations.size(); i++) {
        dists.insert({-this->distance(0, i), i});
    }

    auto it = dists.begin();

    while(this->locations.size() - this->drone_unelgible.size() > this->locations.size() * this->droneEligible/100.0) {
        this->drone_unelgible.insert(it->second);
        it++;
    }

    return this->drone_unelgible;
}

// get drone uneligible locations based on capacity
unordered_set<int>& DVRPD::getDroneUneligibleCapacity() {
    if (this->droneEligible == 100 || this->drone_unelgible.size() != 0) {
        return this->drone_unelgible;
    }

    set<pair<float, int>> caps;

    for (unsigned int i=0; i<locations.size(); i++) {
        caps.insert({-this->capacities[i], i});
    }

    auto it = caps.begin();

    while((int) this->locations.size() - (int) this->drone_unelgible.size() - 1 >= this->locations.size() * this->droneEligible/100.0) {
        this->drone_unelgible.insert(it->second);
        it++;
    }

    return this->drone_unelgible;
}

// Add demand capacity to location with index
float DVRPD::addCapacity(unsigned int index, float capacity) {
    while (index >= this->capacities.size()) {
        this->capacities.push_back(0.0f);
    }

    this->capacities[index] = capacity;
    return capacity;
}