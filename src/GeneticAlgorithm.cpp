#include <bits/stdc++.h>
#include "Location.h"
#include "DVRPD.h"
#include "Tests.h"
#include "Routes.h"
#include <pthread.h>
#include <mutex>
using namespace std;
 
// Number of individuals in each generation
#define POPULATION_SIZE 10
#define MAX_IT_RESTART 100000
#define MAX_ITERATIONS 1000000
#define KEPT_FRACTION 0.7
#define DRONES_AMOUNT 0
#define VEHICLES_AMOUNT 6
#define LOCATIONS_AMOUNT 10
#define MAX_X 1000
#define MAX_Y 1000
#define MAX_TIME 3600 // 1 hour
#define MAX_IT_NO_IMP 30000

struct thread_data {
    int thread_id;
    multiset<pair < float, Routes* >>* popu;
    DVRPD* vrp;
    mutex* mtx;
    mutex* best_mtx;
    pair <float, Routes*>* best;
    time_t start_time;
};

int max_time = MAX_TIME;
string out_file = "routes.txt";

// get diversity of a solution in the population
float getDiversity(Routes* routes, float* divs) {
    const int top_n = 5;
    multiset<float> neighbors;
    float div = 0;
    int id = routes->id;
    for (int i=0; i<(POPULATION_SIZE+1); i++) {
        if (i==id)
            continue;
        int index = 0;
        if (i<id) {
            index = id * (POPULATION_SIZE+1) + i;
        } else {
            index = i * (POPULATION_SIZE+1) + id;
        }

        neighbors.insert(divs[index]);    
    } 
    auto it = neighbors.begin();
    for (int i=0; i<top_n; i++) 
        div += *it;
    
    return div/top_n;
}

void print_diversity(multiset<pair < float, Routes* >>* population, float* divs) {
    //TODO diversity very bad with ratioFitness
    cout << "Diversity:" << endl;
    auto it = population->begin();
    while(it != population->end()) {
        cout << getDiversity(it->second, divs) << ", " << it->first << "," <<  it->second->getFitness() << endl;
        it++;
    }  
}

// get biased fitness score based on fitness and diversity
float getBiasedFitness(Routes* routes, float best, float* divs) {
    float fitness = routes->getFitness();

    const float fitness_perc = 0.9;

    if (fitness < best) 
        return fitness;

    float diversity = getDiversity(routes, divs);

    return fitness_perc * fitness + (1.0-fitness_perc) * fitness * diversity;
}

// calculate and set diversity of a new solution in a population
void setDiversity(Routes* routes, float* divs, multiset<pair < float, Routes* >>* population) {
    auto it = population->begin();
    int id1 = routes->id;

    while (it != population->end()) {
        int id2 = it->second->id;
        if (id1 == id2) {
            it++;
            continue;
        }
        float div = it->second->get_diversity(routes);

        int index = 0;

        if (id1 < id2) {
            index = id2 * (POPULATION_SIZE+1) + id1;
        } else {
            index = id1 * (POPULATION_SIZE+1) + id2;
        }

        divs[index] = div;
        it++;
    }
}

// run our genetic algorithm on the given population
void runGeneticAlgorithm(multiset<pair < float, Routes* >>* population, int min_it, int max_it, int max_it_no_imp, bool print_score) {

    int runs = 0;
    int generation = 0;

    float best_fitness = FLT_MAX;

    float divs[(POPULATION_SIZE+1)*(POPULATION_SIZE+1)] = {};

    auto it = population->begin();

    int id = 0;
    while (it != population->end()) {
        it->second->id = id;
        it++;
        id++;
    }

    it = population->begin();

    if (it->first == -210937.5) {
        cout << "here";
    }

    while (it != population->end()) {
        setDiversity(it->second, divs, population);
        it++;
    }

    multiset<pair < float, Routes* >> clone = *population;

    population->clear();

    it = clone.begin();

    while (it != clone.end()) {
        pair < float, Routes* > sol = *it;
        for (unsigned int i=0; i< sol.second->routes.size(); i++) {
            sol.second->routes[i].fitness= -1.0;
        }
        float curr_fitness = getBiasedFitness(sol.second, -FLT_MAX, divs);
        population->insert({curr_fitness, sol.second});
        it++;
    }

    clone.clear();

    int muts[8] = {};
 
    while(generation < min_it || (generation < max_it && runs < max_it_no_imp))
    {
        runs++;
        generation++;

        float curr_fitness = population->begin()->first;

        if (curr_fitness < best_fitness) {
            best_fitness = curr_fitness;
            runs = 0;
        }
 
        // Perform Elitism, that mean 10% of fittest population
        // goes to the next generation

        pair<float, Routes*> mutation_pair = *population->rbegin();
        Routes* mutation = mutation_pair.second;

        population->erase(mutation_pair);
 
        
        int r = rand() % 100;
        // 80% mutation 20% cross-over
        if (r >= 10) {
            //MUTATION
            auto it = population->begin();
            advance(it, rand() % POPULATION_SIZE);
            Routes* parent = it->second;
            parent->mutate(mutation, true);
        } else {
            // CROSS-OVER
            auto it = population->begin();
            advance(it, rand() % POPULATION_SIZE);
            Routes* parent1 = it->second;
            it = population->begin();
            advance(it, rand() % POPULATION_SIZE);
            Routes* parent2 = it->second;
            parent1->cross_over(parent2, mutation);
            mutation->last_mutation=7;
        }

        setDiversity(mutation, divs, population);
        float mf = getBiasedFitness(mutation, best_fitness, divs);

        if (mf <= population->begin()->first) {
            pair<float, Routes*> prev_best = *population->begin();
            population->insert({mf, mutation});
            population->erase(prev_best);
            float bf = getBiasedFitness(prev_best.second, mf, divs);
            population->insert({bf, prev_best.second});

            muts[mutation->last_mutation]++;
        } else {
            population->insert({mf, mutation});
        }
        
        //uncomment for extra debug info
        /*if (generation % 10000 == 0) {
            //cout<< "Generation: " << generation << "\t";
            //cout<< "Fitness: "<< population->begin()->second->getFitness() << "\n";
            //cout<< "makespan: "<< population->begin()->second->getMakespan() << "\n";

            cout << population->begin()->second->getMakespan() << "\n";

            //for (int i=0; i<8; i++) {
            //    cout << muts[i] << ", ";
            //}
            
            //print_diversity(population, divs);
        }*/
     }

     cout<< "Generation: " << generation << "\t";
     cout<< "Fitness: "<< population->begin()->second->getFitness() << "\n";
     cout<< "Distance: "<< population->begin()->second->getTotalDistance() << "\t";
     cout<< "D/H: "<< population->begin()->second->getFitnessDeliveriesPerDistance() << "\t";
     cout<< "makespan: "<< population->begin()->second->getMakespan() << "\n";

     if (print_score)
        print_diversity(population, divs);

}

// keep top n of the population and remove others
void get_top_n(multiset<pair < float, Routes* >>* population, unsigned int n) {
    auto it = population->begin();
    advance(it, n);

    auto it_s = it;

    while (it != population->end()) {
        delete it->second;
        it++;
    }

    population->erase(it_s, population->end());
}

// get a solution from random restart
pair <float, Routes*> solution_from_random(DVRPD* vrp, int min_it, int max_it, int it_no_imp) {
    multiset<pair < float, Routes* >> population;

    // create random population
    for(int i = 0;i<POPULATION_SIZE+1;i++) {
        Routes* genome = new Routes(vrp);
        genome->make_nn_routes();
        population.insert({0.0f, genome});
    }
    runGeneticAlgorithm(&population, min_it, max_it, it_no_imp, false);

    get_top_n(&population, 1);
    return *population.begin();
}

// check if given solution is better than the best, and if so, write away best solution
void check_best(pair <float, Routes*>* best, pair <float, Routes*>* sol) {
    if (sol->first < best->first) {
        *best = *sol;
        ofstream file(out_file);
        sol->second->print(file);
        file.close();
    }
}

// check fitness function on given vrp solution file 
void check_solution(DVRPD* vrp, string file) {
    Routes* routes = new Routes(vrp);
    file.resize(file.size() - 3);
    file.append("sol");
    routes->routes_from_file(file);

    cout << "max fitness: " << routes->getFitness() << endl;
    cout << "min distance fitness: " << routes->getTotalDistance() << endl;
    cout << "min makespan: " << routes->getFitnessMakespan() << " h" << endl;

    delete routes;
}

// run our head worker that processes head population with best solutions
void* run_head_thread(void* threadarg) {
    struct thread_data *my_data;
    my_data = (struct thread_data *) threadarg;

    cout << "Thread created with ID : " << my_data->thread_id << endl;

    while(time(0) - my_data->start_time < max_time) {

        my_data->mtx->lock();

        if (my_data->popu->size() >= POPULATION_SIZE+1) {

            cout << "HEAD IS PROCESSING" << endl;

            multiset<pair < float, Routes* >> population;

            for (int i=0; i<POPULATION_SIZE+1; i++) {
                pair <float, Routes*> sol = *my_data->popu->begin();
                my_data->popu->erase(my_data->popu->begin());
                population.insert(sol);
            }

            my_data->mtx->unlock();

            runGeneticAlgorithm(&population, 100000, MAX_ITERATIONS, MAX_IT_NO_IMP, true);

            get_top_n(&population, POPULATION_SIZE*KEPT_FRACTION+1);

            my_data->mtx->lock();

            auto it = population.begin();

            while (it != population.end()) {
                my_data->popu->insert(*it);
                it++;
            }

            my_data->mtx->unlock();

            pair <float, Routes*> sol = *population.begin();

            my_data->best_mtx->lock();
            if (my_data->best->first > sol.first) {
                check_best(my_data->best, &sol);
            }
            my_data->best_mtx->unlock();
            
        } else {
            cout << "HEAD IS WORKING" << endl;

            my_data->mtx->unlock();
            auto startt = std::chrono::system_clock::now();
            pair <float, Routes*> sol = solution_from_random(my_data->vrp, 0, MAX_IT_RESTART, MAX_IT_NO_IMP);
            std::chrono::duration<double> t = std::chrono::system_clock::now() - startt;
            cout << "time: " << t.count() << endl;
            my_data->mtx->lock();
            my_data->popu->insert(sol);
            my_data->mtx->unlock();

            my_data->best_mtx->lock();
            if (my_data->best->first > sol.first) {
                check_best(my_data->best, &sol);
            }
            my_data->best_mtx->unlock();
        }

    }

    pthread_exit(NULL);
}

// run worker thread that does random restarts and processes them if we have too many
void* run_thread(void* threadarg) {
    struct thread_data *my_data;
    my_data = (struct thread_data *) threadarg;

    while(time(0) - my_data->start_time < max_time) {

        my_data->mtx->lock();

        if (my_data->popu->size() >= POPULATION_SIZE*2) {

            cout << "WORKER IS PROCESSING" << endl;

            multiset<pair < float, Routes* >> population;

            for (int i=0; i<POPULATION_SIZE+1; i++) {
                auto it = my_data->popu->begin();
                advance(it, POPULATION_SIZE - POPULATION_SIZE*KEPT_FRACTION);
                pair <float, Routes*> sol = *it;
                my_data->popu->erase(it);
                population.insert(sol);
            }

            my_data->mtx->unlock();

            runGeneticAlgorithm(&population, 100000, MAX_ITERATIONS, MAX_IT_NO_IMP, true);

            get_top_n(&population, 2);

            my_data->mtx->lock();

            auto it = population.begin();

            while (it != population.end()) {
                my_data->popu->insert(*it);
                it++;
            }

            my_data->mtx->unlock();


            pair <float, Routes*> sol = *population.begin();

            my_data->best_mtx->lock();
            if (my_data->best->first > sol.first) {
                check_best(my_data->best, &sol);
            }
            my_data->best_mtx->unlock();
            
        } else {
            cout << "WORKER IS WORKING, POPU SIZE: " << my_data->popu->size() << endl;

            my_data->mtx->unlock();
            
            auto startt = std::chrono::system_clock::now();
            pair <float, Routes*> sol = solution_from_random(my_data->vrp, 0, MAX_IT_RESTART, MAX_IT_NO_IMP);
            std::chrono::duration<double> t = std::chrono::system_clock::now() - startt;
            cout << "time: " << t.count() << endl;
            my_data->mtx->lock();
            my_data->popu->insert(sol);
            my_data->mtx->unlock();

            my_data->best_mtx->lock();
            if (my_data->best->first > sol.first) {
                check_best(my_data->best, &sol);
            }
            my_data->best_mtx->unlock();
        }

    }

    pthread_exit(NULL);
}

int main(int argc, char *argv[]) {
    //uncomment for running tests
    //Tests* t = new Tests();
    //t->runTests();

    srand((unsigned)(time(0)));

    if (argc != 5 && argc != 7) {
        cout << "USAGE: ./hgsd <vrp_path> <vehicles_amount> <drones_amount> <thread_num> <max_time> <out_file>" << endl;
        exit(1);
    }

    if (argc == 7) {
        max_time = stoi(argv[5]);
        out_file = argv[6];
    }

    // create VRP with defined amount of drones, vehicles and locations
    DVRPD* vrp = new DVRPD(argv[1], stoi(argv[3]), stoi(argv[2]));

    // uncomment to use random vrp instead of file
    //DVRPD* vrp = new DVRPD(DRONES_AMOUNT,VEHICLES_AMOUNT);
    //vrp->createRandomLocations(LOCATIONS_AMOUNT, MAX_X, MAX_Y);

    int thread_num = stoi(argv[4]);

    pthread_t threads[thread_num];
    thread_data td[thread_num];
    mutex mtx, best_mtx;
    void *status;

    // uncomment to check solution in same map
    // check_solution(vrp, argv[1]);

    multiset<pair < float, Routes* >> new_sols;

    pair <float, Routes*> best_sol = {FLT_MAX, NULL};

    //set UneligibleCapacity
    vrp->getDroneUneligibleCapacity();

    for (int i=0; i<thread_num; i++) {
        td[i] = {i, &new_sols, vrp, &mtx, &best_mtx, &best_sol, time(0)};
        if (i == 0) {
            pthread_create(&threads[i], NULL, run_head_thread, (void *)&td[i]);
        } else {
            pthread_create(&threads[i], NULL, run_thread, (void *)&td);
        }
        
    }

    for (int i=0; i<thread_num; i++) {
        pthread_join(threads[i], &status);
    }

    for (auto it = new_sols.begin(); it != new_sols.end(); it++) {
        delete it->second;
    }

    delete vrp;
}