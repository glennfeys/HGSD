# Hybrid Genetic Search with Drones for solving PDSVRP

Hybrid Genetic Search with Drones, HGSD in short is an algorithm to efficiently solve PDSVRP problems. It is based on the HGS-ADC algorithm of [Vidal et al.](https://doi.org/10.1287/opre.1120.1048). But has a lot of new ideas and mutations, and is adapted to work with drones. The program is written for a masters thesis "Dynamic vehicle routing problem using heterogenous fleet of vehicles and drones"  the thesis was written in Dutch and is also included as `thesis_DVRPD_Dutch.pdf` but an extended abstract was also made with the main ideas and accomplishments of the thesis and the program, also including benchmarks with papers from [Salue et al.](https://doi.org/10.1016/j.ejor.2021.08.014), [Raj et al.](http://dx.doi.org/10.2139/ssrn.3879710). Showing that our algorithm gives the current best results for almost all of the large-scale PDSVRP benchmark instances. 

## Getting started
### Installation
```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```
or
```
bash build.sh
```

### Usage
```
./hgsd <vrp_path> <vehicles_amount> <drones_amount> <thread_num> <max_time> <out_file>
```
- vrp_path: Path of VRP file in the format used in [CVRPLIB](http://vrp.atd-lab.inf.puc-rio.br/index.php/en/)
- vehicles_amount: Amount of vehicles to use in solution
- drones_amount: Amount of drones to use in solution
- thread_num: Number of threads to use in calculations
- max_time: Maximum time the algorithm can take
- out_file: Path to write back the outpot solution file.

Example:
```
./hgsd ../data/Vrp-Set-X/X/X-n411-k19.vrp 10 9 6 100 ./routes.sol
```

## Output format
We used a new output format inspired by the Output format used in cVRP in solutions of CVRP.

This is an example of a solution file for PDSVRP that we produce
```
Truck Route #1: 7 84 51 39 15 8 57 44 72 117 66 19 61 48 65 60 59 42 67 100 134 102 81
Truck Route #2: 98 71 135 33 101 11 17 47 119 128 116 68 83 1 18 21 103 28 138
Truck Route #3: 124 88 70 62 4 23 9 26 108 94 69 38 131 34 115 27 123 50 97 13 58 35 110 45
Truck Route #4: 121 20 120 78 80 41 92 107 132 93 127 130 106 77 43 46 82 75
Truck Route #5: 16 32 85 114 137 25 125 36 129 53 96 5 6 113 90 12 49 2 99 87
Drone Route #6: 112 14 24 63 118 89 56
Drone Route #7: 22 136 86 122 52 54 30
Drone Route #8: 37 133 111 10 109 29
Drone Route #9: 40 31 74 104 73 76 3
Drone Route #10: 55 64 105 126 95 79 91
Makespan 1930.35
```
it contains the routes the trucks and drones follow without the depot in the beginning and end for trucks and between every location for drones. At the end we give the Makespan.

The Makespan is calculated by the maximum Euclidian distance of a route of a truck/drone with drones havving a speed factor that reduces the distance by a factor (default is 2). 


## plotting solutions and 
plotting the sollutions can also be done in `notebooks/plotRoutes.ipynb` here the makespan calculations used in the paper are also programmed in python so people can easily check their new sollutions with our evaluation method. 

## Settings
fitness functions used to evaluate other papers or other fitness functions discussed in our paper are also available in `src/routes.cpp`. To use these you just have to change to this function in the `getFitness` function. The ones used by Salue et al. and Raj et al. are present.

we also have implemented 2 ways of `getDroneUneligibleCapacity` which use highest capacity to get drone unelegible locations and `getDroneUneligible` which uses the technique described by [Salue et al.]( https://doi.org/10.1002/net.21846) that uses index for 30% and furthest locations for the other 70%. to change between these 2 you have to change it in the main of `src/GeneticAlgorithm.cpp`.

We can also change some constants of drones in `include/Drone.h` percentage drone elegible locations in `include/DVRPD.h` and other constants like population size and such in `src/GeneticAlgorithm.cpp`.

In the future we better use a config file to set these settings of the algorithm.