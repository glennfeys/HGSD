# Hybrid Genetic Search with Drones for solving PDSVRP

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
./hgsd ../data/Vrp-Set-X/X/X-n411-k19.vrp 10 1 6 100 ./routes.txt
```
