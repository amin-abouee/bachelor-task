## bachelor-task


## How to build 
```
mkdir build
cd build
cmake ..
make -j 8 .
```


## How to run
First of all, you need to set the config.json parameters.
* file_paths
    * elevation_filepath (string): Relative or absolute path of assets/elevation.data (absolute path is better!)
    * overrides_filepath (string): Relative or absolute path of assets/overrides.data (absolute path is better!)
* constraints
    * image_dimension (unsigned int): Size of image or squared grid map
    * rover_loc ([int, int]): Location of rover (You can not set it in water or elevation 0)
    * bachelor_loc ([int, int]): Location of bachelor party (You can not set it in water or elevation 0)
    * wedding_loc ([int, int]): Location of wedding party (You can not set it in water or elevation 0)
* shortest_path_parameters
    * up_hill_cost_model (string): model for up hill cost estimation (["octile, peak, mean-peak, l2, l1, linf, angle, dificulty-level, l2-trim"] -> Cost class)
    * down_hill_cost_model (string): model for down hill cost estimation (["octile, peak, mean-peak, l2, l1, linf, angle, dificulty-level, l2-trim"] -> Cost class)
    * heuristic_model (string): A* heuristic model (["dijkstra, l1, l2, diagonal, l1-altitude, l2-altitude, diagonal-altitude"] -> Heuristic class)

Cmake generates the executable files in **../bin/{type_of_build}/** folders
```
build> ../bin/release/bachelor ../config/config.json
```

## Run test
```
build> ../bin/release/bachelor_test
```