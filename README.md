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

## Test Clang and GCC 

Os:	Mac OS X
Version: 10.14.5
C++ compile: Apple LLVM version 10.0.1 (clang-1001.0.46.4)
Target: x86_64-apple-darwin18.6.0
Thread model: posix
InstalledDir: /Library/Developer/CommandLineTools/usr/bin

```
➜  build git:(master) make -j 8
Scanning dependencies of target visualizer
Scanning dependencies of target source
[  6%] Building CXX object visualizer/CMakeFiles/visualizer.dir/visualizer.cpp.o
[ 12%] Building CXX object src/CMakeFiles/source.dir/cell_info.cpp.o
[ 18%] Building CXX object src/CMakeFiles/source.dir/a_star.cpp.o
[ 31%] Building CXX object src/CMakeFiles/source.dir/square_grid_graph.cpp.o
[ 31%] Building CXX object src/CMakeFiles/source.dir/matrix.cpp.o
[ 43%] Building CXX object src/CMakeFiles/source.dir/shortest_path.cpp.o
[ 43%] Building CXX object src/CMakeFiles/source.dir/cost.cpp.o
[ 50%] Building CXX object src/CMakeFiles/source.dir/up_hill_cost.cpp.o
[ 56%] Building CXX object src/CMakeFiles/source.dir/down_hill_cost.cpp.o
[ 62%] Building CXX object src/CMakeFiles/source.dir/heuristic.cpp.o
[ 68%] Linking CXX static library libvisualizer.a
[ 68%] Built target visualizer
[ 75%] Linking CXX shared library libsource.dylib
[ 75%] Built target source
Scanning dependencies of target bachelor_test
Scanning dependencies of target bachelor
[ 81%] Building CXX object test/CMakeFiles/bachelor_test.dir/test.cpp.o
[ 87%] Building CXX object src/CMakeFiles/bachelor.dir/main.cpp.o
[ 93%] Linking CXX executable ../../bin/release/bachelor_test
[ 93%] Built target bachelor_test
[100%] Linking CXX executable ../../bin/release/bachelor
[100%] Built target bachelor
```


Os:	Mac OS X
Version: 10.14.5
C++ compile: g++-9 (Homebrew GCC 9.1.0) 9.1.0

```
➜  build git:(master) ✗ make -j 8
[ 14%] Building CXX object src/CMakeFiles/source.dir/cell_info.cpp.o
[ 14%] Building CXX object visualizer/CMakeFiles/visualizer.dir/visualizer.cpp.o
[ 35%] Building CXX object src/CMakeFiles/source.dir/square_grid_graph.cpp.o
[ 35%] Building CXX object src/CMakeFiles/source.dir/shortest_path.cpp.o
[ 35%] Building CXX object src/CMakeFiles/source.dir/matrix.cpp.o
[ 50%] Building CXX object src/CMakeFiles/source.dir/a_star.cpp.o
[ 50%] Building CXX object src/CMakeFiles/source.dir/cost.cpp.o
[ 57%] Building CXX object src/CMakeFiles/source.dir/up_hill_cost.cpp.o
[ 64%] Building CXX object src/CMakeFiles/source.dir/heuristic.cpp.o
[ 71%] Building CXX object src/CMakeFiles/source.dir/down_hill_cost.cpp.o
[ 78%] Linking CXX static library libvisualizer.a
[ 78%] Built target visualizer
/Users/amin/Workspace/cplusplus/bachelor-task/src/cost.cpp: In member function 'double Cost::computeCost(const CellLocation&, uint8_t, const CellLocation&, uint8_t, const Cost::CostModel&)':
/Users/amin/Workspace/cplusplus/bachelor-task/src/cost.cpp:54:1: warning: control reaches end of non-void function [-Wreturn-type]
   54 | }
      | ^
/Users/amin/Workspace/cplusplus/bachelor-task/src/heuristic.cpp: In member function 'double Heuristic::computeHeuristic(const CellLocation&, const CellLocation&, const Heuristic::HeuristicModel&, double)':
/Users/amin/Workspace/cplusplus/bachelor-task/src/heuristic.cpp:46:1: warning: control reaches end of non-void function [-Wreturn-type]
   46 | }
      | ^
[ 85%] Linking CXX shared library libsource.dylib
[ 85%] Built target source
[ 92%] Building CXX object src/CMakeFiles/bachelor.dir/main.cpp.o
[100%] Linking CXX executable ../../bin/release/bachelor
[100%] Built target bachelor
```