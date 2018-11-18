# Kidnapped Vehicle Project
Self-Driving Car Engineer Nanodegree Program

In this project implemented a 2 dimensional particle filter which was given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data. The robot has been kidnapped and transported to a new location! Using it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

## Basic Build Instructions

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

## Input & Output

INPUT: 
	// sense noisy position data from the simulator
	["sense_x"]
	["sense_y"]
	["sense_theta"]

	//get the previous velocity and yaw rate to predict the particle's transitioned state
	["previous_velocity"]
	["previous_yawrate"]

	// receive noisy observation data from the simulator, in a respective list of x/y values
	["sense_observations_x"]
	["sense_observations_y"]

OUTPUT: 
	// best particle values used for calculating the error evaluation
	["best_particle_x"]
	["best_particle_y"]
	["best_particle_theta"]

	// for respective (x,y) sensed positions ID label
	["best_particle_associations"]

	// for respective (x,y) sensed positions
	["best_particle_sense_x"] <= list of sensed x positions
	["best_particle_sense_y"] <= list of sensed y positions

## Implementing the Particle Filter
The directory structure of this repository is as follows:
```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```



## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

## Success Criteria
1. **Accuracy**: your particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: your particle filter should complete execution within the time of 100 seconds.

