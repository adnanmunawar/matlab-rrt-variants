# Matlab Implementation of RRT Variants
===================

# Author:
## Adnan Munawar (amunawar@wpi.edu)

RRT*, RRT-connect, lazy RRT and RRT extend have been implemented for 2d and 3d c-spaces with visualization

### General Information:

This is a basic yet meaningful implementation of RRT and its variants in Matlab.

## How to run
All you need to do is fire up the benchmarkRRT.m file, it is pretty self explanatory.

## Specify the number of runs for each planner
* num_of_runs =1;

## Specify if we want to run the specific planner or not, 1 for yes and 0 for no.
* run_RRTconnect =0 or 1; 
* run_RRTextend = 0 or 1;
* run_LazyRRT = 0 or 1;
* run_RRTstar = 0 or 1;

## Specify whether to run the planner in 2D or 3D (only for now)
* dim = 3;

## Specify the step size, the world is 100 \* 100 for 2D and 100 \* 100 \*100 for 3D 
* stepsize = [10];

## Specify whether to use random obstacles or to use pre programmed obstacles
* random_world = 0 or 1;

## For RRT* only
* radius = 10;
* samples = 4000;

## Showing output or not
* show_output = 0 or 1;
* show_benchmark_results = 0 or 1;
