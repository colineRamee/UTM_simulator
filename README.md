# UTM Simulator
## Description
This is the code I developed for my PhD thesis.
The code is distributed under a MIT license. Please cite my thesis if using this code: Ramee, C. (2021). 
*Conceptual-Level Analysis and Design of Unmanned Air Traffic Management Systems* [Doctoral Dissertation, Georgia Institute of Technology].
### UTM Simulator
The code for the simulation itself is included in the utm_simulator folder. It is packaged as a library.

### Scripts
This folder contains some of the functions I wrote to analyze the simulation results.

### Cluster
This folder contains some example to generate and run cases in batch.

### Data
This folder contains the data files used to define demand and static obstacles.

### Logs
This folder contains the json log file. The example subfolder contains the data from the first two experiments presented in the thesis.

## Installation
First clone the code.

`git clone https://github.com/colineRamee/UTM_simulator`

Navigate to the new folder.
### Install with conda
To install the code dependencies in a new conda virtual environment:

`conda env create - f utm_simulator.yml`

### Install with pip
When installing with pip, the gurobi library is called gurobipy. 
The setup.py file can be used to install dependencies by running:

`python setup.py install`

Note that to run simulations that use the LocalVO method for preflight planning, an active gurobi license is required. 
If you don't have a license all the other methods will still work.

## Running the code
The file main_layered.py provides an example of how to run one simulation with multiple altitude levels.
The file main_2d.py provides an example of how to run one simulation with a single altitude level. The main 
difference between the two from a user perspective is the display. To see an example of how to run the simulation 
headless, you can check the file cluster/run_cases_locally.py. There is no need to create an update queue and a display.
A text file that logs the simulation debug messages is created at the start, if the simulation ends cleanly that text file is removed.
Depending on the simulation type, agent density, and computational power, 
runtime can vary greatly. The visualization will crash if the simulation runs 
too slowly (it crashes when it does not get updated frequently enough). It is advised to turn it off when running more than 50 agents. 
The results are saved to a JSON file containing a summary of the run settings, 
and metrics for each agent in the simulation. To see examples of how to process the log files, 
look at scripts/example_analysis.py.


