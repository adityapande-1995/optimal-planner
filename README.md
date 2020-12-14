# LQR-RRT planner for raceline optimization
This project aims to find the optimal racing line for a omni-drive four-wheeled robot using the LQR-RRT planner. Depending on the track to follow and the goal to reach, the geometrically shortest path between the robot and goal might not be the fastest, when dynamics are taken into account. THis project will try to find the fastest racing lines for some set of predefined tracks (workspaces). The demo video of the best performaces can be found [here](https://github.com/adityapande-1995/optimal-planner/blob/main/animations/best-runs.mp4). The last two simulations apply constraints on controls (tries to keep control inputs as low as possible), whereas the rest on states (forces faster convergence)

![Animation for best runs](https://github.com/adityapande-1995/optimal-planner/blob/main/animations/best-runs-gif.gif)

# Requirements

benchmark.sh requires the command line utilities :
- silversearcher-ag
- convert, display
- sed, awk

The main python script requires the libraries:
- numpy, matplotlib, python-control, sympy (for AB.py file, derives A and B matrices)

# File descriptions
- main.py and helper-functions.py : Contain main code to run LQR-RRT algorithm. To change workspace and Q,R matrices, modify the line no. 29 in main.py 
```a = LQR_RRT_planner(Qmat = np.diag([1,1,1,0.1,0.1,0.1])*0.5 , Rmat = np.diag([1,1,1,1])*1.0 , time_limit = 1.5, workspace=1)``` to the required parameters.

- run.sh : Use this to run a planning and simulation step. Usage is described in the section below
- benchmark.sh : Do multiple runs of run.sh, sort and group them by performance and place in ```benchmarks/``` directory.
- best.sh : Replay best runs for each workspace
- AB.py : Linearize the system dynamics, find A and B matrices to be used later in helper_functions.py

# Usage
- To run the program once, execute : 
```cd planner && ./run.sh```. This will create a ```current_run``` directory in ```planner``` which will contain snapshots of the simulation, the states, and time series plot.
The playback method mentioned below can be used to playback the simulation from the ```current_run``` directory.

- To run benchmark with say 10 runs and the desired of the folder to be named "test_one":
```cd planner && ./benchmark.sh 10 test_one```

The results will be stored in ```benchmarks/test_one``` directory. A ```sorted.txt``` file will be created in the same, which will rank the simulation runs according to the time they took to converge, and a collage of top 3 simulations will be created as ```top_3.png```.

- To playback the best performances (stored in best.sh) :
```cd planner && ./best.sh```

- To replay the animation from say ```benchmarks/bench2/RUN_24```, run:
```cd planner && python3 playback.py ../benchmarks/bench2/RUN_24```


