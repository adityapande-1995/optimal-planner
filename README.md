# Requirements

benchmark.sh requires the command line utilities :
- silversearcher-ag
- convert, display
- sed, awk

The main python script requires the libraries:
- numpy, matplotlib, python-control

# Usage
- To run the program once, 
```cd planner && ./run.sh```

- To run benchmark with say 10 runs and the desired of the folder to be named "test_one":
```cd planner && ./benchmark.sh 10 test_one```

The results will be stored in ```benchmarks/``` directory

- To replay the animation from say ```benchmarks/bench2/RUN_24```, run:
```cd planner && python3 playback.py ../benchmarks/bench2/RUN_24```


