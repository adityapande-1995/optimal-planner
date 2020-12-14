#!/bin/bash
echo "***** Running state_penalized best results (fastest convergence) *****"
for f in 1 2 3 4 5 6
do
	python3 playback.py ../benchmarks/states_penalised/$f
done

echo ""
echo "**** Running control penalized fastest paths ****"
for f in 1 2
do
	python3 playback.py ../benchmarks/control_penalised/$f
done
 


