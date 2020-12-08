#!/bin/bash
echo "Usage : ./benchmark.sh 20 bench_dirname"
count=0

while [ $count -lt $1 ]
do
	echo Iteration $count 
	./run.sh
	mv current_run RUN_$count
	let count=count+1
done

# Sort simulations by time to reach the goal
echo "\n -- Sorting results based on time to reach goal : --"
ag -G'\.txt$' Time | sort -k2 -n > sorted.txt
cat sorted.txt
echo "-------------------------------------------------------"
echo "Merging and showing top 3 images...\n"

# Merge top 3 images
convert +append $(for i in 1 2 3 ;do printf $(sed -n $i'p' < sorted.txt | awk -F / '{print $1}')/simulation.png ; printf " " ;done) top_3.png 
display top_3.png

# Move everything to a folder in prev directory
mkdir ../benchmarks/$2
mv RUN_* sorted.txt top_3.png ../benchmarks/$2

 
