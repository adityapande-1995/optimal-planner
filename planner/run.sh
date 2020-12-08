#!/bin/sh
rm -f log.txt ; python3 main.py > log.txt
rm -rf current_run
mkdir current_run
mv *.png *.p *.txt current_run

