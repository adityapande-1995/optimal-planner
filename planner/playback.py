#!python3
from helper_functions import *
import pickle
import os, sys

os.chdir(sys.argv[1])

trajectory = pickle.load( open( "traj.p", "rb" ) )
path_rects, path2 = pickle.load( open( "otherdata.p", "rb" ) )
animate(trajectory[0],trajectory[1],path_rects,path2)


