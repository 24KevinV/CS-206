# import hillclimber
import os
import parallelHillClimber

for i in range(10):
    print("current run number: {}".format(i))
    phc = parallelHillClimber.PARALLEL_HILL_CLIMBER()
    phc.Evolve()