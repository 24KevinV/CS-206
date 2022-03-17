# import hillclimber
import os
import parallelHillClimber

phc = parallelHillClimber.PARALLEL_HILL_CLIMBER()
phc.Evolve()

# for i in range(5):
#     os.system("python generate.py")
#     os.system("python simulate.py")