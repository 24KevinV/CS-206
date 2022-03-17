import constants as c
import copy
import os
import solution


class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("del brain*.nndf")
        os.system("del fitness*.txt")
        self.nextAvailableID = 0
        self.parents = {}
        for i in range(c.populationSize):
            self.parents[i] = solution.SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1
        self.children = None

    def Evaluate(self, solutions):
        for i in range(c.populationSize):
            solutions[i].Start_Simulation("DIRECT")
        for i in range(c.populationSize):
            solutions[i].Wait_For_Simulation_To_End()

    def Evolve(self):
        self.Evaluate(self.parents)
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation(currentGeneration)
        self.Show_Best()

    def Evolve_For_One_Generation(self, currentGeneration):
        self.Spawn()
        self.Mutate()
        # if currentGeneration == 0 or currentGeneration == c.numberOfGenerations - 1:
        #     self.parent.Evaluate("GUI")
        # else:
        self.Evaluate(self.children)
        self.Print()
        self.Select()

    def Mutate(self):
        for i in range(c.populationSize):
            self.children[i].Mutate()

    def Print(self):
        print()
        for i in range(c.populationSize):
            print(self.parents[i].fitness, self.children[i].fitness)

    def Select(self):
        for i in range(c.populationSize):
            if self.parents[i].fitness > self.children[i].fitness:
                self.parents[i] = self.children[i]

    def Show_Best(self):
        best_solution = 0
        for i in range(c.populationSize):
            if self.parents[i].fitness < self.parents[best_solution].fitness:
                best_solution = i
        self.parents[best_solution].Start_Simulation("GUI")
        print("Best Fitness:", self.parents[best_solution].fitness)

    def Spawn(self):
        self.children = {}
        for i in range(c.populationSize):
            self.children[i] = copy.deepcopy(self.parents[i])
            self.children[i].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1
