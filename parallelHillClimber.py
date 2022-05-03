import constants as c
import copy
import os
import solution


class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("del brain*.nndf")
        os.system("del fitness*.txt")
        self.datafile = open("octopod fitness.txt", "a")
        self.datafile.write("Start of Run\n")
        self.nextAvailableID = 0
        self.parents = {}
        for i in range(c.populationSize):
            self.parents[i] = solution.SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1
        self.children = None

    def Evaluate(self, solutions):
        for i in range(c.populationSize // c.populationChunk):
            for j in range(c.populationChunk):
                solutions[i * c.populationChunk + j].Start_Simulation("DIRECT")
            for j in range(c.populationChunk):
                # print(str(i * c.populationChunk + j))
                solutions[i * c.populationChunk + j].Wait_For_Simulation_To_End()

        # for i in range(c.populationSize % c.populationChunk):
        #     solutions[i + c.populationSize // c.populationChunk].Start_Simulation("DIRECT")
        # for i in range(c.populationSize % c.populationChunk):
        #     print(str(i + (c.populationSize // c.populationChunk) * c.populationChunk))
        #     solutions[i + (c.populationSize // c.populationChunk) * c.populationChunk].Wait_For_Simulation_To_End()

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
        self.datafile.write(str(currentGeneration) + "\n")
        self.Print()
        self.Select()
        print(currentGeneration)

    def Mutate(self):
        for i in range(c.populationSize):
            self.children[i].Mutate()

    def Print(self):
        print()
        for i in range(c.populationSize):
            print(self.parents[i].fitness, self.children[i].fitness)
            self.datafile.write(str(self.parents[i].fitness) + "\n")

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
        self.datafile.close()

    def Spawn(self):
        self.children = {}
        for i in range(c.populationSize):
            self.children[i] = copy.deepcopy(self.parents[i])
            self.children[i].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1
