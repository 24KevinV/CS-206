import constants as c
import copy
import solution


class HILL_CLIMBER:
    def __init__(self):
        self.parent = solution.SOLUTION()
        self.child = None

    def Evolve(self):
        self.parent.Evaluate("DIRECT")
        for currentGeneration in range(c.numberOfGenerations):
            self.Spawn()
            self.Mutate()
            if currentGeneration == 0 or currentGeneration == c.numberOfGenerations - 1:
                self.parent.Evaluate("GUI")
            else:
                self.child.Evaluate("DIRECT")
            self.Print()
            self.Select()

    def Mutate(self):
        self.child.Mutate()

    def Print(self):
        print(self.parent.fitness, self.child.fitness)

    def Select(self):
        if self.parent.fitness > self.child.fitness:
            self.parent = self.child

    def Spawn(self):
        self.child = copy.deepcopy(self.parent)
