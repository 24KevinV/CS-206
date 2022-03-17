import numpy as np
import os
import pyrosim.pyrosim as pyrosim
import random
import time

length = 1
width = 1
height = 1
x = 0
y = 0
z = height / 2


class SOLUTION:
    def __init__(self, myID):
        self.myID = myID
        self.weights = np.random.rand(3, 2)
        self.weights = self.weights * 2 - 1
        self.fitness = None
        # self.Evaluate()

    def Evaluate(self, directOrGUI):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()
        os.system("start /B python simulate.py {} {}".format(directOrGUI, self.myID))
        while not os.path.exists("fitness{}.txt".format(self.myID)):
            time.sleep(0.01)
        fitnessFile = open("fitness{}.txt".format(self.myID), "r")
        self.fitness = float(fitnessFile.read())
        fitnessFile.close()

    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()
        os.system("start /B python simulate.py {} {}".format(directOrGUI, self.myID))

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists("fitness{}.txt".format(self.myID)):
            time.sleep(0.01)
        fitnessFile = open("fitness{}.txt".format(self.myID), "r")
        self.fitness = float(fitnessFile.read())
        fitnessFile.close()
        os.system("del fitness{}.txt".format(self.myID))


    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")

        pyrosim.Send_Cube(name="Box", pos=[-3, 3, 0.5], size=[length, width, height])

        # x = 1
        # y = 0
        # z = height + height / 2
        # pyrosim.Send_Cube(name="Box2", pos=[x, y, z], size=[length, width, height])
        # for i in range(5):
        #     for j in range(5):
        #         length = 1
        #         width = 1
        #         height = 1
        #         z = height / 2
        #         for k in range(10):
        #             pyrosim.Send_Cube(name="Box", pos=[i, j, z], size=[length, width, height])
        #             z += (height + height * 0.9) / 2
        #             length *= 0.9
        #             width *= 0.9
        #             height *= 0.9

        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")
        pyrosim.Send_Cube(name="Torso", pos=[0, 0, 1.5], size=[length, width, height])
        pyrosim.Send_Joint(name="Torso_BackLeg", parent="Torso", child="BackLeg",
                           type="revolute", position=[-0.5, 0, 1])
        pyrosim.Send_Cube(name="BackLeg", pos=[-0.5, 0, -0.5], size=[length, width, height])
        pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso", child="FrontLeg",
                           type="revolute", position=[0.5, 0, 1])
        pyrosim.Send_Cube(name="FrontLeg", pos=[0.5, 0, -0.5], size=[length, width, height])
        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain{}.nndf".format(self.myID))
        pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
        pyrosim.Send_Sensor_Neuron(name=1, linkName="BackLeg")
        pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLeg")
        pyrosim.Send_Motor_Neuron(name=3, jointName="Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name=4, jointName="Torso_FrontLeg")

        for currentRow in range(3):
            for currentColumn in range(2):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn + 3,
                                     weight=self.weights[currentRow][currentColumn])
        pyrosim.End()

    def Mutate(self):
        randomRow = random.randint(0, 2)
        randomColumn = random.randint(0, 1)
        self.weights[randomRow, randomColumn] = random.random() * 2 - 1

    def Set_ID(self, ID):
        self.myID = ID
