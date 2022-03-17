# import constants as c
# import numpy as np
import os

from motor import MOTOR
import pybullet as p
from pyrosim.neuralNetwork import NEURAL_NETWORK
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR


class ROBOT:
    def __init__(self, solutionID):
        self.sensors = None
        self.nn = NEURAL_NETWORK("brain{}.nndf".format(solutionID))
        os.system("del brain{}.nndf".format(solutionID))
        self.motors = None
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        self.solutionID = solutionID

    def Act(self):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                self.motors[jointName].Set_Value(desiredAngle, self.robotId)

    def Get_Fitness(self):
        stateOfLinkZero = p.getLinkState(self.robotId, 0)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]
        f = open("tmp{}.txt".format(self.solutionID), "w")
        f.write(str(xCoordinateOfLinkZero))
        f.close()
        os.rename("tmp{}.txt".format(self.solutionID),  "fitness{}.txt".format(self.solutionID))

    def Prepare_To_Act(self):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)

    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t):
        for s in self.sensors:
            self.sensors[s].Get_Value(t)

    def Think(self):
        # self.nn.Print()
        self.nn.Update()


