# import constants as c
# import numpy as np
from motor import MOTOR
import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR


class ROBOT:
    def __init__(self):
        self.sensors = None
        self.motors = None
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()

    def Act(self, t):
        for m in self.motors:
            self.motors[m].Set_Value(t, self.robotId)

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


