import constants as c
import numpy as np
import pybullet as p
import pyrosim.pyrosim as pyrosim


class MOTOR:
    def __init__(self, jointName):
        self.offset = None
        self.frequency = None
        self.amplitude = None
        self.motorValues = None
        self.jointName = jointName
        self.Prepare_To_Act()

    def Prepare_To_Act(self):
        self.amplitude = c.amplitude
        self.frequency = c.frequency
        if self.jointName == "Torso_BackLeg":
            self.frequency /= 2
        self.offset = c.phaseOffset
        self.motorValues = self.amplitude * np.sin(self.frequency * c.x + self.offset)

    def Set_Value(self, t, robotId):
        pyrosim.Set_Motor_For_Joint(bodyIndex=robotId,
                                    jointName=self.jointName,
                                    controlMode=p.POSITION_CONTROL,
                                    # targetPosition=pi * (random.random() - 0.5),
                                    targetPosition=self.motorValues[t],
                                    maxForce=c.BackLeg_motorForce)

    def Save_Values(self):
        np.save("data/motorValues.npy", self.motorValues)
