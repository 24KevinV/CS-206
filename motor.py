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

    def Set_Value(self, desiredAngle, robotId):
        pyrosim.Set_Motor_For_Joint(bodyIndex=robotId,
                                    jointName=self.jointName,
                                    controlMode=p.POSITION_CONTROL,
                                    # targetPosition=pi * (random.random() - 0.5),
                                    targetPosition=desiredAngle,
                                    maxForce=c.BackLeg_motorForce)

