# from math import pi
import numpy as np
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import random
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")

p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)

ITERATIONS = 1000

BackLeg_amplitude = 2 * np.pi / 6
BackLeg_frequency = 10
BackLeg_phaseOffset = 0

FrontLeg_amplitude = np.pi / 4
FrontLeg_frequency = 10
FrontLeg_phaseOffset = 0

x = np.linspace(0, 2 * np.pi, ITERATIONS)
BackLeg_targetAngles = BackLeg_amplitude * np.sin(BackLeg_frequency * x + BackLeg_phaseOffset)
FrontLeg_targetAngles = FrontLeg_amplitude * np.sin(FrontLeg_frequency * x + FrontLeg_phaseOffset)
# np.save("data/targetAngles.npy", targetAngles)
# exit()

backLegSensorValues = np.zeros(ITERATIONS)
frontLegSensorValues = np.zeros(ITERATIONS)

for i in range(ITERATIONS):
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    pyrosim.Set_Motor_For_Joint(bodyIndex=robotId,
                                jointName="Torso_BackLeg",
                                controlMode=p.POSITION_CONTROL,
                                # targetPosition=pi * (random.random() - 0.5),
                                targetPosition=BackLeg_targetAngles[i],
                                maxForce=25)

    pyrosim.Set_Motor_For_Joint(bodyIndex=robotId,
                                jointName="Torso_FrontLeg",
                                controlMode=p.POSITION_CONTROL,
                                # targetPosition=pi * (random.random() - 0.5),
                                targetPosition=FrontLeg_targetAngles[i],
                                maxForce=25)

    time.sleep(1/60)

p.disconnect()

# print(backLegSensorValues)

np.save("data/backLegSensorValues.npy", backLegSensorValues)
np.save("data/frontLegSensorValues.npy", frontLegSensorValues)
