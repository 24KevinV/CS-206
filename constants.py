import numpy as np

ITERATIONS = 1000
sleep_time = 1/180

BackLeg_amplitude = 2 * np.pi / 6
BackLeg_frequency = 10
BackLeg_phaseOffset = 0

FrontLeg_amplitude = np.pi / 4
FrontLeg_frequency = 10
FrontLeg_phaseOffset = 0

amplitude = np.pi / 4
frequency = 5
phaseOffset = 0

x = np.linspace(0, 2 * np.pi, ITERATIONS)
BackLeg_targetAngles = BackLeg_amplitude * np.sin(BackLeg_frequency * x + BackLeg_phaseOffset)
FrontLeg_targetAngles = FrontLeg_amplitude * np.sin(FrontLeg_frequency * x + FrontLeg_phaseOffset)

BackLeg_motorForce = 25
FrontLeg_motorForce = 25

gravity = [0, 0, -9.8]

numberOfGenerations = 10
populationChunk = 10
numChunks = 1
populationSize = populationChunk * numChunks


numSensorNeurons = 17
numMotorNeurons = 16

motorJointRange = 1
