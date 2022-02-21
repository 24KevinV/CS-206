import numpy as np
import matplotlib.pyplot

backLegSensorValues = np.load("data/backLegSensorValues.npy")
frontLegSensorValues = np.load("data/frontLegSensorValues.npy")
targetAngles = np.load("data/targetAngles.npy")
# print(backLegSensorValues)

# matplotlib.pyplot.plot(backLegSensorValues, label="Back Leg Sensor Values", linewidth=5)
# matplotlib.pyplot.plot(frontLegSensorValues, label="Front Leg Sensor Values")
matplotlib.pyplot.plot(targetAngles)
matplotlib.pyplot.legend()
matplotlib.pyplot.show()
