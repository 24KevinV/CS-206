import constants as c
import pybullet as p
import pybullet_data
from robot import ROBOT
import time
from world import WORLD


class SIMULATION:
    def __init__(self, directOrGUI, solutionID):
        self.directOrGUI = directOrGUI
        if directOrGUI == "DIRECT":
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.setGravity(c.gravity[0], c.gravity[1], c.gravity[2])

        self.world = WORLD()
        self.robot = ROBOT(solutionID)

    def __del__(self):
        p.disconnect()

    def Get_Fitness(self):
        self.robot.Get_Fitness()

    def Run(self):
        for t in range(c.ITERATIONS):
            p.stepSimulation()
            self.robot.Sense(t)
            self.robot.Think()
            self.robot.Act()
            if self.directOrGUI == "GUI":
                time.sleep(c.sleep_time)
