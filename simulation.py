import constants as c
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
from robot import ROBOT
import time
from world import WORLD


class SIMULATION:
    def __init__(self):
        # self.world = WORLD()
        # self.robot = ROBOT()
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.setGravity(c.gravity[0], c.gravity[1], c.gravity[2])
        # planeId = p.loadURDF("plane.urdf")
        # robotId = p.loadURDF("body.urdf")
        #
        # p.loadSDF("world.sdf")

        # pyrosim.Prepare_To_Simulate(robotId)

        self.world = WORLD()
        self.robot = ROBOT()

    def __del__(self):
        p.disconnect()

    def Run(self):
        for t in range(c.ITERATIONS):
            p.stepSimulation()
            self.robot.Sense(t)
            self.robot.Act(t)
            # pyrosim.Set_Motor_For_Joint(bodyIndex=robotId,
            #                             jointName="Torso_BackLeg",
            #                             controlMode=p.POSITION_CONTROL,
            #                             # targetPosition=pi * (random.random() - 0.5),
            #                             targetPosition=c.BackLeg_targetAngles[i],
            #                             maxForce=c.BackLeg_motorForce)
            #
            # pyrosim.Set_Motor_For_Joint(bodyIndex=robotId,
            #                             jointName="Torso_FrontLeg",
            #                             controlMode=p.POSITION_CONTROL,
            #                             # targetPosition=pi * (random.random() - 0.5),
            #                             targetPosition=c.FrontLeg_targetAngles[i],
            #                             maxForce=c.FrontLeg_motorForce)

            time.sleep(c.sleep_time)