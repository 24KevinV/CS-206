import constants as c
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
        self.weights = np.random.rand(c.numSensorNeurons, c.numMotorNeurons)
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
        while True:
            try:
                fitnessFile = open("fitness{}.txt".format(self.myID), "r")
                self.fitness = float(fitnessFile.read())
                fitnessFile.close()
                os.system("del fitness{}.txt".format(self.myID))
                break
            except Exception:
                print("waiting for fitness{}.txt".format(self.myID))
                time.sleep(0.01)


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
        # Upper Legs
        pyrosim.Send_Cube(name="Torso", pos=[0, 0, 1], size=[length, width, height])
        pyrosim.Send_Joint(name="Torso_BackLeg", parent="Torso", child="BackLeg",
                           type="revolute", position=[0, -0.5, 1], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="BackLeg", pos=[0, -0.5, 0], size=[0.2, 1, 0.2])
        pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso", child="FrontLeg",
                           type="revolute", position=[0, 0.5, 1], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="FrontLeg", pos=[0, 0.5, 0], size=[0.2, 1, 0.2])
        pyrosim.Send_Joint(name="Torso_LeftLeg", parent="Torso", child="LeftLeg",
                           type="revolute", position=[-0.5, 0, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5, 0, 0], size=[1, 0.2, 0.2])
        pyrosim.Send_Joint(name="Torso_RightLeg", parent="Torso", child="RightLeg",
                           type="revolute", position=[0.5, 0, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="RightLeg", pos=[0.5, 0, 0], size=[1, 0.2, 0.2])

        # Lower Legs
        pyrosim.Send_Joint(name="BackLeg_LowerBackLeg", parent="BackLeg", child="LowerBackLeg",
                           type="revolute", position=[0, -1, 0], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="LowerBackLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])
        pyrosim.Send_Joint(name="FrontLeg_LowerFrontLeg", parent="FrontLeg", child="LowerFrontLeg",
                           type="revolute", position=[0, 1, 0], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="LowerFrontLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])

        pyrosim.Send_Joint(name="LeftLeg_LowerLeftLeg", parent="LeftLeg", child="LowerLeftLeg",
                           type="revolute", position=[-1, 0, 0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LowerLeftLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])
        pyrosim.Send_Joint(name="RightLeg_LowerRightLeg", parent="RightLeg", child="LowerRightLeg",
                           type="revolute", position=[1, 0, 0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LowerRightLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])

        # Corner Legs
        pyrosim.Send_Joint(name="Torso_BLLeg", parent="Torso", child="BLLeg",
                           type="revolute", position=[-0.5, 0, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="BLLeg", pos=[-0.5, -0.35, 0], size=[1, 0.2, 0.2])
        pyrosim.Send_Joint(name="Torso_BRLeg", parent="Torso", child="BRLeg",
                           type="revolute", position=[-0.5, 0, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="BRLeg", pos=[-0.5, 0.35, 0], size=[1, 0.2, 0.2])
        pyrosim.Send_Joint(name="Torso_FLLeg", parent="Torso", child="FLLeg",
                           type="revolute", position=[0.5, 0, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="FLLeg", pos=[0.5, -0.35, 0], size=[1, 0.2, 0.2])
        pyrosim.Send_Joint(name="Torso_FRLeg", parent="Torso", child="FRLeg",
                           type="revolute", position=[0.5, 0, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="FRLeg", pos=[0.5, 0.35, 0], size=[1, 0.2, 0.2])

        # Lower Corner Legs
        pyrosim.Send_Joint(name="BLLeg_LowerBLLeg", parent="BLLeg", child="LowerBLLeg",
                           type="revolute", position=[-1, -0.35, 0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LowerBLLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])
        pyrosim.Send_Joint(name="BRLeg_LowerBRLeg", parent="BRLeg", child="LowerBRLeg",
                           type="revolute", position=[-1, 0.35, 0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LowerBRLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])
        pyrosim.Send_Joint(name="FLLeg_LowerFLLeg", parent="FLLeg", child="LowerFLLeg",
                           type="revolute", position=[1, -0.35, 0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LowerFLLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])
        pyrosim.Send_Joint(name="FRLeg_LowerFRLeg", parent="FRLeg", child="LowerFRLeg",
                           type="revolute", position=[1, 0.35, 0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LowerFRLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])

        pyrosim.End()


    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain{}.nndf".format(self.myID))
        pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
        pyrosim.Send_Sensor_Neuron(name=1, linkName="BackLeg")
        pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLeg")
        pyrosim.Send_Sensor_Neuron(name=3, linkName="LeftLeg")
        pyrosim.Send_Sensor_Neuron(name=4, linkName="RightLeg")
        pyrosim.Send_Sensor_Neuron(name=5, linkName="LowerBackLeg")
        pyrosim.Send_Sensor_Neuron(name=6, linkName="LowerFrontLeg")
        pyrosim.Send_Sensor_Neuron(name=7, linkName="LowerLeftLeg")
        pyrosim.Send_Sensor_Neuron(name=8, linkName="LowerRightLeg")

        pyrosim.Send_Sensor_Neuron(name=9, linkName="BLLeg")
        pyrosim.Send_Sensor_Neuron(name=10, linkName="BRLeg")
        pyrosim.Send_Sensor_Neuron(name=11, linkName="FLLeg")
        pyrosim.Send_Sensor_Neuron(name=12, linkName="FRLeg")
        pyrosim.Send_Sensor_Neuron(name=13, linkName="LowerBLLeg")
        pyrosim.Send_Sensor_Neuron(name=14, linkName="LowerBRLeg")
        pyrosim.Send_Sensor_Neuron(name=15, linkName="LowerFLLeg")
        pyrosim.Send_Sensor_Neuron(name=16, linkName="LowerFRLeg")

        pyrosim.Send_Motor_Neuron(name=17, jointName="Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name=18, jointName="Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron(name=19, jointName="Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron(name=20, jointName="Torso_RightLeg")
        pyrosim.Send_Motor_Neuron(name=21, jointName="BackLeg_LowerBackLeg")
        pyrosim.Send_Motor_Neuron(name=22, jointName="FrontLeg_LowerFrontLeg")
        pyrosim.Send_Motor_Neuron(name=23, jointName="LeftLeg_LowerLeftLeg")
        pyrosim.Send_Motor_Neuron(name=24, jointName="RightLeg_LowerRightLeg")

        pyrosim.Send_Motor_Neuron(name=25, jointName="Torso_BLLeg")
        pyrosim.Send_Motor_Neuron(name=26, jointName="Torso_BRLeg")
        pyrosim.Send_Motor_Neuron(name=27, jointName="Torso_FLLeg")
        pyrosim.Send_Motor_Neuron(name=28, jointName="Torso_FRLeg")
        pyrosim.Send_Motor_Neuron(name=29, jointName="BLLeg_LowerBLLeg")
        pyrosim.Send_Motor_Neuron(name=30, jointName="BRLeg_LowerBRLeg")
        pyrosim.Send_Motor_Neuron(name=31, jointName="FLLeg_LowerFLLeg")
        pyrosim.Send_Motor_Neuron(name=32, jointName="FRLeg_LowerFRLeg")

        for currentRow in range(c.numSensorNeurons):
            for currentColumn in range(c.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn + c.numSensorNeurons,
                                     weight=self.weights[currentRow][currentColumn])
        pyrosim.End()

    def Mutate(self):
        randomRow = random.randint(0, c.numSensorNeurons - 1)
        randomColumn = random.randint(0, c.numMotorNeurons - 1)
        self.weights[randomRow, randomColumn] = random.random() * 2 - 1

    def Set_ID(self, ID):
        self.myID = ID
