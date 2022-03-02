import random

import pyrosim.pyrosim as pyrosim

length = 1
width = 1
height = 1
x = 0
y = 0
z = height / 2


def Create_World():
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


def Generate_Body():
    pyrosim.Start_URDF("body.urdf")
    pyrosim.Send_Cube(name="Torso", pos=[0, 0, 1.5], size=[length, width, height])
    pyrosim.Send_Joint(name="Torso_BackLeg", parent="Torso", child="BackLeg", type="revolute", position=[-0.5, 0, 1])
    pyrosim.Send_Cube(name="BackLeg", pos=[-0.5, 0, -0.5], size=[length, width, height])
    pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso", child="FrontLeg", type="revolute", position=[0.5, 0, 1])
    pyrosim.Send_Cube(name="FrontLeg", pos=[0.5, 0, -0.5], size=[length, width, height])
    pyrosim.End()


def Generate_Brain():
    pyrosim.Start_NeuralNetwork("brain.nndf")
    pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
    pyrosim.Send_Sensor_Neuron(name=1, linkName="BackLeg")
    pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLeg")
    pyrosim.Send_Motor_Neuron(name=3, jointName="Torso_BackLeg")
    pyrosim.Send_Motor_Neuron(name=4, jointName="Torso_FrontLeg")

    # pyrosim.Send_Synapse(sourceNeuronName=1, targetNeuronName=3, weight=-0.5)
    # pyrosim.Send_Synapse(sourceNeuronName=2, targetNeuronName=3, weight=-0.5)
    # pyrosim.Send_Synapse(sourceNeuronName=0, targetNeuronName=4, weight=1.0)
    # pyrosim.Send_Synapse(sourceNeuronName=2, targetNeuronName=4, weight=1.0)

    # pyrosim.Send_Synapse(sourceNeuronName=0, targetNeuronName=3, weight=-0.5)
    # pyrosim.Send_Synapse(sourceNeuronName=1, targetNeuronName=3, weight=-0.5)
    # pyrosim.Send_Synapse(sourceNeuronName=0, targetNeuronName=4, weight=-1.0)
    # pyrosim.Send_Synapse(sourceNeuronName=2, targetNeuronName=4, weight=-1.0)

    for i in range(0,3):
        for j in range(3,5):
            pyrosim.Send_Synapse(sourceNeuronName=i, targetNeuronName=j, weight=random.uniform(-1.0, 1.0))

    pyrosim.End()


Create_World()
Generate_Body()
Generate_Brain()
