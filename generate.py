import pyrosim.pyrosim as pyroism

length = 1
width = 1
height = 1
x = 0
y = 0
z = height / 2


def Create_World():
    pyroism.Start_SDF("world.sdf")

    pyroism.Send_Cube(name="Box", pos=[-3, 3, 0.5], size=[length, width, height])

    # x = 1
    # y = 0
    # z = height + height / 2
    # pyroism.Send_Cube(name="Box2", pos=[x, y, z], size=[length, width, height])
    # for i in range(5):
    #     for j in range(5):
    #         length = 1
    #         width = 1
    #         height = 1
    #         z = height / 2
    #         for k in range(10):
    #             pyroism.Send_Cube(name="Box", pos=[i, j, z], size=[length, width, height])
    #             z += (height + height * 0.9) / 2
    #             length *= 0.9
    #             width *= 0.9
    #             height *= 0.9

    pyroism.End()


def Create_Robot():
    pyroism.Start_URDF("body.urdf")
    pyroism.Send_Cube(name="Torso", pos=[0, 0, 1.5], size=[length, width, height])
    pyroism.Send_Joint(name="Torso_BackLeg", parent="Torso", child="BackLeg", type="revolute", position=[-0.5, 0, 1])
    pyroism.Send_Cube(name="BackLeg", pos=[-0.5, 0, -0.5], size=[length, width, height])
    pyroism.Send_Joint(name="Torso_FrontLeg", parent="Torso", child="FrontLeg", type="revolute", position=[0.5, 0, 1])
    pyroism.Send_Cube(name="FrontLeg", pos=[0.5, 0, -0.5], size=[length, width, height])
    pyroism.End()


Create_World()
Create_Robot()