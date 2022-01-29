import pyrosim.pyrosim as pyroism

pyroism.Start_SDF("boxes.sdf")
length = 1
width = 1
height = 1
x = 0
y = 0
z = height / 2

# pyroism.Send_Cube(name="Box", pos=[x, y, z], size=[length, width, height])
# x = 1
# y = 0
# z = height + height / 2
# pyroism.Send_Cube(name="Box2", pos=[x, y, z], size=[length, width, height])
for i in range(5):
    for j in range(5):
        length = 1
        width = 1
        height = 1
        z = height / 2
        for k in range(10):
            pyroism.Send_Cube(name="Box", pos=[i, j, z], size=[length, width, height])
            z += (height + height * 0.9) / 2
            length *= 0.9
            width *= 0.9
            height *= 0.9

pyroism.End()
