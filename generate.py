import pyrosim.pyrosim as pyroism

pyroism.Start_SDF("box.sdf")
pyroism.Send_Cube(name="Box", pos=[0, 0, 0.5], size=[1, 1, 1])

pyroism.End()