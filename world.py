import pybullet as p
import time


class WORLD:
    def __init__(self):
        self.planeId = p.loadURDF("plane.urdf")
        while True:
            try:
                p.loadSDF("world.sdf")
                break
            except Exception:
                time.sleep(0.01)
