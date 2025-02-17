from urchin import URDF
import numpy as np
robot = URDF.load('scara.urdf')
print("About to animate")
robot.animate(cfg_trajectory={
    'base_link_to_shoulder' : [-np.pi / 4, np.pi / 4],
    'shoulder_to_elbow' : [0.0, -np.pi / 2.0],
    'elbow_to_endZ' : [0.0, 0],
    'endZ_to_endR' : [0.0, np.pi / 2.0]
})
print("Animation complete")