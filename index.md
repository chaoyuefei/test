### Position-Based Servoing Example with Two Dynamic Obstacles
```python
#!/usr/bin/env python
"""
@author Jesse Haviland
"""
import swift
import spatialgeometry as sg
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import qpsolvers as qp
import cProfile

# Launch the simulator Swift
env = rtb.backends.Swift()
env = swift.Swift()
env.launch()

# Create a Panda robot object
@@ -75,25 +83,16 @@ panda.q = panda.qr
n = 7

# Make two obstacles with velocities
s0 = rtb.Sphere(
    radius=0.05,
    base=sm.SE3(0.52, 0.4, 0.3)
)
s0 = sg.Sphere(radius=0.05, base=sm.SE3(0.52, 0.4, 0.3))
s0.v = [0, -0.2, 0, 0, 0, 0]

s1 = rtb.Sphere(
    radius=0.05,
    base=sm.SE3(0.1, 0.35, 0.65)
)
s1 = sg.Sphere(radius=0.05, base=sm.SE3(0.1, 0.35, 0.65))
s1.v = [0, -0.2, 0, 0, 0, 0]

collisions = [s0, s1]

# Make a target
target = rtb.Sphere(
    radius=0.02,
    base=sm.SE3(0.6, -0.2, 0.0)
)
target = sg.Sphere(radius=0.02, base=sm.SE3(0.6, -0.2, 0.0))

# Add the Panda and shapes to the simulator
env.add(panda)
@@ -104,24 +103,22 @@ env.add(target)
# Set the desired end-effector pose to the location of target
Tep = panda.fkine(panda.q)
Tep.A[:3, 3] = target.base.t
Tep.A[2, 3] += 0.1
# Tep.A[2, 3] += 0.1

arrived = False
while not arrived:

def step():
    # The pose of the Panda's end-effector
    Te = panda.fkine(panda.q)

    # Transform from the end-effector to desired pose
    eTep = Te.inv() * Tep

    # Spatial error
    e = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi/180]))
    e = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi / 180]))

    # Calulate the required end-effector spatial velocity for the robot
    # to approach the goal. Gain is set to 1.0
    v, arrived = rtb.p_servo(Te, Tep, 0.5, 0.05)
    v, arrived = rtb.p_servo(Te, Tep, 0.5, 0.01)

    # Gain term (lambda) for control minimisation
    Y = 0.01
@@ -160,9 +157,14 @@ while not arrived:
        # Form the velocity damper inequality contraint for each collision
        # object on the robot to the collision in the scene
        c_Ain, c_bin = panda.link_collision_damper(
            collision, panda.q[:n], 0.3, 0.05, 1.0,
            startlink=panda.link_dict['panda_link1'],
            endlink=panda.link_dict['panda_hand'])
            collision,
            panda.q[:n],
            0.3,
            0.05,
            1.0,
            start=panda.link_dict["panda_link1"],
            end=panda.link_dict["panda_hand"],
        )

        # If there are any parts of the robot within the influence distance
        # to the collision in the scene
@@ -187,7 +189,15 @@ while not arrived:
    panda.qd[:n] = qd[:n]

    # Step the simulator by 50 ms
    env.step(0.05)
    env.step(0.01)
    return arrived
arrived = False
while not arrived:
    arrived = step()
```

* * *
