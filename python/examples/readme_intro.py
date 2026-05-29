from spherical_wrist import KinematicModel, Robot
from scipy.spatial.transform import RigidTransform, Rotation
import numpy as np


kinematic_model = KinematicModel(
    a1=400,
    a2=-250,
    b=0,
    c1=830,
    c2=1175,
    c3=1444,
    c4=230,
    offsets=(0, 0, 0, 0, 0, 0),
    flip_axes=(True, False, True, True, False, True),
)

robot = Robot(kinematic_model, degrees=True)

ee_rotation = Rotation.from_euler("xyz", [0, -90, 0], degrees=True)
ee_transform = RigidTransform.from_components(
    rotation=ee_rotation,
    translation=[0, 0, 0],
)

joints = (10, 0, -90, 0, 0, 0)
pose = robot.forward(joints, ee_transform=ee_transform)

print(f"Position: {np.round(pose.translation, 2)}")
print(f"Rotation (XYZ Euler): {np.round(pose.rotation.as_euler('XYZ', degrees=True), 2)}")

solutions = robot.inverse(pose, ee_transform=ee_transform)
print(f"Found {len(solutions)} IK solutions")
for solution in solutions:
    print(f"  {np.round(solution, 2)}")

