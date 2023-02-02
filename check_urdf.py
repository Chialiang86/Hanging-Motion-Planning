import os
import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
import xml.etree.ElementTree as ET

urdf = 'models/hook/Hook_60/base.urdf'
assert os.path.exists(urdf), f'{urdf} not exists'

physics_client_id = p.connect(p.GUI)
p.resetDebugVisualizerCamera(
    cameraDistance=0.3,
    cameraYaw=120,
    cameraPitch=0,
    cameraTargetPosition=[0.0, 0.0, 0.0]
)

tree = ET.parse(urdf)
root = tree.getroot()
center = np.array(
    [
        float(i) for i in root[0].find(
            "inertial"
        ).find(
            "origin"
        ).attrib['xyz'].split(' ')
    ]
)

rot = [0, np.pi / 2, 0]
matrix = R.from_rotvec(rot).as_matrix()
quat = R.from_rotvec(rot).as_quat()
quat = list(quat)

offset = matrix @ center

obj_id = p.loadURDF(urdf, [0, 0, 0], quat)
pos, rot = p.getBasePositionAndOrientation(obj_id)
pose = list(pos) + list(rot)
print(pose)

obj_id = p.loadURDF(urdf, [0, 0, 0], quat)
p.resetBasePositionAndOrientation(obj_id, [0, 0, 0], quat)
pos, rot = p.getBasePositionAndOrientation(obj_id)
pose = list(pos) + list(rot)
print(pose)

pose = np.asarray(pose)
pose[:3] = pose[:3] + offset
print(list(pose))

while True:
    # key callback
    keys = p.getKeyboardEvents()            
    if ord('q') in keys and keys[ord('q')] & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN): 
        break
