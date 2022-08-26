from sys import flags
import pybullet as p
import pybullet_robot.panda.panda_sim as panda_sim
from math import pi
import time
import pybullet_data
import numpy as np
from tqdm import tqdm
import os
from utils.robot import Robot
import xml.etree.ElementTree as ET

def reset_pose(obj_id, x_offset=0.1, y_offset=0., z_offset=1.):
    x = (np.random.rand() - 0.5) * 0.1 + x_offset
    y = (np.random.rand() - 0.5) * 0.4 + y_offset
    z = (np.random.rand() - 0.5) * 0.4 + z_offset

    roll = np.random.rand() * pi * 2
    pitch = np.random.rand() * pi * 2
    yaw = np.random.rand() * pi * 2
    p.setGravity(0, 0, 0)
    p.resetBasePositionAndOrientation(
        obj_id,
        [x, y, z],
        p.getQuaternionFromEuler([roll, pitch, yaw]))

def load_hook(obj_file, pos=[0,0,0.4], rot=[np.pi/2, 0, np.pi/2], shift=[0, -0.02, 0], scale=[1, 1, 1]):
    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName=obj_file,
        rgbaColor=[1, 1, 1, 1],
        specularColor=[0.4, 0.4, 0],
        visualFramePosition=shift,
        meshScale=scale,
        flags=p.GEOM_FORCE_CONCAVE_TRIMESH
    )
    collision_shape_id = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName=obj_file,
        collisionFramePosition=shift,
        meshScale=scale,
        flags=p.GEOM_FORCE_CONCAVE_TRIMESH
    )
    hook_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=[0, 0, 0],
        useMaximalCoordinates=True
    )
    hook_pos = pos
    hook_orientation = p.getQuaternionFromEuler(rot)
    p.resetBasePositionAndOrientation(hook_id, hook_pos, hook_orientation)

    return hook_id

def main():

    # init p
    gravity = -9.8
    timestep = 1000.
    physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
    p.resetDebugVisualizerCamera(
        cameraDistance=0.5,
        cameraYaw=10,
        cameraPitch=0,
        cameraTargetPosition=[0.05, 0, 0.3])
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setTimeStep(1 / timestep)
    p.setGravity(0,0, gravity)

    # init hook and env
    plane_id = p.loadURDF("plane.urdf")
    # wall_id = p.loadURDF("models/wall/wall.urdf", [0, 0, 0.7], [0, 0, 0, 1])
    # hook_position = [0,0,0.3]
    # hook_orientation = p.getQuaternionFromEuler([np.pi/2, 0, np.pi/2])
    # hook_id = p.loadURDF("models/geo_data/hook_wall/23/base.urdf", hook_position, hook_orientation)
    hook_id = load_hook("models/geo_data/hook_wall/23/base.obj", 
                pos=[0, 0, 0.3], 
                rot=[0, 0, np.pi], 
                shift=[0, 0, 0], 
                scale=[0.3, 0.3, 0.3])

    # init object
    urdf_file = 'models/hanging_eval/025_mug/base.urdf'
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    center = np.array([float(i) for i in root[0].find(
        "inertial").find("origin").attrib['xyz'].split(' ')])
    obj_pose = [0.1, 0, 0.5]
    obj_orientation = p.getQuaternionFromEuler([0, 0, 0])
    obj_id = p.loadURDF(urdf_file, obj_pose, obj_orientation)

    # init robot
    # robot = Robot(p, start_pos=[0.8, 0, 0] ,urdf_path="models/robot/")

    panda = panda_sim.PandaSim(p,[0,0,0])

    #set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
    
    height_thresh = 0.2
    for i in tqdm(range(10000)):

        # simulation initialization 
        panda.step()
        p.setGravity(0, 0, 0)
        reset_pose(obj_id, x_offset=0.1, y_offset=0., z_offset=.5)
        p.stepSimulation()

        # reset when contact in the first time
        contact_points = p.getContactPoints(obj_id, hook_id)
        if contact_points:
            print('[failed] : reset')
            continue

        # toss to the hook by force in direction x
        p.resetBaseVelocity(obj_id, [-0.1, 0, 0])
        for _ in range(500):
            p.stepSimulation()

        p.resetBaseVelocity(obj_id, [0, 0, 0])
        p.setGravity(0, 0, gravity)
        for _ in range(int(timestep * 1)):
            pos, rot = p.getBasePositionAndOrientation(obj_id)
            if pos[2] < height_thresh:
                failed = True
            p.stepSimulation()

        if failed:
            print('[failed] : gravity first')
            continue
        
        # y force positive test
        p.setGravity(0, 2, -5)
        for _ in range(int(timestep * 1)):
            pos, rot = p.getBasePositionAndOrientation(obj_id)
            if pos[2] < height_thresh:
                failed = True
            p.stepSimulation()
        if failed:
            print('[failed] : y force positive test')
            continue

        # y force negative test
        p.setGravity(0, -2, -5)
        for _ in range(int(timestep * 1)):
            pos, rot = p.getBasePositionAndOrientation(obj_id)
            if pos[2] < height_thresh:
                failed = True
            p.stepSimulation()
        if failed:
            print('[failed] : y force negative test')
            continue
        
        # gravity test
        p.setGravity(0, 0, gravity)
        for _ in range(int(timestep * 1)):
            pos, rot = p.getBasePositionAndOrientation(obj_id)
            if pos[2] < height_thresh:
                failed = True
            p.stepSimulation()
        if failed:
            print('[failed] : gravity second')
            continue

        contact_points = p.getContactPoints(obj_id, hook_id)
        pos, rot = p.getBasePositionAndOrientation(obj_id)

        if len(contact_points) >= 0:
            print(contact_points)
            continue


    cubePos, cubeOrn = p.getBasePositionAndOrientation(obj_id)
    print(cubePos,cubeOrn)
    p.disconnect()

if __name__=="__main__":

    main()
