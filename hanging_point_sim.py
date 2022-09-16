# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.

import os, inspect
import argparse
import glob
import time
import json
import numpy as np
import skrobot
import os
import xml.etree.ElementTree as ET

from tqdm import tqdm
from utils.renderer import Renderer

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print(currentdir)
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import pybullet as p
import pybullet_data
from pybullet_robot_envs.envs.panda_envs.panda_env import pandaEnv


def reset_pose(obj_id, x_offset=0.1, y_offset=0., z_offset=1.):
    x = (np.random.rand() - 0.5) * 0.1 + x_offset
    y = (np.random.rand() - 0.5) * 0.4 + y_offset
    z = (np.random.rand() - 0.5) * 0.4 + z_offset

    roll = np.random.rand() * np.pi * 2
    pitch = np.random.rand() * np.pi * 2
    yaw = np.random.rand() * np.pi * 2
    p.setGravity(0, 0, 0)
    p.resetBasePositionAndOrientation(
        obj_id,
        [x, y, z],
        p.getQuaternionFromEuler([roll, pitch, yaw]))

# def load_obj(obj_file, pos=[0,0,0.4], rot=[np.pi/2, 0, np.pi/2], shift=[0, -0.02, 0], scale=[1, 1, 1]):
#     visual_shape_id = p.createVisualShape(
#         shapeType=p.GEOM_MESH,
#         fileName=obj_file,
#         rgbaColor=[1, 1, 1, 1],
#         specularColor=[0.4, 0.4, 0],
#         visualFramePosition=shift,
#         meshScale=scale,
#     )
#     collision_shape_id = p.createCollisionShape(
#         shapeType=p.GEOM_MESH,
#         fileName=obj_file,
#         collisionFramePosition=shift,
#         meshScale=scale,
#     )
#     obj_id = p.createMultiBody(
#         baseMass=0,
#         baseCollisionShapeIndex=collision_shape_id,
#         baseVisualShapeIndex=visual_shape_id,
#         basePosition=[0, 0, 0],
#         useMaximalCoordinates=True
#     )
#     obj_pos = pos
#     obj_orientation = p.getQuaternionFromEuler(rot)
#     p.resetBasePositionAndOrientation(obj_id, obj_pos, obj_orientation)

#     return obj_id


def load_obj_urdf(urdf_path, pos=[0, 0, 0], rot=[0, 0, 0]):

    tree = ET.parse(urdf_path)
    root = tree.getroot()
    center = np.array([float(i) for i in root[0].find(
        "inertial").find("origin").attrib['xyz'].split(' ')])
    obj_id = p.loadURDF(urdf_path, pos, p.getQuaternionFromEuler(rot))
    return obj_id, center

def render(pos, rot):
    rot_matrix = p.getMatrixFromQuaternion(rot)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)

    # camera params
    height = 640
    width = 480
    fx, fy = 596.6278076171875, 596.6278076171875
    cx, cy = 311.98663330078125, 236.76170349121094
    near, far = 0.1, 10

    camera_vector = rot_matrix.dot((0, 0, 1))
    up_vector = rot_matrix.dot((0, -1, 0))

    camera_eye_pos = np.array(pos)
    camera_target_position = camera_eye_pos + 0.2 * camera_vector

    view_matrix = p.computeViewMatrix(camera_eye_pos, camera_target_position, up_vector)

    proj_matrix = (2.0 * fx / width, 0.0, 0.0, 0.0,
                   0.0, 2.0 * fy / height, 0.0, 0.0,
                   1.0 - 2.0 * cx / width, 2.0 * cy / height - 1.0, (far + near) / (near - far), -1.0,
                   0.0, 0.0, 2.0 * far * near / (near - far), 0.0)

    return p.getCameraImage(width=width, height=height, viewMatrix=view_matrix, projectionMatrix=proj_matrix,
                     renderer=p.ER_BULLET_HARDWARE_OPENGL)  # renderer=self._p.ER_TINY_RENDERER)


def reset_pose(obj_id, x_offset=0.1, y_offset=0., z_offset=1.):
    x = (np.random.rand() - 0.5) * 0.2 + x_offset
    y = (np.random.rand() - 0.5) * 0.1 + y_offset
    z = (np.random.rand() - 0.5) * 0.2 + z_offset

    roll = np.random.rand() * np.pi * 2
    pitch = np.random.rand() * np.pi * 2
    yaw = np.random.rand() * np.pi * 2
    p.setGravity(0, 0, 0)
    p.resetBasePositionAndOrientation(
        obj_id,
        [x, y, z],
        p.getQuaternionFromEuler([roll, pitch, yaw]))

def update_debug_param(robot : pandaEnv):

    p.removeAllUserParameters()

    param_ids = []
    joint_ids = []
    num_joints = p.getNumJoints(robot.robot_id)

    joint_states = p.getJointStates(robot.robot_id, range(0, num_joints))
    joint_poses = [x[0] for x in joint_states]

    for i in range(num_joints):
        joint_info = p.getJointInfo(robot.robot_id, i)
        joint_name = joint_info[1]
        joint_type = joint_info[2]

        if joint_type is p.JOINT_REVOLUTE or joint_type is p.JOINT_PRISMATIC:
            joint_ids.append(i)
            param_ids.append(
                p.addUserDebugParameter(joint_name.decode("utf-8"), joint_info[8], joint_info[9], joint_poses[i]))
    
    return param_ids    
    
def main(args):

    # ----------------------- #
    # --- config io paths --- #
    # ----------------------- #

    hook_dir = os.path.join(args.hook_root, args.hook)
    if not os.path.exists(hook_dir):
        print(f'{hook_dir} not exists')
        return

    obj_dir = os.path.join(args.object_root, args.obj)
    if not os.path.exists(obj_dir):
        print(f'{obj_dir} not exists')
        return

    if not os.path.exists(args.output_dir):
        os.mkdir(args.output_dir)
    if not os.path.exists(os.path.join(args.output_dir, f'{args.hook}-{args.obj}')):
        os.mkdir(os.path.join(args.output_dir, f'{args.hook}-{args.obj}'))

    
    hook_path = os.path.join(hook_dir, 'base.urdf')
    obj_paths = glob.glob(f'{obj_dir}/*/base.urdf')

    # ------------------------ #
    # --- Setup simulation --- #
    # ------------------------ #

    # Create pybullet GUI
    physics_client_id = p.connect(p.GUI)
    # p.resetDebugVisualizerCamera(2.1, 90, -30, [0.0, -0.0, -0.0])
    p.resetDebugVisualizerCamera(
        cameraDistance=0.3,
        cameraYaw=120,
        cameraPitch=-30,
        cameraTargetPosition=[0.7, 0.0, 1.3]
    )
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=150)
    sim_timestep = 1.0 / 1000
    gravity = -9.8
    p.setTimeStep(sim_timestep)

    # Load plane contained in pybullet_data
    planeId = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))

    # Set gravity for simulation
    p.setGravity(0, 0, gravity)
    p.stepSimulation()

    # -------------------------- #
    # --- Load other objects --- #
    # -------------------------- #

    p.loadURDF(os.path.join(pybullet_data.getDataPath(), "table/table.urdf"), [1, 0.0, 0.0])
    # obj_id = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "lego/lego.urdf"), [0.5, 0.0, 0.8])
    # obj_id = p.loadURDF(os.path.join(ycb_objects.getDataPath(), 'YcbBanana', "model.urdf"), [0.5, 0.0, 0.8])

    # wall
    # wall_pos = [0.8, -0.21, 0.8]
    # wall_orientation = p.getQuaternionFromEuler([0, 0, 0])
    # wall_id = p.loadURDF("models/wall/wall.urdf", wall_pos, wall_orientation)

    # stop critiria
    max_contact = 1
    max_iter = 1000

    # load hook
    # hook_pos=[0.8, -0.2, 1.0] for hook_90
    # hook_pos=[0.8, -0.2, 1.0]
    hook_pos=[0.5, -0.1, 1.3]
    hook_rot=[np.pi/2, 0, np.pi]
    hook_id = p.loadURDF(hook_path, hook_pos, p.getQuaternionFromEuler(hook_rot))
    
    ignore_list = []
    ignore_list = [
                        'bag_5', 'daily_5', 'scissor_4', 'mug_59', 'wrench_1', 
                        'bag_6', 'bag_70',
                        'scissor_14', 'scissor_19', 'scissor_39', 'scissor_62',
                        'daily_11', 'daily_23', 
                        'headphone_20', 'headphone_22',
                        'mug_67', 'mug_115', 
                        # 'wrench_27', 
                        'wrench_35',
                    ]   
    focus_list = ['scissor_39', 'wrench_27', 'mug_67']

    height_thresh = 0.8
    for obj_path in obj_paths:
        cont = False
        for ignore_item in ignore_list:
            if ignore_item in obj_path:
                print(f'ignore : {ignore_item}')
                cont = True
        if cont:
            continue


        obj_id, center = load_obj_urdf(obj_path)
        
        obj_name = args.obj + '_' + obj_path.split('/')[-2]
        hook_name = args.hook
        result_path = os.path.join(args.output_dir, f'{args.hook}-{args.obj}', f'{hook_name}-{obj_name}.json')
        result_json = {
            'hook_path': hook_path,
            'obj_path': obj_path,
            'hook_pose': hook_pos + list(p.getQuaternionFromEuler(hook_rot)),
            'contact_info': []
        }
        print(f'processing {result_path} ...')
        contact_cnt = 0
        try_num = 0

        while contact_cnt < max_contact :# and try_num < max_iter:
            try_num += 1
            failed = False

            p.setGravity(0, 0, 0)

            if args.hook == 'Hook_bar':
                reset_pose(obj_id, x_offset=0.5, y_offset=0.0, z_offset=1.4) # for hook_bar
            elif args.hook == 'Hook_180':
                reset_pose(obj_id, x_offset=0.5, y_offset=0.0, z_offset=1.4) # for hook_bar
            elif args.hook == 'Hook_90':
                reset_pose(obj_id, x_offset=0.5, y_offset=0.0, z_offset=1.35) # for hook_90
            elif args.hook == 'Hook_60':
                reset_pose(obj_id, x_offset=0.5, y_offset=0.0, z_offset=1.35) # for hook_60
            elif args.hook == 'Hook_skew':
                reset_pose(obj_id, x_offset=0.5, y_offset=0.0, z_offset=1.3) # for hook_60

            p.stepSimulation()
            contact_points = p.getContactPoints(obj_id, hook_id)
            if contact_points:
                continue

            # toss to the hook by force in direction x
            if args.hook == 'Hook_bar':
                p.resetBaseVelocity(obj_id, [0.0, -0.2, -0.1]) # for hook_bar
            elif args.hook == 'Hook_180':
                p.resetBaseVelocity(obj_id, [0.0, -0.2, -0.1]) # for hook_bar
            elif args.hook == 'Hook_90':
                p.resetBaseVelocity(obj_id, [0.0, -0.2, -0.1]) # for hook_90
            elif args.hook == 'Hook_60':
                p.resetBaseVelocity(obj_id, [0.0, -0.2, -0.1]) # for hook_90
            elif args.hook == 'Hook_skew':
                p.resetBaseVelocity(obj_id, [0.0, -0.2, -0.1]) # for hook_90

            
            for _ in range(500):
                p.stepSimulation()
                pos, rot = p.getBasePositionAndOrientation(obj_id)
                if pos[2] < height_thresh:
                    failed = True
                    break
            if failed:
                continue

            p.resetBaseVelocity(obj_id, [0, 0, 0])
            p.setGravity(0, 0, gravity)
            for _ in range(1000):
                p.stepSimulation()
                pos, rot = p.getBasePositionAndOrientation(obj_id)
                if pos[2] < height_thresh:
                    failed = True
                    break

            if failed:
                continue

            contact_points = p.getContactPoints(obj_id, hook_id)
            if len(contact_points) == 0:
                continue

            p.setGravity(2, 0, -5)
            for _ in range(1000):
                pos, rot = p.getBasePositionAndOrientation(obj_id)
                if pos[2] < height_thresh:
                    failed = True
                    break
                p.stepSimulation()
            if failed:
                continue

            p.setGravity(-2, 0, -5)
            for _ in range(1000):
                pos, rot = p.getBasePositionAndOrientation(obj_id)
                if pos[2] < height_thresh:
                    failed = True
                p.stepSimulation()
            if failed:
                continue

            p.setGravity(0, 0, gravity)
            for _ in range(20000):
                pos, rot = p.getBasePositionAndOrientation(obj_id)
                if pos[2] < height_thresh:
                    failed = True
                    break

                p.stepSimulation()
            if failed:
                continue

            contact_points = p.getContactPoints(obj_id, hook_id)
            if len(contact_points) == 0:
                continue

            obj_coords = skrobot.coordinates.Coordinates(
                pos=pos,
                rot=skrobot.coordinates.math.xyzw2wxyz(rot))

            min_height_contact_point = sorted(
                contact_points, key=lambda x: x[5][2])[0][5]

            contact_point = skrobot.coordinates.Coordinates(
                pos=min_height_contact_point,
                rot=skrobot.coordinates.math.rotation_matrix_from_axis(
                    [0, -1, 0], [0, 0, -1]))

            contact_point_obj = obj_coords.inverse_transformation().transform(
                contact_point).translate(center, 'world')

            pose = np.concatenate(
                [contact_point_obj.T()[:3, 3][None, :],
                 contact_point_obj.T()[:3, :3]]).tolist()
                
            contact_info = {
                'contact_pose': pose,
                'object_pose': list(pos + rot)
            }
            contact_cnt+=1
            print(f'{contact_cnt} : {contact_info}')

            result_json['contact_info'].append(contact_info)
        p.removeBody(obj_id)
        if len(result_json['contact_info']) > 0:
            with open(result_path, 'w') as f:
                json.dump(result_json, f, indent=4)
                print(f'{result_path} saved')
        else :
            print(f'no pose : {obj_path}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', '-or', type=str, default='data')
    parser.add_argument('--object-root', '-ir', type=str, default='models/geo_data')
    parser.add_argument('--hook-root', '-hr', type=str, default='models/hook')
    parser.add_argument('--obj', '-o', type=str, default='hanging_exp')
    parser.add_argument('--hook', '-ho', type=str, default='Hook_90')
    args = parser.parse_args()
    main(args)