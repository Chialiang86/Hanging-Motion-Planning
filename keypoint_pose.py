# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.
import os, inspect
import argparse
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import pybullet as p
import pybullet_data
import quaternion
from PIL import Image

# for motion planners
from utils.motion_planning_utils import get_sample7d_fn, get_distance7d_fn, get_extend7d_fn, get_collision7d_fn
from pybullet_planning.interfaces.planner_interface.joint_motion_planning import check_initial_end
from pybullet_planning.motion_planners.rrt_connect import birrt

# for robot control
from pybullet_robot_envs.envs.panda_envs.panda_env import pandaEnv

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print(currentdir)
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)


def get_obj_hook_pose(physics_client_id, json_dict : dict):

    # hook initialization
    hook_pos = json_dict['hook_pose'][:3]
    hook_orientation = json_dict['hook_pose'][3:]
    # set the original coordinate of the hook
    hook_id = p.loadURDF(json_dict['hook_path'], hook_pos, hook_orientation)

    # get target hanging pose
    assert len(json_dict['contact_info']) > 0, 'contact info is empty'
    contact_index = 0
    contact_info = json_dict['contact_info'][contact_index]
    tgt_obj_pos = contact_info['obj_pose'][:3]
    tgt_obj_rot = contact_info['obj_pose'][3:]

    obj_id_target = p.loadURDF(json_dict['obj_path'])
    # set the center coordinate of the object
    p.resetBasePositionAndOrientation(obj_id_target, tgt_obj_pos, tgt_obj_rot)
    tgt_pose = refine_tgt_obj_pose(physics_client_id, obj_id_target, obstacles=[hook_id])
    p.removeBody(obj_id_target)

    obj_id = p.loadURDF(json_dict['obj_path'])

    return obj_id, hook_id, tgt_pose

def xyzw2wxyz(quat : np.ndarray):
    assert len(quat) == 4, f'quaternion size must be 4, got {len(quat)}'
    return np.asarray([quat[3], quat[0], quat[1], quat[2]])

def wxyz2xyzw(quat : np.ndarray):
    assert len(quat) == 4, f'quaternion size must be 4, got {len(quat)}'
    return np.asarray([quat[1], quat[2], quat[3], quat[0]])

def get_matrix_from_pos_rot(pos : list or tuple or np.ndarray, rot : list or tuple or np.ndarray):
    assert (len(pos) == 3 and len(rot) == 4) or (len(pos) == 3 and len(rot) == 3)
    pos_m = np.asarray(pos)
    if len(rot) == 3:
        rot_m = R.from_rotvec(rot).as_matrix()
        # rot_m = np.asarray(p.getMatrixFromQuaternion(p.getQuaternionFromEuler(rot))).reshape((3, 3))
    elif len(rot) == 4: # x, y, z, w
        rot_m = R.from_quat(rot).as_matrix()
        # rot_m = np.asarray(p.getMatrixFromQuaternion(rot)).reshape((3, 3))
    ret_m = np.identity(4)
    ret_m[:3, :3] = rot_m
    ret_m[:3, 3] = pos_m
    return ret_m

def get_pos_rot_from_matrix(pose : np.ndarray):
    assert pose.shape == (4, 4)
    pos = pose[:3, 3]
    rot = R.from_matrix(pose[:3, :3]).as_quat()
    return pos, rot

def refine_tgt_obj_pose(physicsClientId, body, obstacles=[]):
    collision7d_fn = get_collision7d_fn(physicsClientId, body, obstacles=obstacles)

    low_limit = [-0.005, -0.005, -0.005, -np.pi / 180, -np.pi / 180, -np.pi / 180]
    high_limit = [ 0.005,  0.005,  0.005,  np.pi / 180,  np.pi / 180,  np.pi / 180]
    obj_pos, obj_rot = p.getBasePositionAndOrientation(body)

    original_pose = np.asarray(obj_pos + obj_rot)
    refine_pose = original_pose
    while collision7d_fn(tuple(refine_pose)):
        refine_pose6d = np.concatenate((np.asarray(obj_pos), R.from_quat(obj_rot).as_rotvec())) + np.random.uniform(low_limit, high_limit)
        refine_pose = np.concatenate((refine_pose6d[:3], R.from_rotvec(refine_pose6d[3:]).as_quat()))
        # print(refine_pose)
    return refine_pose

def draw_coordinate(pose : np.ndarray or tuple or list, size : float = 0.1):
    assert (type(pose) == np.ndarray and pose.shape == (4, 4)) or (len(pose) == 7)

    if len(pose) == 7:
        pose = get_matrix_from_pos_rot(pose[:3], pose[3:])

    origin = pose[:3, 3]
    x = origin + pose[:3, 0] * size
    y = origin + pose[:3, 1] * size
    z = origin + pose[:3, 2] * size
    p.addUserDebugLine(origin, x, [1, 0, 0])
    p.addUserDebugLine(origin, y, [0, 1, 0])
    p.addUserDebugLine(origin, z, [0, 0, 1])

def draw_bbox(start : list or tuple or np.ndarray,
              end : list or tuple or np.ndarray):
    
    assert len(start) == 3 and len(end) == 3, f'infeasible size of position, len(position) must be 3'

    points_bb = [
        [start[0], start[1], start[2]],
        [end[0], start[1], start[2]],
        [end[0], end[1], start[2]],
        [start[0], end[1], start[2]],
        [start[0], start[1], end[2]],
        [end[0], start[1], end[2]],
        [end[0], end[1], end[2]],
        [start[0], end[1], end[2]],
    ]

    for i in range(4):
        p.addUserDebugLine(points_bb[i], points_bb[(i + 1) % 4], [1, 0, 0])
        p.addUserDebugLine(points_bb[i + 4], points_bb[(i + 1) % 4 + 4], [1, 0, 0])
        p.addUserDebugLine(points_bb[i], points_bb[i + 4], [1, 0, 0])

def get_dense_waypoints(start_config : list or tuple or np.ndarray, end_config : list or tuple or np.ndarray, resolution : float=0.005):

    assert len(start_config) == 7 and len(end_config) == 7

    d12 = np.asarray(end_config[:3]) - np.asarray(start_config[:3])
    steps = int(np.ceil(np.linalg.norm(np.divide(d12, resolution), ord=2)))
    obj_init_quat = quaternion.as_quat_array(xyzw2wxyz(start_config[3:]))
    obj_tgt_quat = quaternion.as_quat_array(xyzw2wxyz(end_config[3:]))

    ret = []
    # plan trajectory in the same way in collision detection module
    for step in range(steps):
        ratio = (step + 1) / steps
        pos = ratio * d12 + np.asarray(start_config[:3])
        quat = quaternion.slerp_evaluate(obj_init_quat, obj_tgt_quat, ratio)
        quat = wxyz2xyzw(quaternion.as_float_array(quat))
        position7d = tuple(pos) + tuple(quat)
        ret.append(position7d)

    return ret

def rrt_connect_7d(physics_client_id, obj_id, start_conf, target_conf, 
                    obstacles : list = [], sim_timestep : float = 1.0 / 240.0, max_vel : float = 0.2, diagnosis=False, **kwargs):

    # https://github.com/yijiangh/pybullet_planning/blob/dev/src/pybullet_planning/motion_planners/rrt_connect.py

    # config sample location space
    start_pos = start_conf[:3]
    target_pos = target_conf[:3]
    
    low_limit = [0, 0, 0]
    high_limit = [0, 0, 0]
    padding = [0.1, 0.1, 0.2]

    # xlim, ylim
    for i in range(3):
        if start_pos[i] < target_pos[i]:
            low_limit[i] = start_pos[i] - padding[i]
            high_limit[i] = target_pos[i] + padding[i] 
        else :
            low_limit[i] = target_pos[i] - padding[i]
            high_limit[i] = start_pos[i] + padding[i] 
    
    sample7d_fn = get_sample7d_fn(target_conf, low_limit, high_limit)
    distance7d_fn = get_distance7d_fn()
    extend7d_fn = get_extend7d_fn(resolution=0.001)
    collision_fn = get_collision7d_fn(physics_client_id, obj_id, obstacles=obstacles)

    if not check_initial_end(start_conf, target_conf, collision_fn, diagnosis=diagnosis):
        return None

    return birrt(start_conf, target_conf, distance7d_fn, sample7d_fn, extend7d_fn, collision_fn, **kwargs)

def hanging_by_rrt(physics_client_id : int, robot : pandaEnv, obj_id : int, target_conf : list or tuple, 
                    obstacles : list=[], sim_timestep : float=1.0 / 240.0, max_vel : float=0.2):
    obj_pos, obj_rot = p.getBasePositionAndOrientation(obj_id)

    start_conf = obj_pos + obj_rot
    waypoints, nodes = rrt_connect_7d(physics_client_id, obj_id, start_conf=start_conf, target_conf=target_conf, obstacles=obstacles)
    
    if waypoints is None:
        print("Oops, no solution!")
        return

    p.resetBasePositionAndOrientation(obj_id, waypoints[-1][:3], waypoints[-1][3:])
    for _ in range(200): # 1 sec
        p.stepSimulation()
        time.sleep(sim_timestep)
    
    return waypoints

def get_obj_to_cp(obj_id, hook_id):
    contact_points = p.getContactPoints(obj_id, hook_id)
    if contact_points == ():
        return None

    min_height_contact_point = sorted(
        contact_points, key=lambda x: x[5][2])[0][5]
    min_height_contact_point = [min_height_contact_point[0], min_height_contact_point[1], min_height_contact_point[2] -0.008]
    obj_pos, obj_rot = p.getBasePositionAndOrientation(obj_id)

    obj_transform = get_matrix_from_pos_rot(obj_pos, obj_rot)

    contact_point_homo = np.ones((4, 1))
    contact_point_homo[:3, 0] = min_height_contact_point
    obj_to_contact_point = np.linalg.inv(obj_transform) @ contact_point_homo
    return obj_to_contact_point

def get_contact_pose_from_trajectory(waypoints : list or np.ndarray, obj_id : int, hook_id : int):
    
    obj_to_contact_point = get_obj_to_cp(obj_id, hook_id)

    contact_object_poses_7d = []
    for i in range(len(waypoints) - 1):
        positions7d = get_dense_waypoints(waypoints[i], waypoints[i+1], resolution=0.002)

        # plan trajectory in the same way in collision detection module
        for index in range(len(positions7d) - 1):

            # object transform
            object_transform = get_matrix_from_pos_rot(positions7d[index][:3], positions7d[index][3:])
            p.resetBasePositionAndOrientation(obj_id, positions7d[index][:3], positions7d[index][3:])
            time.sleep(1.0 / 240.0)

            # contact pose position
            contact_pos = (object_transform @ obj_to_contact_point).reshape(4)[:3]

            # contact pose rotation
            x_direction = np.asarray(positions7d[index + 1][:3]) - np.asarray(positions7d[index][:3])
            x_direction /= np.linalg.norm(x_direction, ord=2)
            y_direction = np.cross(x_direction, [0, 0, -1])
            y_direction /= np.linalg.norm(y_direction, ord=2)
            z_direction = np.cross(x_direction, y_direction)
            rotation_mat = np.vstack((x_direction, y_direction, z_direction)).T
            contact_quat = R.from_matrix(rotation_mat).as_quat()

            # contact transform
            contact_transform = get_matrix_from_pos_rot(contact_pos, contact_quat)
            # draw_coordinate(contact_transform)
            
            # contact pose relative to object
            contact_relative_transform = np.linalg.inv(object_transform) @ contact_transform
            contact_relative_pos = contact_relative_transform[:3, 3]
            contact_relative_quat = R.from_matrix(contact_relative_transform[:3, :3]).as_quat()
            contact_relative_pose_7d = list(contact_relative_pos) + list(contact_relative_quat)
            contact_object_poses_7d.append(contact_relative_pose_7d)
    
    # mean of position
    # contact_object_pos = np.mean(contact_object_poses_7d[:, :3], axis=0)

    # # mean of rotation
    # # see : https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
    # Q = contact_object_poses_7d[:, 3:].T # 4 x N
    # QQt =  Q @ Q.T
    # egvalues, egvecs = np.linalg.eig(QQt)
    # contact_object_quat = egvecs[0]
    # contact_object_pose_7d = np.concatenate((contact_object_pos, contact_object_quat))
    
    length = len(contact_object_poses_7d)
    remain = 0.5
    contact_object_poses_7d = np.asarray(contact_object_poses_7d)
    contact_object_poses_7d = contact_object_poses_7d[:int(length * remain)] # omit some of them
    contact_object_pose_7d = np.mean(contact_object_poses_7d, axis=0)
    contact_object_pose_7d[3:] /= np.linalg.norm(contact_object_pose_7d[3:], ord=2)

    return contact_object_pose_7d

def get_kpt_trajectory_from_trajectory(waypoints : list or np.ndarray, contact_relative_transform : np.ndarray, obj_id : int, hook_id : int):

    assert contact_relative_transform.shape == (4, 4), f'wrong shape of contact_relative_transform : {contact_relative_transform.shape}'
    assert len(waypoints) > 0 and len(waypoints[0]) == 7, f'waypoints is empty or each pose is not 7d (pos 3d, rot 4d in x, y, z, w format)'

    hook_pos, hook_rot = p.getBasePositionAndOrientation(hook_id)
    hook_transform = get_matrix_from_pos_rot(hook_pos, hook_rot)

    contact_hook_trajectory_7d = []
    imgs = []
    for i in range(len(waypoints) - 1):
        positions7d = get_dense_waypoints(waypoints[i], waypoints[i+1], resolution=0.002)

        # plan trajectory in the same way in collision detection module
        for position7d in positions7d:

            # object transform
            p.resetBasePositionAndOrientation(obj_id, position7d[:3], position7d[3:])
            time.sleep(1.0 / 240.0)

            # contact pose position
            obj_transform = get_matrix_from_pos_rot(position7d[:3], position7d[3:])
            contact_transform = obj_transform @ contact_relative_transform
            
            draw_coordinate(contact_transform)
            
            # contact pose relative to hook
            contact_hook_transform = np.linalg.inv(hook_transform) @ contact_transform
            contact_hook_pos, contact_hook_quat = get_pos_rot_from_matrix(contact_hook_transform)
            contact_hook_pose_7d = list(contact_hook_pos) + list(contact_hook_quat)
            contact_hook_trajectory_7d.append(contact_hook_pose_7d)

            img = p.getCameraImage(480, 480, renderer=p.ER_BULLET_HARDWARE_OPENGL)[2]
            imgs.append(img)

    
    contact_hook_trajectory_7d = np.asarray(contact_hook_trajectory_7d)

    return contact_hook_trajectory_7d, imgs

def main(args):

    assert os.path.exists(args.output_root), f'{args.output_root} not exists'
    
    # ------------------------ #
    # --- Setup simulation --- #
    # ------------------------ #

    # Create pybullet GUI
    physics_client_id = p.connect(p.GUI)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.3,
        cameraYaw=120,
        cameraPitch=-30,
        cameraTargetPosition=[0.5, 0.0, 1.3]
    )
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=150)
    sim_timestep = 1.0 / 240
    p.setTimeStep(sim_timestep)
    p.setGravity(0, 0, -9.8)

    # ------------------- #
    # --- Setup robot --- #
    # ------------------- #

    # Load plane contained in pybullet_data
    p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
    robot = pandaEnv(physics_client_id, use_IK=1)
    p.stepSimulation()

    # -------------------------- #
    # --- Load other objects --- #
    # -------------------------- #

    table_id = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "table/table.urdf"), [1, 0.0, 0.0])

    input_jsons = [
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_bag_5.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_bag_6.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_bag_70.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_106.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_114.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_115.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_11.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_23.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_2.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_41.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_42.json",
        "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_5.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_63.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_71.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_72.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_7.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_84.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_85.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_8.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_97.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_100.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_112.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_113.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_115.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_118.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_11.json" ,
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_123.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_126.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_128.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_129.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_132.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_135.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_142.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_145.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_146.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_147.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_149.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_150.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_159.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_166.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_173.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_181.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_184.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_193.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_199.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_19.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_204.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_205.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_43.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_59.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_64.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_67.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_70.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_73.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_80.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_82.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_90.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_101.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_12.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_14.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_19.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_22.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_27.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_31.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_39.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_48.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_4.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_58.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_62.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_74.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_79.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_8.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_92.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_95.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_98.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_10.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_12.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_17.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_1.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_25.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_27.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_31.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_32.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_35.json",
        # "data/data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_36.json",
    ]

    # extract args
    # input_json = args.input_json

    for input_json in input_jsons:

        p.removeAllUserDebugItems()
        
        pair = os.path.splitext(input_json)[0].split('/')[-1]
        obj_name = pair.split('-')[1]

        if not os.path.exists(input_json):
            print(f'{input_json} not exists')

        json_dict = None
        with open(input_json, 'r') as f:
            json_dict = json.load(f)

        if 'contact_info' not in json_dict.keys():
            print('please get the contact_info.')
            continue
        if 'hook_pose' not in json_dict.keys():
            print('please get the hook_pose.')
            continue

        # get the information that needed in rrt
        obj_id, hook_id, tgt_pose = get_obj_hook_pose(physics_client_id, json_dict)
        contact_point_pos_obj = json_dict['contact_info'][0]['contact_point_obj'][:3] # homogeneous position

        # object position initialization
        obj_init_pos = list(np.asarray(tgt_pose[:3]) + np.array([0.0, 0.15, 0.0]))
        obj_init_rot = tgt_pose[3:]
        p.resetBasePositionAndOrientation(obj_id, obj_init_pos, obj_init_rot)

        # run RRT algorithm
        waypoints = None
        while waypoints is None:
            waypoints = hanging_by_rrt(physics_client_id, robot, obj_id, target_conf=tgt_pose, obstacles=[table_id, hook_id], max_vel=0.1)
        
        # write contact pose relative to object to file
        # TODO: extract rotation only
        contact_object_pose_7d = None
        contact_object_pose_7d = get_contact_pose_from_trajectory(waypoints=waypoints, obj_id=obj_id, hook_id=hook_id) 
        
        # Omit position
        contact_object_pos = contact_object_pose_7d[:3] # will be omited
        contact_object_quat = contact_object_pose_7d[3:]
        contact_object_pose_7d = list(contact_point_pos_obj) + list(contact_object_quat)
        obj_pos, obj_rot = p.getBasePositionAndOrientation(obj_id)
        obj_pose = list(obj_pos) + list(obj_rot)
        obj_dict = {
            'file': json_dict['obj_path'],
            # 'contact_pose': contact_object_pose_7d.tolist(),
            'contact_pose': contact_object_pose_7d,
            'obj_pose': obj_pose
        }

        # dir name
        out_fname = ''
        if args.output_dir == '':
            os.makedirs(f'{args.output_root}/keypoint_trajectory', exist_ok=True)
            out_fname = f'{args.output_root}/keypoint_trajectory/{obj_name}.json'
        else :
            os.makedirs(f'{args.output_root}/{args.output_dir}', exist_ok=True)
            out_fname = f'{args.output_root}/{args.output_dir}/{obj_name}.json'
        with open(out_fname, 'w') as f:
            json.dump(obj_dict, f, indent=4)
            print(f'{out_fname} has been written')
        
        p.removeAllUserDebugItems()
        p.resetBasePositionAndOrientation(obj_id, obj_init_pos, obj_init_rot)
        obj_transform = get_matrix_from_pos_rot(obj_init_pos, obj_init_rot)
        contact_relitive_transform = get_matrix_from_pos_rot(contact_object_pose_7d[:3], contact_object_pose_7d[3:])
        contact_transform = obj_transform @ contact_relitive_transform
        draw_coordinate(obj_transform)
        draw_coordinate(contact_transform)
        time.sleep(6)

        p.removeBody(obj_id)
        p.removeBody(hook_id)

    print('process completed...')        
        
    # while True:
    #     # key callback
    #     keys = p.getKeyboardEvents()            
    #     if ord('q') in keys and keys[ord('q')] & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN): 
    #         break

start_msg = \
'''
======================================================================================
this script will output the keypoint pose of a predefined objects into
[output_root]/[output_dir]/[obj_name].json

dependency :
- hook folder that contains [hook_root]/[hook_name]/base.urdf
- object folder that contains [object_root]/[object_name]/base.urdf
- the source folder that contain [data_root]/[pivot_root]/[hook_name-object_name]/[hook_name-object_name].json
======================================================================================
'''

print(start_msg)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # parser.add_argument('--input-json', '-ij', type=str, default='data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_bag_5.json')
    # parser.add_argument('--id', '-id', type=str)
    parser.add_argument('--output-root', '-or', type=str, default='keypoint_trajectory')
    parser.add_argument('--output-dir', '-od', type=str, default='')
    args = parser.parse_args()
    main(args)