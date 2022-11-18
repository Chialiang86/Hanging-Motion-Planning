# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.
from tqdm import tqdm
import os, inspect, glob
import argparse
import json
import time
import numpy as np
import quaternion
from scipy.spatial.transform import Rotation as R
import pybullet as p
import pybullet_data
# from PIL import Image

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
    hook_id = p.loadURDF(json_dict['hook_path'], hook_pos, hook_orientation)

    # get target hanging pose
    assert len(json_dict['contact_info']) > 0, 'contact info is empty'
    contact_index = 0
    contact_info = json_dict['contact_info'][contact_index]
    tgt_obj_pos = contact_info['obj_pose'][:3]
    tgt_obj_rot = contact_info['obj_pose'][3:]
    obj_id_target = p.loadURDF(json_dict['obj_path'])
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
    max_iter = 100000
    i = 0
    while i < max_iter and collision7d_fn(tuple(refine_pose)):
        refine_pose6d = np.concatenate((np.asarray(obj_pos), R.from_quat(obj_rot).as_rotvec())) + np.random.uniform(low_limit, high_limit)
        refine_pose = np.concatenate((refine_pose6d[:3], R.from_rotvec(refine_pose6d[3:]).as_quat()))
        # print(refine_pose)
        i += 1
    if i == max_iter:
        print(f'connot find refined pose')
        return None
    print(f'successfully find refined pose')
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
    padding = [0.05, 0.05, 0.1]

    # xlim, ylim
    for i in range(3):
        if start_pos[i] < target_pos[i]:
            low_limit[i] = start_pos[i] - padding[i]
            high_limit[i] = target_pos[i] + padding[i] 
        else :
            low_limit[i] = target_pos[i] - padding[i]
            high_limit[i] = start_pos[i] + padding[i]
    # draw_bbox(low_limit, high_limit) 
    
    sample7d_fn = get_sample7d_fn(target_conf, low_limit, high_limit)
    distance7d_fn = get_distance7d_fn()
    extend7d_fn = get_extend7d_fn(resolution=0.001)
    collision_fn = get_collision7d_fn(physics_client_id, obj_id, obstacles=obstacles)

    if not check_initial_end(start_conf, target_conf, collision_fn, diagnosis=diagnosis):
        return None, None

    return birrt(start_conf, target_conf, distance7d_fn, sample7d_fn, extend7d_fn, collision_fn, **kwargs)

def hanging_by_rrt(physics_client_id : int, robot : pandaEnv, obj_id : int, target_conf : list or tuple, 
                    obstacles : list=[], sim_timestep : float=1.0 / 240.0, max_vel : float=0.2):
    obj_pos, obj_rot = p.getBasePositionAndOrientation(obj_id)

    start_conf = obj_pos + obj_rot
    waypoints, nodes = rrt_connect_7d(physics_client_id, obj_id, start_conf=start_conf, target_conf=target_conf, obstacles=obstacles)
    
    if waypoints is None:
        print("Oops, no solution!")
        return None
    
    return waypoints

def get_kpt_trajectory_from_trajectory(waypoints : list or np.ndarray, contact_relative_transform : np.ndarray, obj_id : int, hook_id : int):

    assert contact_relative_transform.shape == (4, 4), f'wrong shape of contact_relative_transform : {contact_relative_transform.shape}'
    assert len(waypoints) > 0 and len(waypoints[0]) == 7, f'waypoints is empty or each pose is not 7d (pos 3d, rot 4d in x, y, z, w format)'

    hook_pos, hook_rot = p.getBasePositionAndOrientation(hook_id)
    hook_transform = get_matrix_from_pos_rot(hook_pos, hook_rot)

    contact_hook_trajectory_7d = []
    imgs = []
    for i in range(len(waypoints) - 1):
        positions7d = get_dense_waypoints(waypoints[i], waypoints[i+1], resolution=0.001)

        # plan trajectory in the same way in collision detection module
        for position7d in positions7d:

            # object transform
            p.resetBasePositionAndOrientation(obj_id, position7d[:3], position7d[3:])
            time.sleep(1.0 / 240.0)

            # contact pose position
            obj_transform = get_matrix_from_pos_rot(position7d[:3], position7d[3:])
            contact_transform = obj_transform @ contact_relative_transform
            
            # draw_coordinate(contact_transform)
            
            # contact pose relative to hook
            contact_hook_transform = np.linalg.inv(hook_transform) @ contact_transform
            contact_hook_pos, contact_hook_quat = get_pos_rot_from_matrix(contact_hook_transform)
            contact_hook_pose_7d = list(contact_hook_pos) + list(contact_hook_quat)
            contact_hook_trajectory_7d.append(contact_hook_pose_7d)

            img = p.getCameraImage(480, 480, renderer=p.ER_BULLET_HARDWARE_OPENGL)[2]
            imgs.append(img)

    contact_hook_trajectory_7d = np.asarray(contact_hook_trajectory_7d)

    return contact_hook_trajectory_7d, imgs

def shorten_kpt_trajectory(kpt_trajectory : np.ndarray, length=0.05):

    assert kpt_trajectory.shape[0] > 0, f'no waypoint in trajectory'
    assert kpt_trajectory.shape[1] == 7, f'waypoint should be in 7d (x, y, z, x, y, z, w) format'

    tmp_length = 0.0
    tmp_index = kpt_trajectory.shape[0] - 1

    while tmp_index > 0:
        tmp_length += np.linalg.norm(kpt_trajectory[tmp_index][:3] - kpt_trajectory[tmp_index - 1][:3], ord=2)
        tmp_index -= 1
        if tmp_length >= length:
            print(tmp_length)
            break

    return kpt_trajectory[tmp_index:]

def main(args):
    
    # ------------------------ #
    # --- Setup simulation --- #
    # ------------------------ #

    time_stamp = time.localtime()
    time_mon_day = '{:02d}{:02d}'.format(time_stamp.tm_mon, time_stamp.tm_mday)
    dir_postfix = time_mon_day if args.dir_postfix == '' else args.dir_postfix
    max_cnt = args.max_cnt

    # dir name
    dir_name = f'keypoint_trajectory_{dir_postfix}'
    assert os.path.exists(dir_name), f'{dir_name} not exists'

    # Create pybullet GUI
    physics_client_id = p.connect(p.DIRECT)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.2,
        cameraYaw=90,
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
        # for trajectory
        # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_5.json',
        # 'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_daily_5.json',
        # 'data/Hook_60-hanging_exp/Hook_60-hanging_exp_daily_5.json',
        # 'data/Hook_90-hanging_exp/Hook_90-hanging_exp_daily_5.json',
        # 'data/Hook_180-hanging_exp/Hook_180-hanging_exp_daily_5.json',
        # "data/Hook1-hanging_exp/Hook1-hanging_exp_daily_5.json",
        # "data/Hook2-hanging_exp/Hook2-hanging_exp_daily_5.json",
        # "data/Hook12-hanging_exp/Hook12-hanging_exp_daily_5.json",
        # "data/Hook15-hanging_exp/Hook15-hanging_exp_daily_5.json",
        # "data/Hook23-hanging_exp/Hook23-hanging_exp_daily_5.json",
        # "data/Hook35-hanging_exp/Hook35-hanging_exp_daily_5.json",
        # "data/Hook40-hanging_exp/Hook40-hanging_exp_daily_5.json",
        # "data/Hook42-hanging_exp/Hook42-hanging_exp_daily_5.json",
        # "data/Hook44-hanging_exp/Hook44-hanging_exp_daily_5.json",
        # "data/Hook47-hanging_exp/Hook47-hanging_exp_daily_5.json",
        # "data/Hook57-hanging_exp/Hook57-hanging_exp_daily_5.json",
        # "data/Hook84-hanging_exp/Hook84-hanging_exp_daily_5.json",
        # "data/Hook122-hanging_exp/Hook122-hanging_exp_daily_5.json",
        # "data/Hook124-hanging_exp/Hook124-hanging_exp_daily_5.json",
        # "data/Hook136-hanging_exp/Hook136-hanging_exp_daily_5.json",
        # "data/Hook145-hanging_exp/Hook145-hanging_exp_daily_5.json",
        # "data/Hook186-hanging_exp/Hook186-hanging_exp_daily_5.json",
        # "data/Hook209-hanging_exp/Hook209-hanging_exp_daily_5.json",
    ]

    input_jsons = glob.glob('data/*/*-hanging_exp_daily_5.json')
    input_jsons.sort()
    input_jsons = input_jsons[::-1]

    ignore_list = [
        'Hook_bar', 'Hook_skew', 'Hook_90', 'Hook_60', 'Hook_180', 'Hook_84'
    ]

    for input_json in input_jsons:

        ignore = False
        for ignore_item in ignore_list:
            if ignore_item in input_json:
                print(f'ignore {input_json}')
                ignore = True
        if ignore:
            continue

        # time.sleep(2)
        p.removeAllUserDebugItems()
        
        pair = os.path.splitext(input_json)[0].split('/')[-1]
        hook_name = pair.split('-')[0]
        obj_name = pair.split('-')[1]
        in_fname = f'{dir_name}/{obj_name}.json'

        print(f'processing {input_json}')
        assert os.path.exists(input_json), f'{input_json} not exists'
        assert os.path.exists(in_fname), f'{in_fname} not exists'

        f_json =  open(input_json, 'r')
        json_dict = json.load(f_json)
        f_json.close()

        if 'initial_pose' not in json_dict.keys():
            print(f'please get the refined hanging poses and write to {input_json}')
            continue
        if 'contact_info' not in json_dict.keys():
            print(f'please get the contact_info and write to {input_json}')
            continue
        if 'hook_pose' not in json_dict.keys():
            print(f'please get the hook_pose and write to {input_json}')
            continue

        # get the information that needed in rrt
        obj_id, hook_id, tgt_pose = get_obj_hook_pose(physics_client_id, json_dict)

        if tgt_pose is None:
            print(f'ignore {input_json} due to unreliable initial pose')
            p.removeBody(obj_id)
            p.removeBody(hook_id)
            continue
        
        # input keypoint pose relative to object
        with open(in_fname, 'r') as f:
            obj_dict = json.load(f)
            contact_pose = obj_dict['contact_pose']
            contact_object_pose = get_matrix_from_pos_rot(contact_pose[:3], contact_pose[3:])

        # output data structure        
        hook_pos, hook_rot = p.getBasePositionAndOrientation(hook_id)
        hook_pose = list(hook_pos) + list(hook_rot)
        trajectory_dict = {
            'file': json_dict['hook_path'],
            'trajectory': [],
            'hook_pose': hook_pose
        }

        # object position initialization
        # for index, initial_info in tqdm(enumerate(json_dict['initial_pose'])):
        for index in tqdm(range(max_cnt)):
            if max_cnt != -1 and index >= max_cnt:
                break

            initial_info = json_dict['initial_pose'][0] # use initial pose
            obj_init_pos = initial_info['object_pose'][:3]
            obj_init_rot = initial_info['object_pose'][3:]
            p.resetBasePositionAndOrientation(obj_id, obj_init_pos, obj_init_rot)
            draw_coordinate(obj_init_pos + obj_init_rot)

            # run RRT algorithm
            waypoints = None
            while waypoints is None:
                waypoints = hanging_by_rrt(physics_client_id, robot, obj_id, target_conf=tgt_pose, obstacles=[table_id, hook_id], max_vel=0.1)
            if waypoints is None:
                p.removeBody(obj_id)
                p.removeBody(hook_id)
                continue

            # write trajectory relative to hook to file
            contact_hook_trajectory_7d, imgs = get_kpt_trajectory_from_trajectory(waypoints=waypoints, contact_relative_transform=contact_object_pose, obj_id=obj_id, hook_id=hook_id)
            contact_hook_trajectory_7d = shorten_kpt_trajectory(contact_hook_trajectory_7d, length=0.08)

            # # visualize trajectory
            # hook_pos, hook_rot = p.getBasePositionAndOrientation(hook_id)
            # hook_transform = get_matrix_from_pos_rot(hook_pos, hook_rot)
            # for waypoint in contact_hook_trajectory_7d:
            #     relative_transform = get_matrix_from_pos_rot(waypoint[:3], waypoint[3:])
            #     waypoint_transform = hook_transform @ relative_transform
            #     draw_coordinate(waypoint_transform)

            trajectory_dict['trajectory'].append(contact_hook_trajectory_7d.tolist())
            
            # imgs_array = [Image.fromarray(img) for img in imgs]
            # if imgs_array is not None:
            #     gif_path = os.path.join(f'{dir_name}', 'traj_gif', f'{hook_name}-{obj_name}-{index}.gif')
            #     imgs_array[0].save(gif_path, save_all=True, append_images=imgs_array[1:], duration=50, loop=0)
        
            p.removeAllUserDebugItems()

        out_fname = f'{dir_name}/{hook_name}.json'
        with open(out_fname, 'w') as f:
            json.dump(trajectory_dict, f, indent=4)
            print(f'{out_fname} has been written')
        p.removeBody(obj_id)
        p.removeBody(hook_id)

    print('process completed...')        
        
    # while True:
    #     # key callback
    #     keys = p.getKeyboardEvents()            
    #     if ord('q') in keys and keys[ord('q')] & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN): 
    #         break


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir-postfix', '-dp', type=str, default='')
    parser.add_argument('--max-cnt', '-mc', type=int, default='20')
    args = parser.parse_args()
    main(args)