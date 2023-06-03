# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.
import os, inspect
import argparse
import json
import time
import quaternion
import scipy.io as sio
import numpy as np
from scipy.spatial.transform import Rotation as R
from PIL import Image
import pybullet as p
import pybullet_data

# for robot control
from pybullet_robot_envs.envs.panda_envs.panda_env import pandaEnv

from utils.bullet_utils import get_matrix_from_pose, get_pose_from_matrix, get_matrix_from_pos_rot, get_pos_rot_from_matrix, xyzw2wxyz, wxyz2xyzw, draw_coordinate, draw_bbox

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

def recover_trajectory(traj_src : np.ndarray, hook_poses : np.ndarray, 
                        centers : np.ndarray, scales, dataset_mode : int=0):
    # traj : dim = batch x num_steps x 6
    # dataset_mode : 0 for abosute, 1 for residual 

    traj = None
    traj = np.copy(traj_src)

    waypoints = []

    if dataset_mode == 0: # "absolute"

        traj[:, :3] = traj[:, :3] * scales + centers
        
        hook_trans = get_matrix_from_pose(hook_poses)
        for wpt_id in range(0, traj.shape[0]): # waypoints

            wpt = np.zeros(6)
            # contact pose rotation
            wpt[:3] = traj[wpt_id]

            # transform to world coordinate first
            current_trans = np.identity(4)
            current_trans[:3, 3] = traj[wpt_id]
            current_trans = hook_trans @ current_trans

            if wpt_id < traj.shape[0] - 1:
                # transform to world coordinate first

                peep_num_max = int(np.ceil(traj.shape[0] / 10.0))
                peep_num = peep_num_max if wpt_id < traj.shape[0] - peep_num_max else traj.shape[0] - wpt_id - 1
                to_pos = np.ones((4, peep_num))
                to_pos[:3] = traj[wpt_id:wpt_id+peep_num].T 
                to_pos = (hook_trans @ to_pos)[:3]
                
                from_pos = np.ones((4, peep_num))
                from_pos[:3] = traj[wpt_id+1:wpt_id+peep_num+1].T 
                from_pos = (hook_trans @ from_pos)[:3]

                weight = np.array([1/x for x in range(3, peep_num+3)])[:peep_num]
                weight /= np.sum(weight)
                diff = (to_pos - from_pos) * weight
                
                x_direction = np.sum(diff, axis=1)
                x_direction /= np.linalg.norm(x_direction, ord=2)
                y_direction = np.cross(x_direction, [0, 0, -1])
                y_direction /= np.linalg.norm(y_direction, ord=2)
                z_direction = np.cross(x_direction, y_direction)
                rotation_mat = np.vstack((x_direction, y_direction, z_direction)).T
                current_trans[:3, :3] = rotation_mat
                
            else :

                current_trans[:3, :3] = R.from_rotvec(waypoints[-1][3:]).as_matrix() # use the last waypoint's rotation as current rotation
            
            waypoints.append(get_pose_from_matrix(current_trans, pose_size=6))
    
    return waypoints


def robot_apply_action(robot : pandaEnv, obj_id : int, action : tuple or list, gripper_action : str = 'nop', 
                        sim_timestep : float = 1.0 / 240.0, diff_thresh : float = 0.005, max_vel : float = 0.2, max_iter = 5000):

    assert gripper_action in ['nop', 'pre_grasp', 'grasp']

    if gripper_action == 'nop':
        assert len(action) == 7, 'action length should be 7'

        robot.apply_action(action, max_vel=max_vel)
        diff = 10.0
        iter = 0
        while diff > diff_thresh and iter < max_iter:       
            iter += 1

            p.stepSimulation()
            time.sleep(sim_timestep)

            tmp_pos = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[4] # position
            tmp_rot = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[5] # rotation
            diff = np.sum((np.array(tmp_pos + tmp_rot) - np.array(action)) ** 2) ** 0.5

    elif gripper_action == 'pre_grasp' :

        robot.pre_grasp()
        for _ in range(int(1.0 / sim_timestep) * 1): # 1 sec
            p.stepSimulation()
            time.sleep(sim_timestep)
    else:

        robot.grasp(obj_id)
        for _ in range(int(1.0 / sim_timestep)): # 1 sec
            p.stepSimulation()
            time.sleep(sim_timestep)

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

def refine_rotation(src_transform, tgt_transform):
    src_rot = src_transform[:3, :3]
    tgt_rot = tgt_transform[:3, :3]

    s2d_before = R.from_matrix(tgt_rot @ np.linalg.inv(src_rot)).as_rotvec()

    rot_180 = np.identity(4)
    rot_180[:3, :3] = R.from_rotvec([0, 0, np.pi]).as_matrix()
    tgt_dual_transform = tgt_transform @ rot_180
    s2d_after = R.from_matrix(tgt_dual_transform[:3, :3] @ np.linalg.inv(src_rot)).as_rotvec()

    return tgt_transform if np.sum((s2d_before) ** 2) < np.sum((s2d_after) ** 2) else tgt_dual_transform

def main(args):

    time_stamp = time.localtime()
    time_mon_day = '{:02d}{:02d}'.format(time_stamp.tm_mon, time_stamp.tm_mday)

    data_dir = f'{args.data_root}/{args.data_dir}'
    kpt_trajectory_dir = f'{args.input_root}/{args.input_dir}' if args.input_dir != '' else f'{args.input_root}/{time_mon_day}'

    assert os.path.exists(data_dir), f'{data_dir} not exists'
    assert os.path.exists(kpt_trajectory_dir), f'{kpt_trajectory_dir} not exists'
    assert os.path.exists(args.output_root), f'{args.output_root} not exists'
    
    # load model
    obj_fname = f'{kpt_trajectory_dir}/{args.obj}'
    hook_fname = f'{kpt_trajectory_dir}/{args.hook}'
    obj_name = os.path.split(obj_fname)[1].split('.')[0]
    hook_name = os.path.split(hook_fname)[1].split('.')[0]
    obj_hook_pair_fname = f'{data_dir}/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-{obj_name}.json'

    assert os.path.exists(obj_hook_pair_fname), f'{obj_hook_pair_fname} not exists'
    assert os.path.exists(obj_fname), f'{obj_fname} not exists'
    assert os.path.exists(hook_fname), f'{hook_fname} not exists'

    with open(obj_hook_pair_fname, 'r') as f:
        obj_hook_pair_dict = json.load(f)
    with open(obj_fname, 'r') as f:
        obj_dict = json.load(f)
    with open(hook_fname, 'r') as f:
        hook_dict = json.load(f)
    
    # demonstration_dir = f'{args.output_root}/{args.output_dir}' if args.output_dir != '' else f'{args.output_root}/{time_mon_day}'
    # if not os.path.exists(demonstration_dir):
    #     os.mkdir(demonstration_dir)

    # assert some attributes exist in the given json files
    assert 'initial_pose' in obj_hook_pair_dict.keys(), \
        f'"initial_pose" not in obj_hook_pair_dict!, please run hanging_init_pose.py'
    assert 'contact_pose' in obj_dict.keys() and 'file' in obj_dict.keys(), \
        f'"contact_pose" or "file" not in obj_dict!, please run keypoint_pose.py'
    assert 'hook_pose' in hook_dict.keys() and 'file' in hook_dict.keys() and 'trajectory' in hook_dict.keys(), \
        f'"hook_pose" or "file" or "trajectory" not in hook_dict!, please run keypoint_trajectory.py'
    
    # ------------------------ #
    # --- Setup simulation --- #
    # ------------------------ #

    # Create pybullet GUI
    physics_client_id = p.connect(p.DIRECT)
    # physics_client_id = p.connect(p.GUI)
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
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

    # -------------------------- #
    # --- Load other objects --- #
    # -------------------------- #

    p.loadURDF(os.path.join(pybullet_data.getDataPath(), "table/table.urdf"), [1, 0.0, 0.0])

    obj_contact_pose_6d = obj_dict['contact_pose']
    obj_contact_relative_transform = get_matrix_from_pos_rot(obj_contact_pose_6d[:3], obj_contact_pose_6d[3:])
    obj_id = p.loadURDF(obj_dict['file'])
    # p.resetBasePositionAndOrientation(obj_id, obj_pos, obj_rot)

    hook_pose_6d = hook_dict['hook_pose']
    hook_pos = hook_pose_6d[:3]
    hook_quat = hook_pose_6d[3:]
    hook_id = p.loadURDF(hook_dict['file'], hook_pos, hook_quat)
    hook_transform = get_matrix_from_pos_rot(hook_pos, hook_quat)

    wpt_num = args.wpt_num
    wpt_dim = args.wpt_dim
    preload_path = f'kptraj_{wpt_num}/val/{hook_name}/traj-{args.traj_id}.json'
    assert os.path.exists(preload_path), f'{preload_path} not exists'
    preload_traj_dict = json.load(open(preload_path, 'r')) 
    trajectories_hook = [preload_traj_dict['trajectory'][::-1]]
    # trajectories_hook = hook_dict['trajectory'][2:3]

    # grasping
    index = 0 # medium
    initial_info = obj_hook_pair_dict['initial_pose'][index] # medium
    obj_pos = initial_info['obj_pose'][:3]
    obj_rot = initial_info['obj_pose'][3:]
    # obj_pos = list(np.array(obj_pos) + np.array([0, 0, 0.02]))

    initial_info = obj_hook_pair_dict['initial_pose'][index] # medium
    robot_pos = initial_info['robot_pose'][:3]
    robot_rot = initial_info['robot_pose'][3:]
    # robot_pos = list(np.array(robot_pos) + np.array([0, 0, 0.02]))
    robot_pose = robot_pos + robot_rot
    robot_transform = get_matrix_from_pos_rot(robot_pos, robot_rot)

    for traj_i in range(len(trajectories_hook)):
        
        robot.reset()

        robot.apply_action(robot_pose, max_vel=-1)
        for _ in range(int(1.0 / sim_timestep * 0.5)): # 1 sec
            p.stepSimulation()
            time.sleep(sim_timestep)
        robot.grasp(obj_id=obj_id)
        for _ in range(int(1.0 / sim_timestep * 0.25)): 
            p.resetBasePositionAndOrientation(obj_id, obj_pos, obj_rot)
            p.stepSimulation()
            time.sleep(sim_timestep)
        time.sleep(1)

        obj_transform = get_matrix_from_pos_rot(obj_pos, obj_rot)
        kpt_transform_world = obj_transform @ obj_contact_relative_transform
        
        trajectory_hook = trajectories_hook[traj_i]

        if wpt_dim == 3:
            trajectory_hook_3d = np.asarray(trajectory_hook)[:, :3]
            trajectory_hook_world = recover_trajectory(trajectory_hook_3d, hook_pose_6d, np.array([0, 0, 0]), 1)

            trajectory_hook = []
            for wpt_world in trajectory_hook_world:
                wpt_trans = np.linalg.inv(get_matrix_from_pose(hook_pose_6d)) @ get_matrix_from_pose(wpt_world)
                trajectory_hook.append(list(get_pose_from_matrix(wpt_trans)))

        # first_waypoint = trajectory_hook[-100:][0]
        first_waypoint = trajectory_hook[0]
        relative_kpt_transform = get_matrix_from_pos_rot(first_waypoint[:3], first_waypoint[3:])
        first_kpt_transform_world = hook_transform @ relative_kpt_transform

        kpt_transform_world = refine_rotation(first_kpt_transform_world, kpt_transform_world)

        kpt_to_gripper = np.linalg.inv(kpt_transform_world) @ robot_transform
        first_gripper_transform = first_kpt_transform_world @ kpt_to_gripper

        first_gripper_pos, first_gripper_rot = get_pos_rot_from_matrix(first_gripper_transform)
        first_gripper_pose = list(first_gripper_pos) + list(first_gripper_rot)

        # draw_coordinate(first_kpt_transform_world, size=0.01)

        trajectory_start = get_dense_waypoints(robot_pose, first_gripper_pose, resolution=0.002)
        for waypoint in trajectory_start:
            robot.apply_action(waypoint)
            p.stepSimulation()
            robot.grasp()
            for _ in range(10): # 1 sec
                p.stepSimulation()
                time.sleep(sim_timestep)
        
        old_gripper_pose = first_gripper_pose
        # trajectory_hook = trajectory_hook[-100:-20] if 'hard' in hook_name or 'devil' in hook_name else trajectory_hook[-100:-5]
        
        ignore_wpt_num = int(np.ceil(len(trajectory_hook[0]) * 0.1)) if wpt_dim == 3 else 0
        for i, waypoint in enumerate(trajectory_hook):

            if i + ignore_wpt_num >= len(trajectory_hook):
                break

            waypoint_abs = get_pose_from_matrix(hook_transform @ get_matrix_from_pose(waypoint))

            gripper_transform = get_matrix_from_pose(waypoint_abs) @ kpt_to_gripper
            gripper_pose = get_pose_from_matrix(gripper_transform)

            fine_gripper_poses = get_dense_waypoints(old_gripper_pose, gripper_pose, resolution=0.002)
            for fine_gripper_pose in fine_gripper_poses:
                robot.apply_action(fine_gripper_pose)
                p.stepSimulation()
                
                robot.grasp()
                for _ in range(5): # 1 sec
                    p.stepSimulation()
                    time.sleep(sim_timestep)
            old_gripper_pose = gripper_pose

        # execution step 2 : release gripper
        robot_apply_action(robot, obj_id, gripper_pose, gripper_action='pre_grasp', 
            sim_timestep=0.05, diff_thresh=0.01, max_vel=-1, max_iter=100)

        # execution step 3 : go to the ending pose
        gripper_rot = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[5]
        gripper_rot_matrix = R.from_quat(gripper_rot).as_matrix()
        ending_gripper_pos = np.asarray(gripper_pose[:3]) + (gripper_rot_matrix @ np.array([[0], [0], [-0.05]])).reshape(3)
        action = tuple(ending_gripper_pos) + tuple(gripper_rot)
        robot_apply_action(robot, obj_id, action, gripper_action='nop', 
            sim_timestep=0.05, diff_thresh=0.005, max_vel=-1, max_iter=100)

        # p.removeAllUserDebugItems()

        # for _ in range(int(0.2/sim_timestep)): 
        #     p.stepSimulation()
        #     time.sleep(sim_timestep)

    contact = False
    contact_points = p.getContactPoints(obj_id, hook_id)
    contact = True if contact_points != () else False

    fname_out = f'hanging_by_trajectory_result_{args.traj_id}_{wpt_num}w_{wpt_dim}d.txt'
    f_out = open(fname_out, 'a')
    f_out.write(f'{hook_name},{obj_name},{traj_i},{1 if contact else 0}\n')
    f_out.close()

# start_msg = \
# '''
# ======================================================================================
# this script will execute the hanging process using the collected keypoint trajectories
# in 
# - [input_root]/[input_dir]/[hook_name].json 
# - [input_root]/[input_dir]/[object_name].json

# dependency :
# - object folder that contains /[object_name]/base.urdf
# - hook folder that contains /[hook_name]/base.urdf
# - the keypoint pose of objects in [input_root]/[input_dir]/[obj_name].json
# - the keypoint trajectories of hooks in [input_root]/[input_dir]/[hook_name].json
# - the folder that cantain initial pose of objects in 
#   [data_root]/[data_dir]/[hook_name-object_set]/[hook_name-object_name].json
# note :
# - you can run this script using ./run.sh hangtraj
# ======================================================================================
# '''

# print(start_msg)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--data-root', '-dr', type=str, default='data')
    parser.add_argument('--data-dir', '-dd', type=str, default='everyday_objects_50')
    parser.add_argument('--input-root', '-ir', type=str, default='keypoint_trajectory')
    parser.add_argument('--input-dir', '-id', type=str, default='everyday_objects_50')
    parser.add_argument('--obj', '-obj', type=str, default='hanging_exp_daily_5.json')
    parser.add_argument('--hook', '-hook', type=str, default='Hook_my_bar_easy.json')
    parser.add_argument('--traj_id', '-ti', type=int, default=0)
    parser.add_argument('--wpt_num', '-wn', type=int, default=10)
    parser.add_argument('--wpt_dim', '-wd', type=int, default=3)
    parser.add_argument('--output-root', '-or', type=str, default='demonstration_data')
    parser.add_argument('--output-dir', '-od', type=str, default='')
    parser.add_argument('--save-demo', '-sd', action="store_true")
    parser.add_argument('--save-gif', '-sg', action="store_true")
    args = parser.parse_args()
    main(args)