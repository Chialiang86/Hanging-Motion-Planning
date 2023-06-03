# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.
import os, inspect, importlib, argparse, json, time, sys, quaternion, glob
import torch
import numpy as np
import open3d as o3d
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
from pointnet2_ops.pointnet2_utils import furthest_point_sample
import pybullet as p
import pybullet_data

from tqdm import tqdm


from PIL import Image

# for motion planners
from utils.motion_planning_utils import get_sample7d_fn, get_distance7d_fn, get_extend7d_fn, get_collision7d_fn, get_collision7d_nce_fn
from pybullet_planning.interfaces.planner_interface.joint_motion_planning import plan_joint_motion, check_initial_end
from pybullet_planning.interfaces.control.control import trajectory_controller
from pybullet_planning.motion_planners.rrt_connect import birrt

from utils.bullet_utils import xyzw2wxyz, wxyz2xyzw, get_matrix_from_pos_rot, get_pos_rot_from_matrix, get_matrix_from_pose, get_pose_from_matrix

# for robot control
# from pybullet_planning.interfaces.robots.joint import get_custom_limits
from pybullet_robot_envs.envs.panda_envs.panda_env import pandaEnv

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print(currentdir)
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

def load_obj_urdf(urdf_path, pos=[0, 0, 0], rot=[0, 0, 0]):

    tree = ET.parse(urdf_path)
    root = tree.getroot()
    center = np.array([float(i) for i in root[0].find(
        "inertial").find("origin").attrib['xyz'].split(' ')])
    
    obj_id = -1
    if len(rot) == 3:
        obj_id = p.loadURDF(urdf_path)
        p.resetBasePositionAndOrientation(obj_id, pos, p.getQuaternionFromEuler(rot))
    elif len(rot) == 4:
        obj_id = p.loadURDF(urdf_path)
        p.resetBasePositionAndOrientation(obj_id, pos, rot)
        
    return obj_id, center

def do_fps(src_points: np.ndarray, sample_num_points = 1000):

    src_points_batch = torch.from_numpy(src_points.copy().astype(np.float32)).unsqueeze(0).to('cuda').contiguous()

    input_pcid = None
    point_num = src_points_batch.shape[1]
    if point_num >= sample_num_points:
        input_pcid = furthest_point_sample(src_points_batch, sample_num_points).long().reshape(-1)  # BN
        ret_points = src_points_batch[0, input_pcid, :].squeeze().cpu().numpy()
    else :
        mod_num = sample_num_points % point_num
        repeat_num = int(sample_num_points // point_num)
        input_pcid = furthest_point_sample(src_points_batch, mod_num).long().reshape(-1)  # BN
        ret_points = torch.cat([src_points_batch.repeat(1, repeat_num, 1), src_points_batch[:, input_pcid]], dim=1).squeeze().cpu().numpy()
    
    return ret_points

def get_model_module(module_name, model_name):
    importlib.invalidate_caches()
    module = importlib.import_module(module_name)
    model = getattr(module, model_name)
    return model


def render(robot):
    pos, rot, _, _, _, _ = p.getLinkState(robot.robot_id, linkIndex=robot.end_eff_idx, computeForwardKinematics=True)
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

    p.getCameraImage(width=width, height=height, viewMatrix=view_matrix, projectionMatrix=proj_matrix,
                     renderer=p.ER_BULLET_HARDWARE_OPENGL)  # renderer=self._p.ER_TINY_RENDERER)

def get_dense_waypoints(start_config : list or tuple or np.ndarray, end_config : list or tuple or np.ndarray, resolution : float=0.005):

    assert len(start_config) == 7 and len(end_config) == 7

    d12 = np.zeros(6)
    d12[:3] = np.asarray(end_config)[:3] - np.asarray(start_config)[:3]
    d12[3:] = R.from_matrix(R.from_quat(end_config[3:]).as_matrix() @ np.linalg.inv(R.from_quat(start_config[3:]).as_matrix())).as_rotvec()
    steps = int(np.ceil(np.linalg.norm(np.divide(d12, resolution), ord=2)))
    obj_init_quat = quaternion.as_quat_array(xyzw2wxyz(start_config[3:]))
    obj_tgt_quat = quaternion.as_quat_array(xyzw2wxyz(end_config[3:]))

    ret = []
    # plan trajectory in the same way in collision detection module
    for step in range(steps):
        ratio = (step + 1) / steps
        pos = ratio * d12[:3] + np.asarray(start_config[:3])
        quat = quaternion.slerp_evaluate(obj_init_quat, obj_tgt_quat, ratio)
        quat = wxyz2xyzw(quaternion.as_float_array(quat))
        position7d = tuple(pos) + tuple(quat)
        ret.append(position7d)

    return ret

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

def get_target_gripper_pose(robot : pandaEnv, obj_id : int, tgt_obj_pos : tuple or list, tgt_obj_rot : tuple or list,):

    # extract current object pose
    obj_pos, obj_rot = p.getBasePositionAndOrientation(obj_id)
    origin_obj_pose = get_matrix_from_pos_rot(obj_pos, obj_rot)

    # extract current gripper pose
    gripper_pos = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[4]
    gripper_rot = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[5]
    origin_gripper_pose = get_matrix_from_pos_rot(gripper_pos, gripper_rot)

    # extract target pose
    tgt_obj_pose = get_matrix_from_pos_rot(tgt_obj_pos, tgt_obj_rot)

    # relative transform from object to gripper
    # see this stack-overflow issue : https://stackoverflow.com/questions/67001118/relative-transform
    obj2gripper_pose = np.linalg.inv(origin_obj_pose) @ origin_gripper_pose

    # target gripper pose by relative transform
    tgt_gripper_pose = tgt_obj_pose @ obj2gripper_pose

    # extract robot action (3 dim for position and 4 dim for rotation)
    tgt_gripper_pos = tgt_gripper_pose[:3, 3]
    tgt_gripper_rot = R.from_matrix(tgt_gripper_pose[:3, :3]).as_quat()

    return tuple(tgt_gripper_pos), tuple(tgt_gripper_rot)

def refine_tgt_obj_pose(body, collision7d_fn=None):


    low_limit = [-0.005, -0.005, -0.005, -np.pi / 180, -np.pi / 180, -np.pi / 180]
    high_limit = [ 0.005,  0.005,  0.005,  np.pi / 180,  np.pi / 180,  np.pi / 180]

    obj_pos, obj_rot = p.getBasePositionAndOrientation(body)
    original_pose = np.asarray(obj_pos + obj_rot)
    refine_pose = original_pose
    p.resetBasePositionAndOrientation(body, obj_pos, obj_rot)
    max_iter = 10000
    i = 0
    while i < max_iter and collision7d_fn(tuple(refine_pose)):
        refine_pose6d = np.concatenate((np.asarray(obj_pos), R.from_quat(obj_rot).as_rotvec())) + np.random.uniform(low_limit, high_limit)
        refine_pose = np.concatenate((refine_pose6d[:3], R.from_rotvec(refine_pose6d[3:]).as_quat()))
        i += 1
    return refine_pose

def draw_coordinate(pose : np.ndarray or tuple or list, size : float = 0.02):
    assert (type(pose) == np.ndarray and pose.shape == (4, 4)) or len(pose) == 7

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

def hanging_by_ik(robot : pandaEnv, obj_id : int, target_conf: tuple or list, 
                    sim_timestep : float = 1.0 / 240.0, max_vel : float = 0.2):
    
    tgt_gripper_pos, tgt_gripper_rot = get_target_gripper_pose(robot, obj_id, target_conf[:3], target_conf[3:])

    # execution step 1 : go to the pose that nearby the target pose
    tgt_gripper_pre_pos = (tgt_gripper_pos[0], tgt_gripper_pos[1] + 0.1, tgt_gripper_pos[2])
    action = tgt_gripper_pre_pos + tgt_gripper_rot
    robot_apply_action(robot, obj_id, action, gripper_action='nop', 
        sim_timestep=sim_timestep, diff_thresh=0.005, max_vel=max_vel)

    # execution step 2 : go to the target pose
    # tgt_gripper_pos = (tgt_gripper_pos[0], tgt_gripper_pos[1] - 0.02, tgt_gripper_pos[2])
    action = tgt_gripper_pos + tgt_gripper_rot
    robot_apply_action(robot, obj_id, action, gripper_action='nop', 
        sim_timestep=sim_timestep, diff_thresh=0.005, max_vel=max_vel)

    # execution step 3 : release the gripper
    robot_apply_action(robot, obj_id, action, gripper_action='pre_grasp', 
        sim_timestep=sim_timestep, diff_thresh=0.005, max_vel=max_vel)

    # execution step 4 : go to the ending pose
    gripper_rot = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[5]
    gripper_rot_matrix = R.from_quat(gripper_rot).as_matrix()
    ending_gripper_pos = np.asarray(tgt_gripper_pos) + (gripper_rot_matrix @ np.array([[0], [0], [-0.2]])).reshape(3)
    action = tuple(ending_gripper_pos) + tgt_gripper_rot
    robot_apply_action(robot, obj_id, action, gripper_action='nop', 
        sim_timestep=sim_timestep, diff_thresh=0.005, max_vel=max_vel)

def rrt_connect_7d(physics_client_id, start_conf, target_conf, hook_trans,
                   obj_pcd = None, obstacle_pcd = None, nce_weight = None, 
                   diagnosis=False, **kwargs):

    # https://github.com/yijiangh/pybullet_planning/blob/dev/src/pybullet_planning/motion_planners/rrt_connect.py

    # config sample location space
    start_pos = start_conf[:3]
    target_pos = target_conf[:3]
    
    low_limit = [0, 0, 0]
    high_limit = [0, 0, 0]
    padding_low  = [0.1, 0.01, 0.02]
    padding_high = [0.0, 0.05, 0.15]

    # xlim, ylim
    for i in range(3):
        if start_pos[i] < target_pos[i]:
            low_limit[i] = start_pos[i] - padding_low[i]
            high_limit[i] = target_pos[i] + padding_high[i] 
        else :
            low_limit[i] = target_pos[i] - padding_low[i]
            high_limit[i] = start_pos[i] + padding_high[i] 
    
    draw_bbox(low_limit, high_limit)

    sample7d_fn = get_sample7d_fn(target_conf, low_limit, high_limit)
    distance7d_fn = get_distance7d_fn()
    extend7d_fn = get_extend7d_fn(resolution=0.001)
    collision_fn = get_collision7d_nce_fn(physics_client_id, obj_pcd, obstacle_pcd, hook_trans, nce_weight)
    # collision_fn = get_collision7d_fn(physics_client_id, obj_id, obstacles=obstacles)

    if not check_initial_end(start_conf, target_conf, collision_fn, diagnosis=diagnosis):
        return None, None

    return birrt(start_conf, target_conf, distance7d_fn, sample7d_fn, extend7d_fn, collision_fn, **kwargs)

# def rrt_connect_7d(physics_client_id, obj_id, start_conf, target_conf, 
#                     obstacles : list = [], obj_pcd = None, obstacle_pcds = [], nce_weight = None, diagnosis=False, **kwargs):

#     # https://github.com/yijiangh/pybullet_planning/blob/dev/src/pybullet_planning/motion_planners/rrt_connect.py

#     # config sample location space
#     start_pos = start_conf[:3]
#     target_pos = target_conf[:3]
    
#     low_limit = [0, 0, 0]
#     high_limit = [0, 0, 0]
#     padding_low  = [0.1, 0.01, 0.02]
#     padding_high = [0.0, 0.05, 0.15]

#     # xlim, ylim
#     for i in range(3):
#         if start_pos[i] < target_pos[i]:
#             low_limit[i] = start_pos[i] - padding_low[i]
#             high_limit[i] = target_pos[i] + padding_high[i] 
#         else :
#             low_limit[i] = target_pos[i] - padding_low[i]
#             high_limit[i] = start_pos[i] + padding_high[i] 
    
#     draw_bbox(low_limit, high_limit)

#     sample7d_fn = get_sample7d_fn(target_conf, low_limit, high_limit)
#     distance7d_fn = get_distance7d_fn()
#     extend7d_fn = get_extend7d_fn(resolution=0.001)
#     # collision_fn = get_collision7d_fn(physics_client_id, obj_id, obstacles=obstacles)
#     collision_fn = get_collision7d_nce_fn(physics_client_id, obj_pcd, obstacle_pcds, nce_weight)

#     if not check_initial_end(start_conf, target_conf, collision_fn, diagnosis=diagnosis):
#         return None

#     return birrt(start_conf, target_conf, distance7d_fn, sample7d_fn, extend7d_fn, collision_fn, **kwargs)


def dist(wpt1 : list or np.ndarray, wpt2 : list or np.ndarray):

    pos_diff = np.asarray(wpt2[:3]) - np.asarray(wpt1[:3])
    tmp_trans = get_matrix_from_pose(wpt1)
    next_trans = get_matrix_from_pose(wpt2)
    diff_trans = np.linalg.inv(tmp_trans) @ next_trans
    diff_6pose = get_pose_from_matrix(diff_trans, 6)
    diff_pos_sum = np.sum(pos_diff ** 2)
    diff_rot_sum = np.sum((diff_6pose[:3] * 180.0 / np.pi * 0.001) ** 2)
    diff_pose_dist = ((diff_pos_sum + diff_rot_sum)) ** 0.5
    diff_pose_dist = np.linalg.norm(pos_diff)
    return diff_pose_dist

def traj_down_sample(traj):

    traj_reverse = traj[::-1]
    sample_dist = 0.00282842712474619 # (2 * 0.2 ** 2) ** 0.5
    
    # copy point cloud to dest
    wpts_reverse = [traj_reverse[0]]
    tmp_diff = 0
    tmp_wpt = traj_reverse[0]
    for wpt_id in range(1, len(traj_reverse)):
        
        tmp_diff += dist(tmp_wpt, traj_reverse[wpt_id])
        if tmp_diff > sample_dist:
            # add first 6d pose
            wpts_reverse.append(list(traj_reverse[wpt_id]))
            tmp_diff = 0
            tmp_wpt = traj_reverse[wpt_id]
    
    return wpts_reverse[::-1]


# def hanging_by_rrt(physics_client_id : int, robot : pandaEnv, 
#                     obj_id : int, target_conf : list or tuple, obstacles : list=[], 
#                     obj_pcd : o3d.geometry.PointCloud = None, obstacle_pcds : o3d.geometry.PointCloud = None, nce_weight = None, 
#                     sim_timestep : float=1.0 / 240.0, max_vel : float=0.2):

def hanging_by_rrt(physics_client_id : int, robot : pandaEnv, 
                    obj_id : int, target_conf : list or tuple, hook_trans=None,
                    obj_pcd : np.ndarray = None, obstacle_pcd : np.ndarray = None, nce_weight = None, 
                    sim_timestep : float=1.0 / 240.0, max_vel : float=0.2):

    obj_pos, obj_rot = p.getBasePositionAndOrientation(obj_id)
    start_conf = obj_pos + obj_rot
    # waypoints, nodes = rrt_connect_7d(physics_client_id, obj_id, start_conf=start_conf, target_conf=target_conf, obstacles=obstacles,
    #                                  obj_pcd=obj_pcd, obstacle_pcds=obstacle_pcds, nce_weight=nce_weight)
    waypoints, nodes = rrt_connect_7d(physics_client_id, 
                                    start_conf=start_conf, target_conf=target_conf, hook_trans=hook_trans,
                                    obj_pcd=obj_pcd, obstacle_pcd=obstacle_pcd, nce_weight=nce_weight)
    
    if waypoints is None:
        print("Oops, no solution!")
        return

    # visualization all nodes
    # for node in nodes:
    #     draw_coordinate(node.config)

    # extract current gripper pose
    gripper_pos = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[4]
    gripper_rot = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[5]
    origin_gripper_pose = get_matrix_from_pos_rot(gripper_pos, gripper_rot)

    origin_obj_pose = get_matrix_from_pos_rot(start_conf[:3], start_conf[3:])

    # relative transform from object to gripper
    # see this stack-overflow issue : https://stackoverflow.com/questions/67001118/relative-transform
    obj2gripper_pose = np.linalg.inv(origin_obj_pose) @ origin_gripper_pose
    planning_resolution = 0.005

    # reset obj pose
    p.resetBasePositionAndOrientation(obj_id, start_conf[:3], start_conf[3:])

    waypoints = traj_down_sample(waypoints)

    imgs = []
    # execution step 1 : execute RRT trajectories
    for i in range(len(waypoints) - 1):

        dense_waypoints = get_dense_waypoints(waypoints[i], waypoints[i+1], planning_resolution)

        # plan trajectory in the same way in collision detection module
        for wpt in dense_waypoints:

            gripper_pose = get_matrix_from_pos_rot(wpt[:3], wpt[3:]) @ obj2gripper_pose
            gripper_pos, gripper_rot = get_pos_rot_from_matrix(gripper_pose)
            gripper_action = np.concatenate((gripper_pos, gripper_rot))

            robot.apply_action(gripper_action)
            p.stepSimulation()
            
            robot.grasp()
            for _ in range(10): # 1 sec
                p.stepSimulation()
                time.sleep(sim_timestep)

            img = p.getCameraImage(480, 480, renderer=p.ER_BULLET_HARDWARE_OPENGL)[2]
            imgs.append(img)
    
    # execution step 2 : release gripper
    robot_apply_action(robot, obj_id, waypoints[-1], gripper_action='pre_grasp', 
        sim_timestep=0.05, diff_thresh=0.01, max_vel=-1, max_iter=100)

    for i in range(int(1/sim_timestep)):
        p.stepSimulation()
        time.sleep(sim_timestep)

    # # execution step 3 : go to the ending pose
    # gripper_rot = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[5]
    # gripper_rot_matrix = R.from_quat(gripper_rot).as_matrix()
    # ending_gripper_pos = np.asarray(waypoints[-1][:3]) + (gripper_rot_matrix @ np.array([[0], [0], [-0.2]])).reshape(3)
    # action = tuple(ending_gripper_pos) + tuple(gripper_rot)
    # robot_apply_action(robot, obj_id, action, gripper_action='nop', 
    #     sim_timestep=0.05, diff_thresh=0.005, max_vel=-1, max_iter=100)
    

    return [Image.fromarray(img) for img in imgs]

def gripper_motion_planning(robot : pandaEnv, tgt_gripper_pos : tuple or list, tgt_gripper_rot : tuple or list, 
                    obstacles : list = [], sim_timestep : float = 1.0 / 240.0, max_vel : float = 0.2):

    # https://github.com/yijiangh/pybullet_planning/blob/dev/src/pybullet_planning/motion_planners/rrt_connect.py

    original_gripper_pos = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[4]
    original_gripper_rot = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[5]

    # num_joints = p.getNumJoints(robot.robot_id)
    target_conf = robot.get_target_joint_conf(list(tgt_gripper_pos) + list(tgt_gripper_rot))
    joints = list(range(0, len(target_conf) - 2)) + [9, 10] # joint 1-7 with gripper [9, 10]
    birrt_res = plan_joint_motion(robot.robot_id, joints, target_conf, obstacles=obstacles, verbose=True)

    if birrt_res is None:
        print("Oops, no solution!")
        return
    
    # draw waypoints
    for step, waypoint in enumerate(birrt_res):
        print(f'{step + 1}/{len(birrt_res)}')
        for i in range(0, len(waypoint)):
            p.resetJointState(robot.robot_id, joints[i], waypoint[i], targetVelocity=0)
        
        gripper_pos = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[4]
        gripper_rot = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[5]
        draw_coordinate(gripper_pos + gripper_rot, size=0.1)

    # reset to original gripper configuration
    original_conf = robot.get_target_joint_conf(list(original_gripper_pos) + list(original_gripper_rot))
    for i in range(0, len(original_conf)):
        p.resetJointState(robot.robot_id, joints[i], original_conf[i], targetVelocity=0.2)
    
    # the acurate way to execute trajectory_controller
    # damn! I'm genuous!
    points = trajectory_controller(robot.robot_id, joints, birrt_res)
    for point in points:
        p.stepSimulation()
        time.sleep(sim_timestep)

    diff_thresh = 0.01
    max_iter = 3000
    for step, waypoint in enumerate(birrt_res):
        print(f'{step + 1}/{len(birrt_res)}')
        for i in range(0, len(waypoint)):
            p.setJointMotorControl2(bodyUniqueId=robot.robot_id,
                                    jointIndex=joints[i],
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=waypoint[i],
                                    maxVelocity=max_vel)
        diff = 1.0
        iter = 0
        while diff > diff_thresh and iter < max_iter:
            iter += 1
            joint_states = p.getJointStates(robot.robot_id, range(0, len(waypoint))) # only jointPosition needed
            joint_poses = [x[0] for x in joint_states]
            diff = np.sum((np.array(joint_poses) - np.array(waypoint)) ** 2)
            print(diff)
            p.stepSimulation()
            time.sleep(sim_timestep)


def main(args):

    # extract args
    max_cnt = args.max_cnt
    input_root = args.input_root

    assert os.path.exists(input_root), f'{input_root} not exists'

    # ================== Model ==================
    
    module = 'nce_package.model.nce'
    model = 'NCE'
    weight_path = 'nce_package/nce_ckpt/collision_detection_sample_single_view/nce_network_epoch-6.pth'
    
    # load model
    nce_network_class = get_model_module(module, model)
    nce_network = nce_network_class({'model.use_xyz': 1}).to('cuda')
    nce_network.load_state_dict(torch.load(weight_path))
    nce_network.eval()

    # ===========================================
    
    # ------------------------ #
    # --- Setup simulation --- #
    # ------------------------ #

    # Create pybullet GUI
    # physics_client_id = p.connect(p.GUI)
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    physics_client_id = p.connect(p.DIRECT)
    # p.resetDebugVisualizerCamera(2.1, 90, -30, [0.0, -0.0, -0.0])
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
    planeId = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
    robot = pandaEnv(physics_client_id, use_IK=1)

    num_joints = p.getNumJoints(robot.robot_id)
    p.stepSimulation()

    # -------------------------- #
    # --- Load other objects --- #
    # -------------------------- #

    table_id = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "table/table.urdf"), [1, 0.0, 0.0])

    # wall
    wall_pos = [0.7, -0.3, 0.8]
    wall_orientation = p.getQuaternionFromEuler([0, 0, 0])
    # wall_id = p.loadURDF("models/wall/wall.urdf", wall_pos, wall_orientation)

    input_jsons = glob.glob(f'{input_root}/*.json')

    for input_json in tqdm(input_jsons):
        if not os.path.exists(input_json):
            print(f'{input_json} not exists')

        json_dict = None
        with open(input_json, 'r') as f:
            json_dict = json.load(f)

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
        obj_id_target, _ = load_obj_urdf(json_dict['obj_path'])

        # ================== Point Cloud ==================

        tree = ET.parse(json_dict['obj_path'])
        root = tree.getroot()
        center = np.array([float(i) for i in root[0].find(
            "inertial").find("origin").attrib['xyz'].split(' ')]).reshape((-1, 1))

        hook_trans = get_matrix_from_pose(json_dict['hook_pose'])
        hook_ply_path = '{}/{}'.format(os.path.split(json_dict['hook_path'])[0], 'base-0.ply')
        hook_pcd = o3d.io.read_point_cloud(hook_ply_path)
        hook_pcd.transform(hook_trans)
        hook_pcd_points = np.asarray(hook_pcd.points)
        hook_pcd_down = do_fps(hook_pcd_points)
        hook_pcd_down_homo = np.hstack((hook_pcd_down - hook_trans[:3, 3], np.ones((hook_pcd_down.shape[0], 1)))).T # from (N x 3) -> (4 x N)

        obj_trans = get_matrix_from_pose(contact_info['obj_pose'])
        obj_ply_path = '{}/{}'.format(os.path.split(json_dict['obj_path'])[0], 'base-0.ply')
        obj_pcd = o3d.io.read_point_cloud(obj_ply_path)
        obj_pcd.translate(-center)
        obj_pcd.transform(obj_trans)
        obj_pcd_points = np.asarray(obj_pcd.points)
        obj_pcd_down = do_fps(obj_pcd_points, 1000)
        obj_pcd_down_homo = np.hstack((obj_pcd_down, np.ones((obj_pcd_down.shape[0], 1)))).T # from (N x 3) -> (4 x N)
        obj_pcd_down_homo = np.linalg.inv(obj_trans) @ obj_pcd_down_homo
        
        p.resetBasePositionAndOrientation(obj_id_target, tgt_obj_pos, tgt_obj_rot)
        collision_fn = get_collision7d_nce_fn(physics_client_id, obj_pcd_down_homo, hook_pcd_down_homo, hook_trans, nce_network)
        tgt_pose = refine_tgt_obj_pose(obj_id_target, collision_fn)
        p.removeBody(obj_id_target)

        # config output path
        output_root = 'nce_package/output'
        output_dir = f'{os.path.split(input_json)[0]}/{os.path.splitext(os.path.split(input_json)[1])[0]}'
        os.makedirs(os.path.join(output_root, output_dir), exist_ok=True)
        output_dir = os.path.join(output_root, output_dir)

        # random position noise
        # low = [-0.005, -0.02, -0.01]
        # high = [0.005, 0.02, 0.03]
        # pos_noise = np.zeros(3)
        # pos_noise = np.random.uniform(low=low, high=high, size=3)
        obj_id, center = load_obj_urdf(json_dict['obj_path'])

        # collision_fn = get_collision7d_fn(physics_client_id, obj_id, obstacles=[hook_id])

        len_init_pose = len(json_dict['initial_pose'])
        # assert len_init_pose <= 3, f'initial poses need to be 3, not {len_init_pose}'
        max_cnt = len_init_pose if args.max_cnt == -1 else args.max_cnt
        assert max_cnt > 0 and max_cnt <= len_init_pose

        for index in range(max_cnt):
            # object initialization
            initial_info = json_dict['initial_pose'][0]
            obj_pos = initial_info['obj_pose'][:3]
            obj_rot = initial_info['obj_pose'][3:]
            # obj_pos = list(np.asarray(obj_pos) + pos_noise)
            # obj_transform = get_matrix_from_pos_rot(obj_pos, obj_rot)

            # grasping
            initial_info = json_dict['initial_pose'][0]
            robot_pos = initial_info['robot_pose'][:3]
            robot_rot = initial_info['robot_pose'][3:]
            # robot_pos = list(np.asarray(robot_pos) + pos_noise)
            robot_pose = robot_pos + robot_rot

            robot.apply_action(robot_pose, max_vel=-1)
            for _ in range(int(1.0 / sim_timestep) * 2): # 1 sec
                p.stepSimulation()
                time.sleep(sim_timestep)
            robot.grasp(obj_id=obj_id)
            for _ in range(int(1.0 / sim_timestep) * 2): # 1 sec
                p.resetBasePositionAndOrientation(obj_id, obj_pos, obj_rot)
                p.stepSimulation()
                time.sleep(sim_timestep)

            # ----------------------------- #
            # --- Setting inital motion --- #
            # ----------------------------- #

            ####################################################

            # run RRT algorithm
            imgs_array = hanging_by_rrt(physics_client_id, robot, 
                                        obj_id, target_conf=tgt_pose, hook_trans=hook_trans,
                                        obj_pcd=obj_pcd_down_homo, obstacle_pcd=hook_pcd_down_homo, nce_weight=nce_network, 
                                        max_vel=0.1)
            # imgs_array = hanging_by_rrt(physics_client_id, robot, obj_id, target_conf=tgt_pose, obstacles=[ table_id, hook_id], max_vel=0.1)
            
            # check success
            
            contact_points = p.getContactPoints(obj_id, hook_id)
            contact = 1 if len(contact_points) > 0 else 0 

            out_line = '[Path]|{}|[Success]|{}\n'.format(input_json.split('/')[-1], 1 if contact else 0)
            print(out_line)
            sys.stdout.flush()
            p.removeAllUserDebugItems()

            # save gif
            if imgs_array is not None:
                status = 'success' if contact else 'failed'
                # init_pose = ['easy', 'medium', 'hard'][index]
                gif_path = os.path.join(output_dir, f'{index}_{status}.gif')
                imgs_array[0].save(gif_path, save_all=True, append_images=imgs_array[1:], duration=50, loop=0)

        p.removeBody(obj_id)
        p.removeBody(hook_id)

    print('process complete...')

# # start message
# start_msg = \
# '''
# ======================================================================================
# this script will execute the robot hanging motion using RRT-Connect algorithm and save 
# the execution process into gif files in 

# dependency :
# - hook folder that contains /[hook_name]/base.urdf
# - root directory of the contact points output
# - the directory of the contact points output contains [output_root]/[hook_name-object_name]/[hook_name-object_name].json
# note :
# - you can run it using ./run.sh hangenv (with predefined [hook_name-object_name])
# ======================================================================================
# '''

# print(start_msg)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--input_root', '-ir', type=str, default='data/data_all_new_testing')
    parser.add_argument('--log_file', '-lf', type=str, default='nce_package/res_1.txt')
    # parser.add_argument('--input_json', '-ij', type=str, default='Hook_hcu_3_hard-everyday_objects/Hook_hcu_3_hard-everyday_objects_50_cooking_11.json')
    parser.add_argument('--id', '-id', type=str, default='0')
    parser.add_argument('--max_cnt', '-mc', type=int, default=1)
    args = parser.parse_args()
    main(args)