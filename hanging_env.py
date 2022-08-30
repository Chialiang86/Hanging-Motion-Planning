# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.
import enum
import os, inspect
import argparse
import json
import math as m
from tqdm import tqdm
import time
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
import pybullet as p
import pybullet_data

# for motion planners
from utils.motion_planning_utils import get_sample7d_fn, get_distance7d_fn, get_extend7d_fn, get_collision7d_fn
from pybullet_planning.interfaces.planner_interface.joint_motion_planning import plan_joint_motion, check_initial_end
from pybullet_planning.motion_planners.rrt_connect import birrt

# for robot control
# from pybullet_planning.interfaces.robots.joint import get_custom_limits
from pybullet_robot_envs.envs.panda_envs.panda_env import pandaEnv
from pybullet_planning.interfaces.control.control import trajectory_controller

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
        obj_id = p.loadURDF(urdf_path, pos, p.getQuaternionFromEuler(rot))
    elif len(rot) == 4:
        obj_id = p.loadURDF(urdf_path, pos, rot)
        
    return obj_id, center

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

def get_matrix_from_pos_rot(pos : list or tuple, rot : list or tuple):
    assert (len(pos) == 3 and len(rot) == 4) or (len(pos) == 3 and len(rot) == 3)
    pos_m = np.asarray(pos)
    if len(rot) == 3:
        rot_m = np.asarray(p.getMatrixFromQuaternion(p.getQuaternionFromEuler(rot))).reshape((3, 3))
    elif len(rot) == 4:
        rot_m = np.asarray(p.getMatrixFromQuaternion(rot)).reshape((3, 3))
    ret_m = np.identity(4)
    ret_m[:3, :3] = rot_m
    ret_m[:3, 3] = pos_m
    return ret_m

def get_pos_rot_from_matrix(pose : np.ndarray):
    assert pose.shape == (4, 4)
    pos = pose[:3, 3]
    rot = R.from_matrix(pose[:3, :3]).as_quat()
    return pos, rot

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
        for _ in range(int(1.0 / sim_timestep) * 2): # 1 sec
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

def rrt_connect_7d(physics_client_id, obj_id, start_conf, target_conf, 
                    obstacles : list = [], sim_timestep : float = 1.0 / 240.0, max_vel : float = 0.2, diagnosis=False, **kwargs):

    # https://github.com/yijiangh/pybullet_planning/blob/dev/src/pybullet_planning/motion_planners/rrt_connect.py

    sample7d_fn = get_sample7d_fn(target_conf)
    distance7d_fn = get_distance7d_fn()
    extend7d_fn = get_extend7d_fn(resolution=0.01)
    collision_fn = get_collision7d_fn(physics_client_id, obj_id, obstacles=obstacles)

    if not check_initial_end(start_conf, target_conf, collision_fn, diagnosis=diagnosis):
        return None

    return birrt(start_conf, target_conf, distance7d_fn, sample7d_fn, extend7d_fn, collision_fn, **kwargs)

def hanging_by_rrt(physics_client_id : int, robot : pandaEnv, obj_id : int, target_conf : list or tuple, 
                    obstacles : list=[], sim_timestep : float=1.0 / 240.0, max_vel : float=0.2):

    obj_pos, obj_rot = p.getBasePositionAndOrientation(obj_id)
    start_conf = obj_pos + obj_rot
    waypoints = rrt_connect_7d(physics_client_id, obj_id, start_conf=start_conf, target_conf=target_conf, obstacles=obstacles)
    
    if waypoints is None:
        print("Oops, no solution!")
        return

    # extract current gripper pose
    gripper_pos = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[4]
    gripper_rot = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[5]
    origin_gripper_pose = get_matrix_from_pos_rot(gripper_pos, gripper_rot)

    origin_obj_pose = get_matrix_from_pos_rot(start_conf[:3], start_conf[3:])

    # relative transform from object to gripper
    # see this stack-overflow issue : https://stackoverflow.com/questions/67001118/relative-transform
    obj2gripper_pose = np.linalg.inv(origin_obj_pose) @ origin_gripper_pose
    planning_resolution = 0.05

    # reset obj pose
    p.resetBasePositionAndOrientation(obj_id, start_conf[:3], start_conf[3:])

    for i in range(len(waypoints) - 1):

        q1_pos = np.asarray(waypoints[i][:3])
        q2_pos = np.asarray(waypoints[i+1][:3])
        q1_rot = np.asarray(waypoints[i][3:])
        q2_rot = np.asarray(waypoints[i+1][3:])

        d12 = q2_pos - q1_pos
        r12_rotvec = R.from_quat(q2_rot).as_rotvec() - R.from_quat(q1_rot).as_rotvec()

        diff_q1_q2 = np.concatenate((d12, r12_rotvec))
        steps = int(np.ceil(np.linalg.norm(np.divide(diff_q1_q2, planning_resolution), ord=2)))
        steps = 10

        # plan trajectory in the same way in collision detection module
        for i in range(steps):
            positions6d = (i + 1) / steps * diff_q1_q2 + np.concatenate((q1_pos, R.from_quat(q1_rot).as_rotvec()))
            positions7d = tuple(positions6d[:3]) + tuple(R.from_rotvec(positions6d[3:]).as_quat())
            draw_coordinate(positions7d)
            # p.resetBasePositionAndOrientation(obj_id, positions7d[:3], positions7d[3:])

            gripper_pose = get_matrix_from_pos_rot(positions7d[:3], positions7d[3:]) @ obj2gripper_pose
            gripper_pos, gripper_rot = get_pos_rot_from_matrix(gripper_pose)
            gripper_action = np.concatenate((gripper_pos, gripper_rot))

            robot_apply_action(robot, obj_id, gripper_action, gripper_action='nop', 
                sim_timestep=0.05, diff_thresh=0.05, max_vel=-1, max_iter=100)
            
            robot.grasp()
            for _ in range(10): # 1 sec
                p.stepSimulation()

    robot_apply_action(robot, obj_id, waypoints[-1], gripper_action='pre_grasp', 
        sim_timestep=0.05, diff_thresh=0.01, max_vel=-1)

    # execution step 4 : go to the ending pose
    gripper_rot = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[5]
    gripper_rot_matrix = R.from_quat(gripper_rot).as_matrix()
    ending_gripper_pos = np.asarray(waypoints[-1][:3]) + (gripper_rot_matrix @ np.array([[0], [0], [-0.2]])).reshape(3)
    action = tuple(ending_gripper_pos) + tuple(gripper_rot)
    robot_apply_action(robot, obj_id, action, gripper_action='nop', 
        sim_timestep=0.05, diff_thresh=0.005, max_vel=-1)

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

def draw_coordinate(pose : np.ndarray or tuple or list, size : float = 0.02):
    assert (type(pose) == np.ndarray and pose.shape == (4, 4)) or (type(pose) == tuple and len(pose) == 7) or (type(pose) == list and len(pose) == 7)

    if type(pose) == tuple or type(pose) == list:
        pose = get_matrix_from_pos_rot(pose[:3], pose[3:])

    origin = pose[:3, 3]
    x = origin + pose[:3, 0] * size
    y = origin + pose[:3, 1] * size
    z = origin + pose[:3, 2] * size
    p.addUserDebugLine(origin, x, [1, 0, 0])
    p.addUserDebugLine(origin, y, [0, 1, 0])
    p.addUserDebugLine(origin, z, [0, 0, 1])


def main(args):

    # extract args
    input_json = args.input_json
    if not os.path.exists(input_json):
        print(f'{input_json} not exists')

    json_dict = None
    with open(input_json, 'r') as f:
        json_dict = json.load(f)
    
    # ------------------------ #
    # --- Setup simulation --- #
    # ------------------------ #

    # Create pybullet GUI
    physics_client_id = p.connect(p.GUI)
    # p.resetDebugVisualizerCamera(2.1, 90, -30, [0.0, -0.0, -0.0])
    p.resetDebugVisualizerCamera(
        cameraDistance=0.5,
        cameraYaw=180,
        cameraPitch=0,
        cameraTargetPosition=[0.7, 0.0, 1.3]
    )
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=150)
    sim_timestep = 1.0 / 240
    p.setTimeStep(sim_timestep)

    # Set gravity for simulation
    p.setGravity(0, 0, -9.8)

    # Load plane contained in pybullet_data
    planeId = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))

    # ------------------- #
    # --- Setup robot --- #
    # ------------------- #

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
    
    # hook initialization
    hook_pos = json_dict['hook_pose'][:3]
    hook_orientation = json_dict['hook_pose'][3:]
    hook_id = p.loadURDF(json_dict['hook_path'], hook_pos, hook_orientation)

    # get target hanging pose
    assert len(json_dict['contact_info']) > 0, 'contact info is empty'
    index = 0
    contact_info = json_dict['contact_info'][index]
    tgt_obj_pos = contact_info['object_pose'][:3]
    tgt_obj_rot = contact_info['object_pose'][3:]
    obj_id_target, _ = load_obj_urdf(json_dict['obj_path'])
    p.resetBasePositionAndOrientation(obj_id_target, tgt_obj_pos, tgt_obj_rot)
    tgt_pose = refine_tgt_obj_pose(physics_client_id, obj_id_target, obstacles=[hook_id, planeId])
    print(tgt_pose)
    p.removeBody(obj_id_target)

    # object initialization
    initial_info = json_dict['initial_pose'][index]
    obj_pos = initial_info['object_pose'][:3]
    obj_rot = initial_info['object_pose'][3:]
    obj_id, center = load_obj_urdf(json_dict['obj_path'])
    # obj_transform = get_matrix_from_pos_rot(obj_pos, obj_rot)

    # grasping
    initial_info = json_dict['initial_pose'][index]
    robot_pos = initial_info['robot_pose'][:3]
    robot_rot = initial_info['robot_pose'][3:]
    # robot_transform = get_matrix_from_pos_rot(robot_pos, robot_rot)
    robot_pose = robot_pos + robot_rot

    robot.apply_action(robot_pose, max_vel=-1)
    for _ in range(int(1.0 / sim_timestep) * 2): # 1 sec
        p.stepSimulation()
        time.sleep(sim_timestep)
    robot.grasp(obj_id=obj_id)
    for _ in range(int(1.0 / sim_timestep) * 3): # 1 sec
        p.resetBasePositionAndOrientation(obj_id, obj_pos, obj_rot)
        p.stepSimulation()
        time.sleep(sim_timestep)

    # ----------------------------- #
    # --- Setting inital motion --- #
    # ----------------------------- #

    # pre_grasp_pos = [grasp_x_offset, 0.0, 0.9]
    # pre_grasp_rot = p.getQuaternionFromEuler([np.pi, 0, 0])
    # pre_grasp_action = pre_grasp_pos + list(pre_grasp_rot)
    # # robot_apply_action(robot, obj_id, pre_grasp_action, gripper_action='pre_grasp', 
    # #     sim_timestep=sim_timestep, diff_thresh=0.05, max_vel=0.5)
    # # robot_apply_action(robot, obj_id, pre_grasp_action, gripper_action='nop', 
    # #     sim_timestep=sim_timestep, diff_thresh=0.05, max_vel=-1)
    # pre_grasp_pos = tgt_pose[:3]
    # pre_grasp_rot = p.getQuaternionFromEuler([np.pi, 0, 0])
    # pre_grasp_action = list(pre_grasp_pos) + list(pre_grasp_rot)
    # robot_apply_action(robot, obj_id, pre_grasp_action, gripper_action='pre_grasp', 
    #     sim_timestep=sim_timestep, diff_thresh=0.05, max_vel=0.5)
    # robot_apply_action(robot, obj_id, pre_grasp_action, gripper_action='nop', 
    #     sim_timestep=sim_timestep, diff_thresh=0.05, max_vel=0.5)

    # # go down to the object
    # grasp_pos = [grasp_x_offset, 0.0, 0.70]
    # grasp_rot = p.getQuaternionFromEuler([np.pi, 0, 0])
    # grasp_action = grasp_pos + list(grasp_rot)
    # robot_apply_action(robot, obj_id, grasp_action, gripper_action='nop', 
    #     sim_timestep=sim_timestep, diff_thresh=0.05, max_vel=0.5)

    # # grasping
    # robot_apply_action(robot, obj_id, None, gripper_action='grasp', 
    #     sim_timestep=sim_timestep, diff_thresh=0.05, max_vel=0.5)

    # # move up slightly
    # after_grasp_pos = [grasp_x_offset, -0.1, 1.1]
    # after_grasp_rot = p.getQuaternionFromEuler([np.pi, 0, 0])
    # after_grasp_action = after_grasp_pos + list(after_grasp_rot)
    # robot_apply_action(robot, obj_id, after_grasp_action, gripper_action='nop', 
    #     sim_timestep=sim_timestep, diff_thresh=0.05, max_vel=0.2)
    # draw_coordinate(after_grasp_action)

    # ------------------------------ #
    # --- Setting hanging motion --- #
    # ------------------------------ #
    if args.method == 'ik':
        # todo : modify input to target_conf
        target_conf = tgt_obj_pos + tgt_obj_rot
        hanging_by_ik(robot, obj_id, target_conf=target_conf)
    elif args.method == 'birrt':
        target_conf = tgt_obj_pos + tgt_obj_rot
        hanging_by_rrt(physics_client_id, robot, obj_id, target_conf=tgt_pose, obstacles=[ table_id, hook_id], max_vel=0.1)
    
    
    print('process complete')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-json', '-ij', type=str, default='data/Hook_60-hanging_exp/Hook_60-hanging_exp_scissor_4.json')
    # parser.add_argument('--input-json', '-ij', type=str, default='data/Hook_60-mug/Hook_60-mug_19.json')
    parser.add_argument('--method', '-m', type=str, default='birrt')
    parser.add_argument('--control', '-c', action='store_true', default=True)
    args = parser.parse_args()
    main(args)