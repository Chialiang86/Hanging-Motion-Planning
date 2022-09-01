# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.
import os, inspect
import argparse
import json
import time
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
import pybullet as p
import pybullet_data

# for motion planners
from utils.motion_planning_utils import get_collision7d_fn

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
        obj_id = p.loadURDF(urdf_path, pos, p.getQuaternionFromEuler(rot))
    elif len(rot) == 4:
        obj_id = p.loadURDF(urdf_path, pos, rot)
        
    return obj_id, center

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


def robot_key_callback(robot : pandaEnv, keys : dict, object_id : int=None):

    move_offset = 0.002
    rot_offset = 0.005
    ret = None

    # move up
    if 65297 in keys and keys[65297] & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN): # up arrow
        # quat_1 = p.getQuaternionFromEuler([m.pi, 0, 0])
        tmp_pos = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[4] # position
        new_pos = (tmp_pos[0], tmp_pos[1], tmp_pos[2] + move_offset)
        robot.apply_action(new_pos)
        ret = 'up'
    # move down
    elif 65298 in keys and keys[65298] & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN): # down arrow
        tmp_pos = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[4] # position
        new_pos = (tmp_pos[0], tmp_pos[1], tmp_pos[2] - move_offset)
        robot.apply_action(new_pos)
        ret = 'down'
    # move left
    elif 65295 in keys and keys[65295] & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN):  # left arrow
        tmp_pos = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[4] # position
        new_pos = (tmp_pos[0], tmp_pos[1] - move_offset, tmp_pos[2])
        robot.apply_action(new_pos)
        ret = 'left'
    # move right
    elif 65296 in keys and keys[65296] & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN): # right arrow
        tmp_pos = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[4] # position
        new_pos = (tmp_pos[0], tmp_pos[1] + move_offset, tmp_pos[2])
        robot.apply_action(new_pos)
        ret = 'right'
    # move front
    elif ord('x') in keys and keys[ord('x')] & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN): 
        tmp_pos = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[4] # position
        new_pos = (tmp_pos[0] + move_offset, tmp_pos[1], tmp_pos[2])
        robot.apply_action(new_pos)
        ret = 'front'
    # move back
    elif ord('d') in keys and keys[ord('d')] & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN): 
        tmp_pos = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[4] # position
        new_pos = (tmp_pos[0] - move_offset, tmp_pos[1], tmp_pos[2])
        robot.apply_action(new_pos)
        ret = 'back'
    # flip flop gripper
    elif 32 in keys and keys[32] & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN): 
        gripper_pos = robot.get_gripper_pos()
        robot.grasp(object_id)
        if gripper_pos[0] < 0.02 or gripper_pos[1] < 0.02:
            robot.pre_grasp()
            for _ in range(100):
                p.stepSimulation()
                time.sleep(0.005)
            ret = 'open'
        else :
            robot.grasp(object_id)
            for _ in range(100):
                p.stepSimulation()
                time.sleep(0.005)
            ret = 'close'
    # rotate gripper : counter clockwise
    elif ord('z') in keys and keys[ord('z')] & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN): 
        joint_name_ids = robot.get_joint_name_ids()
        tmp_pos = p.getJointState(robot.robot_id, joint_name_ids['panda_joint7'])[0]
        p.setJointMotorControlArray(robot.robot_id, [joint_name_ids['panda_joint7']], p.POSITION_CONTROL, targetPositions=[tmp_pos-rot_offset])
        ret = 'turn_cw'
    # flip flop gripper : clockwise
    elif ord('c') in keys and keys[ord('c')] & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN): 
        joint_name_ids = robot.get_joint_name_ids()
        tmp_pos = p.getJointState(robot.robot_id, joint_name_ids['panda_joint7'])[0]
        p.setJointMotorControlArray(robot.robot_id, [joint_name_ids['panda_joint7']], p.POSITION_CONTROL, targetPositions=[tmp_pos+rot_offset])
        ret = 'turn_ccw'
    elif ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
        ret = 'quit'
    return ret


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
        cameraYaw=120,
        cameraPitch=0,
        cameraTargetPosition=[0.7, 0.0, 1.3]
    )
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=150)
    sim_timestep = 1.0 / 240
    p.setTimeStep(sim_timestep)

    # Load plane contained in pybullet_data
    planeId = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))

    # Set gravity for simulation
    p.setGravity(0, 0, -9.8)

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
    # tgt_pose = refine_tgt_obj_pose(physics_client_id, obj_id_target, obstacles=[hook_id, planeId])
    # print(tgt_pose)

    # grasping
    # robot.apply_action(contact_info['object_pose'])
    sim_timestep = 1.0 / 240.0
    p.setTimeStep(sim_timestep)

    # if manual contral needed
    if args.control:
        param_ids = []
        joint_ids = []
        num_joints = p.getNumJoints(robot.robot_id)

        joint_states = p.getJointStates(robot.robot_id, range(0, num_joints))
        joint_poses = [x[0] for x in joint_states]
        idx = 0
        for i in range(num_joints):
            joint_info = p.getJointInfo(robot.robot_id, i)
            joint_name = joint_info[1]
            joint_type = joint_info[2]
            print(joint_name)

            if joint_type is p.JOINT_REVOLUTE or joint_type is p.JOINT_PRISMATIC:
                joint_ids.append(i)
                param_ids.append(
                    p.addUserDebugParameter(joint_name.decode("utf-8"), joint_info[8], joint_info[9], joint_poses[i]))
                idx += 1

        close_flag = False
        param_control = True
        while True:
            
            # key callback
            keys = p.getKeyboardEvents()
            if keys != {} :
                
                if ord('p') in keys and keys[ord('p')] & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN): 

                    gripper_pos = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[4]
                    gripper_rot = p.getLinkState(robot.robot_id, robot.end_eff_idx, physicsClientId=robot._physics_client_id)[5]
                    gripper_pose_shift = list(np.array(gripper_pos) + np.array([0.0, 0.0, 0.0])) + list(gripper_rot)
                    # gripper_pose_shift = list(np.array(gripper_pos) + np.array([0.0, 0.15, 0.1])) + list(gripper_rot)

                    obj_pos, obj_rot = p.getBasePositionAndOrientation(obj_id_target)
                    obj_pose_shift = list(np.array(obj_pos) + np.array([0.0, 0.0, 0.0])) + list(obj_rot)
                    # obj_pose_shift = list(np.array(obj_pos) + np.array([0.0, 0.15, 0.1])) + list(obj_rot)

                    # refine initial pose of gripper and object
                    json_dict['initial_pose'] = []
                    initial_pose_ele = {
                        'robot_pose': gripper_pose_shift,
                        'object_pose': obj_pose_shift
                    }
                    json_dict['initial_pose'].append(initial_pose_ele)
                    print(json_dict['initial_pose'])

                    with open(input_json, 'w') as f:
                        print(f'append to {input_json}')
                        json_dict = json.dump(json_dict, f, indent=4)

                    break

                elif ord('o') in keys and keys[ord('o')] & (p.KEY_WAS_TRIGGERED): 

                    p.removeBody(hook_id)

                elif ord('a') in keys and keys[ord('a')] & (p.KEY_WAS_TRIGGERED): 
                    
                    param_control = False if param_control == True else True
                    param_ids = update_debug_param(robot)

                else:
                    action = robot_key_callback(robot, keys, obj_id_target)
                    if action == 'quit':
                        break
                    elif action == 'close':
                        close_flag = True
                    elif action == 'open':
                        close_flag = False
                    
                    if close_flag:
                        robot.grasp(obj_id_target)
                        for _ in range(5):
                            p.stepSimulation()

                # param_ids = update_debug_param(robot)

            elif param_control:
                new_pos = []
                for i in param_ids:
                    new_pos.append(p.readUserDebugParameter(i))
                p.setJointMotorControlArray(robot.robot_id, joint_ids, p.POSITION_CONTROL, targetPositions=new_pos)

            p.stepSimulation()
            time.sleep(sim_timestep)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-json', '-ij', type=str, default='data/Hook_90-hanging_exp/Hook_90-hanging_exp_wrench_1.json')
    # parser.add_argument('--input-json', '-ij', type=str, default='data/Hook_60-mug/Hook_60-mug_19.json')
    parser.add_argument('--method', '-m', type=str, default='birrt')
    parser.add_argument('--control', '-c', action='store_true', default=True)
    args = parser.parse_args()
    main(args)
