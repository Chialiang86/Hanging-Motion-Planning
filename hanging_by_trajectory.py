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

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print(currentdir)
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)


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

def get_pos_rot_from_matrix(pose : np.ndarray, format='quat'):
    assert pose.shape == (4, 4)
    assert format in ['quat', 'rotvec']
    pos = pose[:3, 3]
    if format == 'quat':
        rot = R.from_matrix(pose[:3, :3]).as_quat()
    if format == 'rotvec':
        rot = R.from_matrix(pose[:3, :3]).as_rotvec()
    return pos, rot

def draw_coordinate(pose : np.ndarray or tuple or list, size : float = 0.05):
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

    src_rotvec = R.from_matrix(src_rot).as_rotvec()
    tgt_rotvec = R.from_matrix(tgt_rot).as_rotvec()

    rot_180 = np.identity(4)
    rot_180[:3, :3] = R.from_rotvec([0, 0, np.pi]).as_matrix()
    tgt_dual_transform = tgt_transform @ rot_180
    tgt_dual_rotvec = R.from_matrix(tgt_dual_transform[:3, :3]).as_rotvec()

    return tgt_transform if np.sum((src_rotvec - tgt_rotvec) ** 2) < np.sum((src_rotvec - tgt_dual_rotvec) ** 2) else tgt_dual_transform

def capture_image(view_mat, proj_matrix, far, near, width=320, height=240):

    img = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view_mat,
        projectionMatrix=proj_matrix
    )

    rgbBuffer = np.reshape(img[2], (height, width, 4))[:,:,:3]
    depthBuffer = np.reshape(img[3], [height, width])
    depthBuffer = far * near / (far - (far - near) * depthBuffer)

    return rgbBuffer, depthBuffer

def main(args):
    
    # ------------------------ #
    # --- Setup simulation --- #
    # ------------------------ #

    # Create pybullet GUI
    physics_client_id = p.connect(p.GUI)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.1,
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
    # p.stepSimulation()

    # -------------------------- #
    # --- Load other objects --- #
    # -------------------------- #

    p.loadURDF(os.path.join(pybullet_data.getDataPath(), "table/table.urdf"), [1, 0.0, 0.0])

    # load model

    root = f'keypoint_trajectory_{args.dir_postfix}'
    demonstration_dir = f'demonstration_data_{args.dir_postfix}'
    obj_fname = f'{root}/{args.obj}'
    hook_fname = f'{root}/{args.hook}'
    obj_name = os.path.split(obj_fname)[1].split('.')[0]
    hook_name = os.path.split(hook_fname)[1].split('.')[0]
    # TODO : need to add postfix to data/ ?
    obj_hook_pair_fname = f'data/Hook_bar-hanging_exp/Hook_bar-{obj_name}.json'
    print(f'hook_name: {hook_name}, obj_name: {obj_name}, obj_hook_pair_fname: {obj_hook_pair_fname}')

    assert os.path.exists(obj_hook_pair_fname), f'{obj_hook_pair_fname} not exists'
    assert os.path.exists(obj_fname), f'{obj_fname} not exists'
    assert os.path.exists(hook_fname), f'{hook_fname} not exists'

    with open(obj_hook_pair_fname, 'r') as f:
        obj_hook_pair_dict = json.load(f)
    with open(obj_fname, 'r') as f:
        obj_dict = json.load(f)
    with open(hook_fname, 'r') as f:
        hook_dict = json.load(f)
    
    if not os.path.exists(demonstration_dir):
        os.mkdir(demonstration_dir)

    # obj_pose_6d = obj_dict['obj_pose']
    # obj_pos = obj_pose_6d[:3]
    # obj_rot = obj_pose_6d[3:]
    obj_contact_pose_6d = obj_dict['contact_pose']
    obj_contact_relative_transform = get_matrix_from_pos_rot(obj_contact_pose_6d[:3], obj_contact_pose_6d[3:])
    obj_id = p.loadURDF(obj_dict['file'])
    # p.resetBasePositionAndOrientation(obj_id, obj_pos, obj_rot)

    hook_id = p.loadURDF(hook_dict['file'])
    hook_pose_6d = hook_dict['hook_pose']
    hook_pos = hook_pose_6d[:3]
    hook_quat = hook_pose_6d[3:]
    hook_transform = get_matrix_from_pos_rot(hook_pos, hook_quat)
    p.resetBasePositionAndOrientation(hook_id, hook_pos, hook_quat)
    trajectories_hook = hook_dict['trajectory']

    # grasping
    index = 0 # medium
    initial_info = obj_hook_pair_dict['initial_pose'][index] # medium
    obj_pos = initial_info['object_pose'][:3]
    obj_rot = initial_info['object_pose'][3:]

    initial_info = obj_hook_pair_dict['initial_pose'][index] # medium
    robot_pos = initial_info['robot_pose'][:3]
    robot_rot = initial_info['robot_pose'][3:]
    robot_pose = robot_pos + robot_rot
    robot_transform = get_matrix_from_pos_rot(robot_pos, robot_rot)

    # rendering
    far = 1.
    near = 0.01
    fov = 90.
    aspect_ratio = 1.
    cameraEyePosition=[0.75, 0.1, 1.3]
    cameraTargetPosition=[0.5, 0.1, 1.3]
    cameraUpVector=[0.0, 0.0, 1.0]
    view_mat = p.computeViewMatrix(
        cameraEyePosition=cameraEyePosition,
        cameraTargetPosition=cameraTargetPosition,
        cameraUpVector=cameraUpVector,
    )
    proj_matrix = p.computeProjectionMatrixFOV(
        fov, aspect_ratio, near, far
    )

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

        obj_transform = get_matrix_from_pos_rot(obj_pos, obj_rot)
        kpt_transform_world = obj_transform @ obj_contact_relative_transform
        
        trajectory_hook = trajectories_hook[traj_i]
        first_waypoint = trajectory_hook[0]
        relative_kpt_transform = get_matrix_from_pos_rot(first_waypoint[:3], first_waypoint[3:])
        first_kpt_transform_world = hook_transform @ relative_kpt_transform

        kpt_transform_world = refine_rotation(first_kpt_transform_world, kpt_transform_world)

        kpt_to_gripper = np.linalg.inv(kpt_transform_world) @ robot_transform
        first_gripper_transform = first_kpt_transform_world @ kpt_to_gripper

        first_gripper_pos, first_gripper_rot = get_pos_rot_from_matrix(first_gripper_transform)
        first_gripper_pose = list(first_gripper_pos) + list(first_gripper_rot)

        # draw_coordinate(kpt_transform_world)
        # draw_coordinate(first_kpt_transform_world)

        trajectory_start = get_dense_waypoints(robot_pose, first_gripper_pose)
        for waypoint in trajectory_start:
            robot.apply_action(waypoint)
            p.stepSimulation()
            robot.grasp()
            for _ in range(10): # 1 sec
                p.stepSimulation()
                time.sleep(sim_timestep)
        
        imgs = []
        demonstration_list = []

        gripper_pose = None
        previous_transform = np.identity(4)
        motion_frequency = 3
        for i, waypoint in enumerate(trajectory_hook):
            if i % motion_frequency == 0:
                
                relative_transform = get_matrix_from_pos_rot(waypoint[:3], waypoint[3:])
                world_transform = hook_transform @ relative_transform
                gripper_transform = world_transform @ kpt_to_gripper
                gripper_pos, gripper_rot = get_pos_rot_from_matrix(gripper_transform)
                gripper_pose = list(gripper_pos) + list(gripper_rot)

                # draw_coordinate(world_transform)

                robot.apply_action(gripper_pose)
                p.stepSimulation()
                
                robot.grasp()
                for _ in range(10): # 1 sec
                    p.stepSimulation()
                    time.sleep(sim_timestep)
                time.sleep(sim_timestep)

                if i > 0:
                    relative_pos, relative_rot = get_pos_rot_from_matrix(np.linalg.inv(previous_transform) @ gripper_transform)
                    relative_action = list(relative_pos) + list(R.from_quat(relative_rot).as_rotvec())
                else :
                    relative_action = [0, 0, 0, 0, 0, 0]

                t = np.identity(4)
                t[:3, 3] = relative_action[:3]
                t[:3, :3] = R.from_rotvec(relative_action[3:]).as_matrix()
                # draw_coordinate(previous_transform @ t)
                previous_transform = gripper_transform
                
                # capture RGBD
                rgb, depth = capture_image(view_mat, proj_matrix, far, near)
                demonstration_list.append({
                    'rgb': rgb,
                    'depth': depth,
                    'action': relative_action
                })

                img = p.getCameraImage(480, 480, renderer=p.ER_BULLET_HARDWARE_OPENGL)[2]
                imgs.append(img)

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

        imgs_array = [Image.fromarray(img) for img in imgs]

        contact = False
        contact_points = p.getContactPoints(obj_id, hook_id)
        contact = True if contact_points != () else False

        # save gif
        output_fname = f'{root}/gif'
        status = 'success' if contact else 'failed'
        if imgs_array is not None and status=='failed':
            gif_path = os.path.join(output_fname, f'{hook_name}-{obj_name}_{traj_i}_{status}.gif')
            imgs_array[0].save(gif_path, save_all=True, append_images=imgs_array[1:], duration=50, loop=0)
        
        if status=='success':
            output_dir = f'{demonstration_dir}/{hook_name}-{traj_i}-{obj_name}'
            os.makedirs(output_dir, exist_ok=True)
            action_dict = {
                'action':[],
                'cameraEyePosition':cameraEyePosition,
                'cameraTargetPosition':cameraTargetPosition,
                'cameraUpVector':cameraUpVector,
                'far': far,
                'near': near,
                'fov': fov,
                'aspect_ratio': aspect_ratio,
            }
            for i, data in enumerate(demonstration_list):
                rgb_fname = f'{output_dir}/{i}.jpg'
                depth_fname = f'{output_dir}/{i}.npy'
                Image.fromarray(data['rgb']).save(rgb_fname)
                np.save(depth_fname, data['depth'])
                action_dict['action'].append(data['action'])

            action_fname = f'{output_dir}/action.json'
            action_f = open(action_fname, 'w')
            json.dump(action_dict, action_f, indent=4)

        with open(f"{root}/result.txt", "a") as myfile:
            print(f'{hook_name}-{obj_name}_{traj_i}_{status}\n')
            myfile.write(f'{hook_name}-{obj_name}_{traj_i}_{status}\n')
            myfile.flush()
            myfile.close()

    # while True:
    #     # key callback
    #     keys = p.getKeyboardEvents()            
    #     if ord('q') in keys and keys[ord('q')] & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN): 
    #         break

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir-postfix', '-dp', type=str, default='1118-1')
    parser.add_argument('--obj', '--obj', type=str, default='hanging_exp_daily_5.json')
    parser.add_argument('--hook', '--hook', type=str, default='Hook57#9_aug.json')
    args = parser.parse_args()
    main(args)