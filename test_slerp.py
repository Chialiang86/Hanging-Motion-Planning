import os, time
import numpy as np
import argparse
import open3d as o3d
import pybullet as p
import pybullet_data
import quaternion
import json

from scipy.spatial.transform import Rotation as R
from sklearn.neighbors import NearestNeighbors


def get_matrix_from_pos_rot(pos : list or tuple, rot : list or tuple):
    assert (len(pos) == 3 and len(rot) == 4) or (len(pos) == 3 and len(rot) == 3)
    pos_m = np.asarray(pos)
    if len(rot) == 3:
        rot_m = R.from_rotvec(rot).as_matrix()
    elif len(rot) == 4:
        rot_m = R.from_quat(rot).as_matrix()
    ret_m = np.identity(4)
    ret_m[:3, :3] = rot_m
    ret_m[:3, 3] = pos_m
    return ret_m

def HCE(obj_pcd : o3d.geometry.PointCloud, obstacle_pcd : o3d.geometry.PointCloud, thresh : float = 0.0):

    obj_pcd_down = obj_pcd.voxel_down_sample(voxel_size=0.002)
    obstacle_pcd_down = obstacle_pcd.voxel_down_sample(voxel_size=0.002)
    obj_points = np.asarray(obj_pcd_down.points)
    obstacle_points = np.asarray(obstacle_pcd_down.points)
    obstacle_normals = np.asarray(obstacle_pcd_down.normals)

    # obstacle_to_obj
    neigh = NearestNeighbors(n_neighbors=1, algorithm='kd_tree')
    neigh.fit(obstacle_points)
    distances, indices = neigh.kneighbors(obj_points)
    distances = distances.squeeze()
    indices = indices.squeeze()

    obstacle_to_obj = obj_points - obstacle_points[indices]
    obstacle_to_obj = (obstacle_to_obj.T / np.linalg.norm(obstacle_to_obj, ord=2, axis=1)).T 
    ret_obstacle_to_obj = np.sum(obstacle_to_obj * obstacle_normals[indices])

    ratio = ret_obstacle_to_obj / obj_points.shape[0]

    return ratio < thresh

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

def xyzw2wxyz(quat : np.ndarray):
    assert len(quat) == 4, f'quaternion size must be 4, got {len(quat)}'
    return np.asarray([quat[3], quat[0], quat[1], quat[2]])

def wxyz2xyzw(quat : np.ndarray):
    assert len(quat) == 4, f'quaternion size must be 4, got {len(quat)}'
    return np.asarray([quat[1], quat[2], quat[3], quat[0]])

def main():
    data_path = 'data/Hook_60-hanging_exp/Hook_60-hanging_exp_bag_5.json'

    if not os.path.exists(data_path):
        print(f'{data_path} not exists')
        return

    data = None
    with open(data_path, 'r') as f:
        data = json.load(f)
    
    obj_urdf = data['obj_path']
    hook_urdf = data['hook_path']
    obj_tgt_pose = data['contact_info'][0]['object_pose']
    obj_init_pose = data['initial_pose'][2]['object_pose']


    # Create pybullet GUI
    physics_client_id = p.connect(p.GUI)
    # p.resetDebugVisualizerCamera(2.1, 90, -30, [0.0, -0.0, -0.0])
    p.resetDebugVisualizerCamera(
        cameraDistance=0.3,
        cameraYaw=90,
        cameraPitch=-30,
        cameraTargetPosition=[0.5, 0.0, 1.3]
    )
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=150)
    sim_timestep = 1.0 / 240
    p.setTimeStep(sim_timestep)

    hook_pose = data['hook_pose']
    hook_id = p.loadURDF(hook_urdf, hook_pose[:3], hook_pose[3:])

    obj_id = p.loadURDF(obj_urdf)
    random_rotvec = np.random.uniform(low=[-np.pi, -np.pi, -np.pi], high=[np.pi, np.pi, np.pi], size=3)
    random_quat = list(R.from_rotvec(random_rotvec).as_quat())
    obj_init_pose = obj_init_pose[:3] + random_quat
    p.resetBasePositionAndOrientation(obj_id, obj_init_pose[:3], obj_init_pose[3:])

    planning_resolution = 0.005
    d12 = np.asarray(obj_tgt_pose[:3]) - np.asarray(obj_init_pose[:3])
    steps = int(np.ceil(np.linalg.norm(np.divide(d12, planning_resolution), ord=2)))
    print(f'steps : {steps}')

    # collision_max = 20
    # collision_cnt = 0
    q1_pos = np.asarray(obj_init_pose[:3])
    q2_pos = np.asarray(obj_tgt_pose[:3])
    q1_rot = np.asarray(obj_init_pose[3:])
    q2_rot = np.asarray(obj_tgt_pose[3:])
    d12 = q2_pos - q1_pos
    r12_rotvec = R.from_quat(q2_rot).as_rotvec() - R.from_quat(q1_rot).as_rotvec()
    diff_q1_q2 = np.concatenate((d12, r12_rotvec))
    steps = int(np.ceil(np.linalg.norm(np.divide(diff_q1_q2, planning_resolution), ord=2)))
    for i in range(steps):
        positions6d = (i + 1) / steps * diff_q1_q2 + np.concatenate((q1_pos, R.from_quat(q1_rot).as_rotvec()))
        positions7d = tuple(positions6d[:3]) + tuple(R.from_rotvec(positions6d[3:]).as_quat())
        draw_coordinate(positions7d)
        p.resetBasePositionAndOrientation(obj_id, positions7d[:3], positions7d[3:])
        time.sleep(sim_timestep)

    for i in range(steps):
        ratio = (i + 1) / steps
        pos = ratio * d12 + np.asarray(obj_init_pose[:3])
        obj_init_quat = quaternion.as_quat_array(xyzw2wxyz(obj_init_pose[3:]))
        obj_tgt_quat = quaternion.as_quat_array(xyzw2wxyz(obj_tgt_pose[3:]))
        quat = quaternion.slerp_evaluate(obj_init_quat, obj_tgt_quat, ratio)
        quat = wxyz2xyzw(quaternion.as_float_array(quat))
        positions7d = tuple(pos) + tuple(quat)
        draw_coordinate(positions7d)
        p.resetBasePositionAndOrientation(obj_id, positions7d[:3], positions7d[3:])
        time.sleep(sim_timestep)

    while True:
        p.stepSimulation()
        time.sleep(sim_timestep)


if __name__=="__main__":
    main()