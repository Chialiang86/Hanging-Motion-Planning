# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.
import os, glob, inspect
import argparse
import copy
import json
import time
import quaternion
import torch
import scipy.io as sio
import numpy as np
import open3d as o3d
import pybullet as p
import xml.etree.ElementTree as ET

from tqdm import tqdm
from scipy.spatial.transform import Rotation as R
from pointnet2_ops.pointnet2_utils import furthest_point_sample

from utils.bullet_utils import get_matrix_from_pose, get_pose_from_matrix, get_matrix_from_pos_rot, get_pos_rot_from_matrix, xyzw2wxyz, wxyz2xyzw, draw_coordinate, draw_bbox

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)\

obj_subpaths = [
    'everyday_objects_50_cooking_11.json', 'everyday_objects_50_cooking_117.json', 'everyday_objects_50_cooking_132.json', 'everyday_objects_50_cooking_169.json', 'everyday_objects_50_cooking_23.json',
    'everyday_objects_50_cooking_310.json', 'everyday_objects_50_cooking_32.json', 'everyday_objects_50_cooking_42.json', 'everyday_objects_50_cooking_74.json', 'everyday_objects_50_cooking_77.json',
    'everyday_objects_50_daily_115.json', 'everyday_objects_50_daily_41.json', 'everyday_objects_50_daily_5.json', 'everyday_objects_50_daily_6.json', 'everyday_objects_50_daily_63.json',
    'everyday_objects_50_daily_70.json', 'everyday_objects_50_daily_71.json', 'everyday_objects_50_daily_72.json', 'everyday_objects_50_daily_8.json', 'everyday_objects_50_daily_85.json',
    'everyday_objects_50_mug_100.json', 'everyday_objects_50_mug_113.json', 'everyday_objects_50_mug_115.json', 'everyday_objects_50_mug_118.json', 'everyday_objects_50_mug_126.json',
    'everyday_objects_50_mug_128.json', 'everyday_objects_50_mug_19.json', 'everyday_objects_50_mug_193.json', 'everyday_objects_50_mug_67.json', 'everyday_objects_50_mug_90.json',
    'everyday_objects_50_scissor_19.json', 'everyday_objects_50_scissor_27.json', 'everyday_objects_50_scissor_31.json', 'everyday_objects_50_scissor_39.json', 'everyday_objects_50_scissor_48.json',
    'everyday_objects_50_scissor_58.json', 'everyday_objects_50_scissor_74.json', 'everyday_objects_50_scissor_79.json', 'everyday_objects_50_scissor_8.json', 'everyday_objects_50_scissor_95.json',
    'everyday_objects_50_tool_1.json', 'everyday_objects_50_tool_10.json', 'everyday_objects_50_tool_12.json', 'everyday_objects_50_tool_127.json', 'everyday_objects_50_tool_27.json',
    'everyday_objects_50_tool_31.json', 'everyday_objects_50_tool_32.json', 'everyday_objects_50_tool_35.json', 'everyday_objects_50_tool_36.json', 'everyday_objects_50_tool_6.json'
]

hook_subpaths = [
    'Hook_hcu_104_devil.json', 'Hook_hcu_134_normal.json', 'Hook_hcu_138_hard.json', 'Hook_hcu_181_hard.json', 'Hook_hcu_190_easy.json', 
    'Hook_hcu_243_normal.json', 'Hook_hcu_279_devil.json', 'Hook_hcu_293_easy.json', 'Hook_hcu_296_normal.json', 'Hook_hcu_303_normal.json', 
    'Hook_hcu_306_normal.json', 'Hook_hcu_335_normal.json', 'Hook_hcu_359_easy.json', 'Hook_hcu_362_easy.json', 'Hook_hcu_364_normal.json', 
    'Hook_hcu_376_devil.json', 'Hook_hcu_380_hard.json', 'Hook_hcu_390_easy.json', 'Hook_hcu_3_hard.json', 'Hook_hcu_75_easy.json', 
    'Hook_hcu_89_devil.json', 'Hook_hs_105_hard.json', 'Hook_hs_117_hard.json', 'Hook_hs_154_hard.json', 'Hook_hs_156_hard.json', 
    'Hook_hs_190_easy.json', 'Hook_hs_216_easy.json', 'Hook_hs_229_normal.json', 'Hook_hs_275_devil.json', 'Hook_hs_293_normal.json', 
    'Hook_hs_314_easy.json', 'Hook_hs_317_easy.json', 'Hook_hs_339_devil.json', 'Hook_hs_363_devil.json', 'Hook_hs_370_easy.json', 
    'Hook_hs_393_devil.json', 'Hook_hs_42_hard.json', 'Hook_hs_70_easy.json', 'Hook_hs_94_hard.json', 'Hook_hs_95_devil.json', 
    'Hook_hsr_118_hard.json', 'Hook_hsr_123_normal.json', 'Hook_hsr_125_easy.json', 'Hook_hsr_13_devil.json', 'Hook_hsr_15_normal.json', 
    'Hook_hsr_218_devil.json', 'Hook_hsr_22_normal.json', 'Hook_hsr_263_hard.json', 'Hook_hsr_298_normal.json', 'Hook_hsr_304_hard.json', 
    'Hook_hsr_312_devil.json', 'Hook_hsr_321_devil.json', 'Hook_hsr_335_hard.json', 'Hook_hsr_371_devil.json', 'Hook_hsr_381_easy.json', 
    'Hook_hsr_391_hard.json', 'Hook_hsr_56_normal.json', 'Hook_hsr_5_normal.json', 'Hook_hsr_71_easy.json', 'Hook_omni_124_devil.json'
]


def refine_rotation(src_transform, tgt_transform):
    src_rot = src_transform[:3, :3]
    tgt_rot = tgt_transform[:3, :3]

    s2d_before = R.from_matrix(tgt_rot @ np.linalg.inv(src_rot)).as_rotvec()

    rot_180 = np.identity(4)
    rot_180[:3, :3] = R.from_rotvec([0, 0, np.pi]).as_matrix()
    tgt_dual_transform = tgt_transform @ rot_180
    s2d_after = R.from_matrix(tgt_dual_transform[:3, :3] @ np.linalg.inv(src_rot)).as_rotvec()

    return tgt_transform if np.sum((s2d_before) ** 2) < np.sum((s2d_after) ** 2) else tgt_dual_transform

def random_sample(start_conf, target_conf):

    # config sample location space
    start_pos = start_conf[:3]
    target_pos = target_conf[:3]
    
    low_limit = [0, 0, 0]
    high_limit = [0, 0, 0]
    up_padding = [0.05, 0.02, 0.08]
    down_padding = [0.05, 0.08, 0.02]

    # xlim, ylim
    for i in range(3):
        if start_pos[i] < target_pos[i]:
            low_limit[i] = start_pos[i] - down_padding[i]
            high_limit[i] = target_pos[i] + up_padding[i] 
        else :
            low_limit[i] = target_pos[i] - down_padding[i]
            high_limit[i] = start_pos[i] + up_padding[i]
    # draw_bbox(low_limit, high_limit) 

    pos_euler = []
    pos_euler.append(np.random.uniform(low_limit[0], high_limit[0])) # x
    pos_euler.append(np.random.uniform(low_limit[1], high_limit[1])) # y 
    pos_euler.append(np.random.uniform(low_limit[2], high_limit[2])) # z
    for i in range(3, 6):
        pos_euler.append(np.random.uniform(-np.pi, np.pi)) # row

    ret = pos_euler[:3] + list(R.from_rotvec(pos_euler[3:]).as_quat())

    return ret

def do_fps(src_points: np.ndarray, sample_num_points = 1000):

    src_points_batch = torch.from_numpy(src_points.copy().astype(np.float32)).unsqueeze(0).to('cuda').contiguous()

    input_pcid = None
    point_num = src_points_batch.shape[1]
    if point_num >= sample_num_points:
        input_pcid = furthest_point_sample(src_points_batch, sample_num_points).long().reshape(-1)  # BN
        ret_points = src_points_batch[0, input_pcid, :].squeeze()
    else :
        mod_num = sample_num_points % point_num
        repeat_num = int(sample_num_points // point_num)
        input_pcid = furthest_point_sample(src_points_batch, mod_num).long().reshape(-1)  # BN
        ret_points = torch.cat([src_points_batch.repeat(1, repeat_num, 1), src_points_batch[:, input_pcid]], dim=1).squeeze()
    
    return ret_points.cpu().numpy()

def main(args):

    time_stamp = time.localtime()
    time_mon_day = '{:02d}{:02d}'.format(time_stamp.tm_mon, time_stamp.tm_mday)

    data_dir = f'{args.data_root}/{args.data_dir}'
    kpt_trajectory_dir = f'{args.input_root}/{args.input_dir}'

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

    assert os.path.exists(data_dir), f'{data_dir} not exists'
    assert os.path.exists(kpt_trajectory_dir), f'{kpt_trajectory_dir} not exists'
    assert os.path.exists(args.output_root), f'{args.output_root} not exists'

    out_root = f'{args.output_root}/{args.output_dir}'
    # os.makedirs(out_root, exist_ok=True)

    sample_num_points = args.sample_num_points
    sample_thresh_per_traj = 100

    hook_pose = [
        0.5,
        -0.1,
        1.3,
        4.329780281177466e-17,
        0.7071067811865475,
        0.7071067811865476,
        4.329780281177467e-17
    ]
    hook_trans = get_matrix_from_pose(hook_pose)

    for obj_subpath in tqdm(obj_subpaths):
        for hook_subpath in hook_subpaths:

            pair_name = '{}-{}'.format(obj_subpath.split('.')[0], hook_subpath.split('.')[0])
            out_dir = f'{out_root}/{pair_name}'
            # os.makedirs(out_dir, exist_ok=True)
    
            # load model
            obj_fname = f'{kpt_trajectory_dir}/{obj_subpath}'
            hook_fname = f'{kpt_trajectory_dir}/{hook_subpath}'
            obj_name = os.path.split(obj_fname)[1].split('.')[0]
            hook_name = os.path.split(hook_fname)[1].split('.')[0]
            obj_hook_pair_fname = f'{data_dir}/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-{obj_name}.json'

            print(hook_name, obj_name)

            assert os.path.exists(obj_hook_pair_fname), f'{obj_hook_pair_fname} not exists'
            assert os.path.exists(obj_fname), f'{obj_fname} not exists'
            assert os.path.exists(hook_fname), f'{hook_fname} not exists'

            with open(obj_hook_pair_fname, 'r') as f:
                obj_hook_pair_dict = json.load(f)
            with open(obj_fname, 'r') as f:
                obj_dict = json.load(f)
            with open(hook_fname, 'r') as f:
                hook_dict = json.load(f)

            # assert some attributes exist in the given json files
            assert 'initial_pose' in obj_hook_pair_dict.keys(), \
                f'"initial_pose" not in obj_hook_pair_dict!, please run hanging_init_pose.py'
            assert 'contact_pose' in obj_dict.keys() and 'file' in obj_dict.keys(), \
                f'"contact_pose" or "file" not in obj_dict!, please run keypoint_pose.py'
            assert 'hook_pose' in hook_dict.keys() and 'file' in hook_dict.keys() and 'trajectory' in hook_dict.keys(), \
                f'"hook_pose" or "file" or "trajectory" not in hook_dict!, please run keypoint_trajectory.py'
            
            # load object and hook

            obj_id = p.loadURDF(obj_dict['file'])
            hook_id = p.loadURDF(hook_dict['file'], hook_pose[:3], hook_pose[3:])

            obj_pose = obj_dict['obj_pose']
            p.resetBasePositionAndOrientation(obj_id, obj_pose[:3], obj_pose[3:])
            
            obj_ply_path = glob.glob('{}/*-0.ply'.format(os.path.split(obj_dict['file'])[0]))[0]
            hook_ply_path = glob.glob('{}/*-0.ply'.format(os.path.split(hook_dict['file'])[0]))[0]

            tree = ET.parse(obj_dict['file'])
            root = tree.getroot()
            center = np.array([float(i) for i in root[0].find(
                "inertial").find("origin").attrib['xyz'].split(' ')]).reshape((-1, 1))

            hook_pcd_static = o3d.io.read_point_cloud(hook_ply_path)
            hook_pcd_static.transform(hook_trans)
            obj_pcd_static = o3d.io.read_point_cloud(obj_ply_path)
            obj_pcd_static.translate(-center)
            obj_pcd_dynamic = copy.deepcopy(obj_pcd_static)
            
            obj_contact_pose_6d = obj_dict['contact_pose']
            obj_contact_relative_transform = get_matrix_from_pos_rot(obj_contact_pose_6d[:3], obj_contact_pose_6d[3:])

            # load trajectory
            trajectories_hooks = hook_dict['trajectory'][:1]

            for traj_i in range(len(trajectories_hooks)):
                    
                    trajectory_hook = trajectories_hooks[traj_i]

                    obj_start_config = get_pose_from_matrix(hook_trans @ get_matrix_from_pose(trajectory_hook[-100]) @ np.linalg.inv(obj_contact_relative_transform))
                    obj_end_config = get_pose_from_matrix(hook_trans @ get_matrix_from_pose(trajectory_hook[0]) @ np.linalg.inv(obj_contact_relative_transform))

                    positive_sample_cnt = 0
                    negative_sample_cnt = 0
                    trail = 0
                    while (positive_sample_cnt < sample_thresh_per_traj) or (negative_sample_cnt < sample_thresh_per_traj):
                        
                        for i in range(0, 100, 5):

                            wi_abs_trans = hook_trans @ get_matrix_from_pose(trajectory_hook[-100:][i])
                            obji_trans = wi_abs_trans @ np.linalg.inv(obj_contact_relative_transform)

                            obji_pose = get_pose_from_matrix(obji_trans)

                            # generate sample pose

                            if trail == 0:

                                obji_pos = obji_pose[:3]
                                obji_rot = obji_pose[3:]
                                obj_target_trans = obji_trans
                                p.resetBasePositionAndOrientation(obj_id, obji_pos, obji_rot)

                            if trail > 0 and trail % 2 == 1:

                                pos_noise = (2.0 * (np.random.rand(3) - 0.5)) * 0.01
                                rot_noise = (2.0 * (np.random.rand(3) - 0.5)) * 30 * (np.pi / 180)
                                obji_pos = obji_pose[:3] + pos_noise
                                obji_rot = R.from_matrix(R.from_rotvec(rot_noise).as_matrix() @ R.from_quat(obji_pose[3:]).as_matrix()).as_quat()
                                obj_target_trans = get_matrix_from_pos_rot(obji_pos, obji_rot)
                                p.resetBasePositionAndOrientation(obj_id, obji_pos, obji_rot)

                            if trail > 0 and trail % 2 == 0:

                                random_pose = random_sample(obj_start_config, obj_end_config)
                                obji_pos = random_pose[:3]
                                obji_rot = random_pose[3:]
                                obj_target_trans = get_matrix_from_pos_rot(obji_pos, obji_rot)
                                p.resetBasePositionAndOrientation(obj_id, obji_pos, obji_rot)

                            p.performCollisionDetection()
                            contact_points = p.getContactPoints(obj_id, hook_id)
                            status = 'collision' if len(contact_points) > 0 else 'safe'

                            positive_sample_cnt += 1 if (len(contact_points) > 0 and positive_sample_cnt < sample_thresh_per_traj) else 0
                            negative_sample_cnt += 1 if (len(contact_points) == 0 and negative_sample_cnt < sample_thresh_per_traj) else 0

                            # generate point cloud
                            if status == 'collision' and positive_sample_cnt > sample_thresh_per_traj:
                                continue
                            if status == 'safe' and negative_sample_cnt > sample_thresh_per_traj:
                                continue

                            obj_pcd_dynamic.transform(obj_target_trans)

                            # extract point cloud
                            hook_points = np.asarray(hook_pcd_static.points)
                            obj_points = np.asarray(obj_pcd_dynamic.points)
                            
                            hook_points_down = do_fps(hook_points, sample_num_points=sample_num_points)
                            # hook_pcd = o3d.geometry.PointCloud()
                            # hook_pcd.points =  o3d.utility.Vector3dVector(hook_points_down)
                            # o3d.visualization.draw_geometries([hook_pcd])

                            obj_points_down  = do_fps(obj_points , sample_num_points=sample_num_points)

                            hook_points_down_4d = np.hstack((hook_points_down, np.ones((hook_points_down.shape[0], 1))))
                            obj_points_down_4d  = np.hstack((obj_points_down , np.zeros((hook_points_down.shape[0], 1))))
                            hook_points_down_4d[:, :3] -= hook_trans[:3, 3]
                            obj_points_down_4d[:, :3] -= hook_trans[:3, 3]
                            
                            pared_points = np.vstack((hook_points_down_4d, obj_points_down_4d))

                            if trail > 0:
                                tmp_pcd = o3d.geometry.PointCloud()
                                colors = np.zeros(pared_points[:,:3].shape)
                                colors[:hook_points_down_4d.shape[0]] = np.array([1, 0, 0])
                                colors[hook_points_down_4d.shape[0]:] = np.array([0, 0, 1])
                                tmp_pcd.points = o3d.utility.Vector3dVector(pared_points[:, :3])
                                tmp_pcd.colors = o3d.utility.Vector3dVector(colors)
                                coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
                                o3d.visualization.draw_geometries([coor, tmp_pcd])

                            # out_path = '{}/{}-{}-{}.npy'.format(out_dir, traj_i, status, positive_sample_cnt if status == 'collision' else negative_sample_cnt)
                            # np.save(out_path, pared_points)

                            obj_pcd_dynamic.transform(np.linalg.inv(obj_target_trans))

                        trail += 1

            p.removeBody(obj_id)
            p.removeBody(hook_id)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--data_root', '-dr', type=str, default='data')
    parser.add_argument('--data_dir', '-dd', type=str, default='everyday_objects_50')
    parser.add_argument('--input_root', '-ir', type=str, default='keypoint_trajectory')
    parser.add_argument('--input_dir', '-id', type=str, default='everyday_objects_50')
    parser.add_argument('--output_root', '-or', type=str, default='collision_detection_sample')
    parser.add_argument('--output_dir', '-od', type=str, default='single_view')
    parser.add_argument('--sample_num_points', '-snp', type=int, default=1000)
    args = parser.parse_args()
    main(args)
