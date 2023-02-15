# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.

import os, inspect
import argparse
import glob
import time
import json
from matplotlib.pyplot import draw
import numpy as np
from sklearn.cluster import DBSCAN
import os

from utils.motion_planning_utils import get_collision7d_fn
from utils.bullet_utils import get_matrix_from_pos_rot, draw_coordinate
from scipy.spatial.transform import Rotation as R
import xml.etree.ElementTree as ET

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print(currentdir)
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import pybullet as p
import pybullet_data
from pybullet_robot_envs.envs.panda_envs.panda_env import pandaEnv

def reset_obj_pose(obj_id, obj_pose, hook_id):

    low_limit = [-0.005, -0.005, -0.005, -np.pi / 180, -np.pi / 180, -np.pi / 180]
    high_limit = [ 0.005,  0.005,  0.005,  np.pi / 180,  np.pi / 180,  np.pi / 180]

    obj_pos, obj_rot = obj_pose[:3], obj_pose[3:]
    original_pose = np.asarray(obj_pos + obj_rot)
    refined_pose = original_pose
    while True:
        contact_points_obj = p.getContactPoints(obj_id, hook_id)
        if len(contact_points_obj) == 0:
            refined_pose6d = np.concatenate((np.asarray(obj_pos), R.from_quat(obj_rot).as_rotvec())) + \
                            np.random.uniform(low_limit, high_limit)
            refined_pose = np.concatenate((refined_pose6d[:3], R.from_rotvec(refined_pose6d[3:]).as_quat()))
            break
    p.resetBasePositionAndOrientation(obj_id, refined_pose[:3], refined_pose[3:])
    print('refined')

def extract_contact_point(candidate_pts : np.ndarray, eps=0.002, min_samples=2):

    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(candidate_pts)
    clustering_labels = clustering.labels_

    # compute cluster height
    cluster_means_z = []
    for i in range(np.max(clustering_labels) + 1):
        cond = np.where(clustering_labels == i)
        cluster_pts = candidate_pts[cond]
        
        cluster_mean_z = np.mean(cluster_pts, axis=0)[2]
        cluster_means_z.append(cluster_mean_z)
    
    # no clustering
    if len(cluster_means_z) == 0:
        return list(sorted(candidate_pts, key=lambda x: x[2])[0])

    cluster_means_z = np.asarray(cluster_means_z)
    
    # get the highest cluster
    highest_cluster_id = np.argsort(-cluster_means_z)[0]
    
    # the lowest point in the highest cluster
    cond = np.where(clustering_labels == highest_cluster_id)
    highest_pts = candidate_pts[cond]
    lphc_id = np.argsort(-highest_pts[:, 1])[0]
    lphc = highest_pts[lphc_id]

    return lphc

def main(args):
    hook_dir = args.hook_dir
    object_dir = args.object_dir
    data_root = args.data_root
    pivot_root = os.path.join(data_root, args.pivot_root)
    output_root = os.path.join(data_root, args.output_root)

    assert os.path.exists(hook_dir), f'{hook_dir} not exists'
    assert os.path.exists(object_dir), f'{object_dir} not exists'
    assert os.path.exists(pivot_root), f'{pivot_root} not exists'

    if not os.path.exists(output_root):
        os.mkdir(output_root)

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
    p.setGravity(0, 0, gravity)

    ignore_list = [ 
        # "daily_5/",
        "bag_5/",  
        "scissor_4/", "mug_59/", "wrench_1/", 
        "bag_6/", "bag_70/",
        "daily_11/", "daily_114/", "daily_115/", "daily_2/", "daily_23/",  
        "daily_42/", "daily_57/", "daily_63/", "daily_84/", "daily_7/", "daily_71/", "daily_72/",
        "daily_85/", "daily_97/", "daily_8/", "daily_106/", "daily_41/",
        "mug_118/",
        "mug_100/", "mug_11/", "mug_112/", "mug_113/", "mug_115/",  "mug_123/", "mug_126/", "mug_128/", 
        "mug_132/", "mug_67/", "mug_70/", "mug_80/", "mug_82/", "mug_90/", "mug_135/", "mug_199/", "mug_73/", "mug_129/",
        "mug_142/", "mug_146/", "mug_147/", "mug_149/", "mug_150/", "mug_159/", "mug_166/", "mug_184/",
        "mug_173/", "mug_181/", "mug_19/", "mug_193/", "mug_204/", "mug_205/", "mug_43/", "mug_145/", "mug_64/",
        "scissor_101/", "scissor_12/", "scissor_14/", "scissor_19/", "scissor_22/", "scissor_27/", "scissor_39/", 
        "scissor_48/", "scissor_58/", "scissor_62/", "scissor_74/", "scissor_79/", "scissor_8/", "scissor_92/", 
        "scissor_95/", "scissor_98/", "scissor_31/",
        "wrench_10/", "wrench_12/",  "wrench_17/", "wrench_35/", "wrench_25/", 
        "wrench_27/", "wrench_31/", "wrench_32/", "wrench_36/", "wrench_6/"
    ]

    object_urdf_files = glob.glob(f'{object_dir}/*/base.urdf')
    hook_urdf_files = glob.glob(f'{hook_dir}/*/base.urdf')
    pivot_hook_urdf_files = []
    # generated_hook_urdf_files = []
    for urdf_file in hook_urdf_files:
        # pivot obj id will not contain '-' in folder name
        if '#' not in urdf_file:
            pivot_hook_urdf_files.append(urdf_file)
    pivot_hook_urdf_files.sort()

    # forward simulation params
    height_thresh = 0.8
    
    for pivot_hook_urdf_file in pivot_hook_urdf_files:

        pivot_hook_name = pivot_hook_urdf_file.split('/')[-2]

        cnt = 0
        generated_hook_urdf_file = '{}#{}/base.urdf'.format(os.path.split(pivot_hook_urdf_file)[0], cnt)
        while os.path.exists(generated_hook_urdf_file):

            print(f'================ {generated_hook_urdf_file} ================')
            
            generated_hook_name = generated_hook_urdf_file.split('/')[-2]

            for object_urdf_file in object_urdf_files:
                
                # filter out by ignore list
                cont_flag = False
                for ignore_item in ignore_list:
                    if ignore_item in object_urdf_file:
                        cont_flag = True 
                        break 
                if cont_flag:
                    continue
                
                # config io path info
                object_dir = object_urdf_file.split('/')[-3]
                object_name = object_urdf_file.split('/')[-2]
                full_object_name = object_dir + '_' + object_name
                # pivot directory
                pivot_dir = f'{pivot_root}/{pivot_hook_name}-{object_dir}'
                assert os.path.exists(pivot_dir), f'{pivot_dir} not exists'
                # config generated hook output directory
                output_dir = f'{output_root}/{generated_hook_name}-{object_dir}'
                os.makedirs(output_dir, exist_ok=True)
                # config generated_hook-obj_name.json
                output_json = f'{output_dir}/{generated_hook_name}-{full_object_name}.json'
                if os.path.exists(output_json):
                    print(f'ignore {output_json}')
                    continue

                # object pose and hook pose
                pivot_json = f'{pivot_dir}/{pivot_hook_name}-{full_object_name}.json'
                f_pivot = open(pivot_json, 'r')
                pivot_dict = json.load(f_pivot)
                assert 'hook_path' in pivot_dict.keys()
                hook_pose = pivot_dict['hook_pose']
                obj_pose = pivot_dict['contact_info'][0]['obj_pose']

                # generated_hook
                generated_hook_id = p.loadURDF(generated_hook_urdf_file, hook_pose[:3], hook_pose[3:])

                # object
                obj_id = p.loadURDF(object_urdf_file, obj_pose[:3], obj_pose[3:])

                # forward simulation
                contact_points = []
                pos, rot = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
                while True:
                    failed = False 
                    
                    # direct force
                    p.setGravity(0, 0, gravity)
                    for _ in range(1000):
                        p.stepSimulation()
                        pos, rot = p.getBasePositionAndOrientation(obj_id)
                        if pos[2] < height_thresh:
                            failed = True
                            break
                    if failed:
                        reset_obj_pose(obj_id, obj_pose, generated_hook_id)
                        continue

                    contact_points = p.getContactPoints(obj_id, generated_hook_id)
                    if len(contact_points) == 0:
                        reset_obj_pose(obj_id, obj_pose, generated_hook_id)
                        continue
                    
                    # left force
                    p.setGravity(2, 0, -5)
                    for _ in range(1000):
                        pos, rot = p.getBasePositionAndOrientation(obj_id)
                        if pos[2] < height_thresh:
                            failed = True
                            break
                        p.stepSimulation()
                    if failed:
                        reset_obj_pose(obj_id, obj_pose, generated_hook_id)
                        continue

                    # right force
                    p.setGravity(-2, 0, -5)
                    for _ in range(1000):
                        pos, rot = p.getBasePositionAndOrientation(obj_id)
                        if pos[2] < height_thresh:
                            failed = True
                        p.stepSimulation()
                    if failed:
                        reset_obj_pose(obj_id, obj_pose, generated_hook_id)
                        continue
                    
                    # make the object stable on the hook
                    p.setGravity(0, 0, gravity)
                    for _ in range(500):
                        pos, rot = p.getBasePositionAndOrientation(obj_id)
                        time.sleep(sim_timestep)
                        if pos[2] < height_thresh:
                            failed = True
                            break

                        p.stepSimulation()
                    if failed:
                        reset_obj_pose(obj_id, obj_pose, generated_hook_id)
                        continue

                    break

                # prepare data for json

                candidate_pts = []
                for contact_point in contact_points:
                    candidate_pts.append(contact_point[5])
                candidate_pts = np.asarray(candidate_pts)
                
                # relative homogeneous contact point
                contact_point = extract_contact_point(candidate_pts, eps=0.01, min_samples=3)
                contact_point_homo = np.concatenate((contact_point, [1]))

                # relative transform (hook, object)
                hook_transform = get_matrix_from_pos_rot(hook_pose[:3], hook_pose[3:])
                obj_transform = get_matrix_from_pos_rot(pos, rot)
                contact_point_hook = np.linalg.inv(hook_transform) @ contact_point_homo
                contact_point_obj = np.linalg.inv(obj_transform) @ contact_point_homo

                contact_info = {
                    'contact_point_hook': contact_point_hook.tolist(),
                    'contact_point_obj': contact_point_obj.tolist(),
                    'obj_pose': list(pos + rot),
                }
                result_json = {
                    'hook_path': generated_hook_urdf_file,
                    'obj_path': object_urdf_file,
                    'hook_pose': hook_pose,
                    'contact_info': [contact_info]
                }
                    
                f_out = open(output_json, 'w')
                json.dump(result_json, f_out, indent=4)
                print(f'processing {output_json} ...')

                p.removeBody(generated_hook_id)
                p.removeBody(obj_id)

            cnt += 1
            generated_hook_urdf_file = '{}#{}/base.urdf'.format(os.path.split(pivot_hook_urdf_file)[0], cnt)


start_msg = \
'''
======================================================================================
this script will generate contact points for the generated new objects in 
[data_root]/[output_root]/[hook_name#id-object_name]/[hook_name#id-object_name].json

dependency :
- hook folder that contains [hook_root]/[hook_name]/base.urdf
- object folder that contains [hook_root]/[object_name]/base.urdf
- the source folder that contain [data_root]/[pivot_root]/[hook_name-object_name]/[hook_name-object_name].json
======================================================================================
'''

print(start_msg)

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--hook-dir', '-hd', type=str, default='models/hook_1120')
    parser.add_argument('--object-dir', '-od', type=str, default='models/geo_data/hanging_exp')
    parser.add_argument('--data-root', '-dr', type=str, default='data')
    parser.add_argument('--pivot-root', '-pr', type=str, default='data')
    parser.add_argument('--output-root', '-or', type=str, default='data_1120')
    args = parser.parse_args()
    main(args)