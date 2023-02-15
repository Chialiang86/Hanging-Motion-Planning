import argparse
import glob
import time
import json
import numpy as np
import os
import cv2
import open3d as o3d
import pybullet as p
import xml.etree.ElementTree as ET

from tqdm import tqdm
from scipy.spatial.transform import Rotation as R
from utils.bullet_utils import get_matrix_from_pos_rot, get_pos_rot_from_matrix, draw_coordinate

def load_obj_urdf(urdf_path, pos=[0, 0, 0], rot=[0, 0, 0]):

    tree = ET.parse(urdf_path)
    root = tree.getroot()
    center = np.array(
      [
        float(i) for i in root[0].find(
          "inertial"
        ).find(
          "origin"
        ).attrib['xyz'].split(' ')
      ]
    )
    scale = np.array(
      [
        float(i) for i in root[0].find(
          "visual"
        ).find(
          "geometry"
        ).find(
          "mesh"
        ).attrib["scale"].split(" ")
      ]
    )
    if len(rot) == 3:
        obj_id = p.loadURDF(urdf_path, pos, p.getQuaternionFromEuler(rot))
    elif len(rot) == 4:
        obj_id = p.loadURDF(urdf_path, pos, rot)
    return obj_id, center, scale

def render_affordance_map(pcd : o3d.geometry.PointCloud, center : np.ndarray, std : float=0.01):
    points = np.asarray(pcd.points)
    print(f'there are {points.shape[0]} in point cloud')
    points_diff = np.linalg.norm(points - center, axis=1, ord=2)
    print(f'the closest point to the center is {np.min(points_diff)}')

    points_gaussian = np.exp(-0.5 * (points_diff / std) ** 2)
    points_gaussian = (points_gaussian - np.min(points_gaussian)) / (np.max(points_gaussian) - np.min(points_gaussian))
    colors = cv2.applyColorMap((255 * points_gaussian).astype(np.uint8), colormap=cv2.COLORMAP_JET).squeeze()
    colors = colors[:,::-1]
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # check the point cloud contain the contact point
    cond = np.where(points_gaussian == np.max(points_gaussian))

    # move contact point to the first point
    val = points_gaussian[cond[0]]
    points_gaussian = np.delete(points_gaussian, cond[0], axis=0)
    points_gaussian = np.insert(points_gaussian, 0, val, axis=0)
    val = points[cond[0]]
    points = np.delete(points, cond[0], axis=0)
    points = np.insert(points, 0, val, axis=0)

    affordance_map = np.hstack((points, points_gaussian.reshape(-1, 1)))

    return affordance_map

def main(args):

    # extract file info
    data_dir = f'{args.data_root}/{args.data_dir}'
    std = args.std
    assert os.path.exists(args.data_root), f'{args.data_root} not exists'
    assert os.path.exists(data_dir), f'{data_dir} not exists'

    # Create pybullet GUI
    physics_client_id = p.connect(p.GUI)
    # p.resetDebugVisualizerCamera(2.1, 90, -30, [0.0, -0.0, -0.0])
    p.resetDebugVisualizerCamera(
        cameraDistance=0.3,
        cameraYaw=120,
        cameraPitch=-30,
        cameraTargetPosition=[0.0, 0.0, 0.0]
    )
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=150)
    sim_timestep = 1.0 / 1000
    gravity = -9.8
    p.setTimeStep(sim_timestep)

    data_dirs = glob.glob(f'{data_dir}/*')
    data_dirs.sort()

    for data_dir in tqdm(data_dirs):

        if not os.path.isdir(data_dir):
          continue
        
        print(f'processing {data_dir} ...')
        pivot_json = glob.glob(f'{data_dir}/*exp_daily_5.json')[0]

        f_json = open(pivot_json, 'r')
        json_dict = json.load(f_json)

        # hook pose
        # hook_pose_7d = json_dict['hook_pose']
        # hook_trans = get_matrix_from_pos_rot(hook_pose_7d[:3], hook_pose_7d[3:])
        # hook urdf
        hook_urdf = json_dict['hook_path']
        hook_id, center, scale = load_obj_urdf(hook_urdf, [0, 0, 0], [0, 0, 0])

        ply_paths = glob.glob(f'{os.path.split(hook_urdf)[0]}/*.ply')
        ply_paths.sort()
        
        # hook ply
        for ply_path in ply_paths:
          if not os.path.exists(ply_path):
            print(f'{ply_path} is not been written')
            p.removeBody(hook_id)
            p.removeAllUserDebugItems()
            continue

          ply_id = os.path.split(ply_path)[-1].split('.')[0].split('-')[-1]
          affordance_path = os.path.split(hook_urdf)[0] + f'/affordance-{ply_id}.npy'
          # if os.path.exists(affordance_path):
          #   print(f'ignore {affordance_path}')
          #   continue

          hook_pcd = o3d.io.read_point_cloud(ply_path)
          # hook_pcd.transform(hook_trans)
          origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

          # obj pose
          contact_pos = json_dict['contact_info'][0]['contact_point_hook']
          contact_trans = get_matrix_from_pos_rot(contact_pos[:3], [0, 0, 0, 1])
          kpt_pos = contact_trans[:3, 3]

          # contact_trans_world = hook_trans @ contact_trans
          # GT target pose 
          # kpt_pos_world = contact_trans_world[:3, 3]

          # hook affordance map
          # affordance_map = render_affordance_map(hook_pcd, kpt_pos_world, std)
          affordance_map = render_affordance_map(hook_pcd, kpt_pos, std)
          np.save(open(affordance_path, 'wb'), affordance_map)
          print(f'{affordance_path} saved')
          # draw_coordinate(contact_trans, size=0.001)

          # o3d.visualization.draw_geometries([hook_pcd])

        # while True:
        #     # key callback
        #     keys = p.getKeyboardEvents()            
        #     if ord('q') in keys and keys[ord('q')] & (p.KEY_WAS_TRIGGERED ): 
        #         break


        p.removeBody(hook_id)
        p.removeAllUserDebugItems()

start_msg = \
'''
======================================================================================
this script will create the affordance maps from the given object files in [root]/[objct_name]/base.urdf 
and the contact points information in [data_root]/[data_dir]/[hook_name-obj_name]/[hook_name-obj_name].urdf 
then save the affordace maps into the same folder

dependency :
- [obj_root]/[obj_name]/base.urdf
- [obj_root]/[obj_name]/base.ply
- [data_root]/[data_dir]/[hook_name-obj_name]/[hook_name-obj_name].urdf
======================================================================================
'''

print(start_msg)

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--data-root', '-ir', type=str, default='data')
    parser.add_argument('--data-dir', '-id', type=str, default='data')
    parser.add_argument('--std', '-std', type=float, default=0.005)
    args = parser.parse_args()
    main(args)