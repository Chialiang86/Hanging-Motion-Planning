import argparse
import glob
import time
import json
import numpy as np
import skrobot
import os
import cv2
import open3d as o3d
import pybullet as p
import xml.etree.ElementTree as ET

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
    points_diff = np.linalg.norm(points - center, axis=1, ord=2)
    points_gaussian = np.exp(-0.5 * (points_diff / std) ** 2)
    points_gaussian = 255 * (points_gaussian - np.min(points_gaussian)) / (np.max(points_gaussian) - np.min(points_gaussian))
    # points_gaussian = 0 * (points_gaussian - np.min(points_gaussian)) / (np.max(points_gaussian) - np.min(points_gaussian))
    colors = cv2.applyColorMap(points_gaussian.astype(np.uint8), colormap=cv2.COLORMAP_JET).squeeze()
    colors = colors[:,::-1]
    pcd.colors = o3d.utility.Vector3dVector(colors)

def main(args):

    # extract file info
    input_dir = args.input_dir
    std = args.std
    assert os.path.exists(input_dir), f'{input_dir} not exists'

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
    p.setTimeStep(sim_timestep)

    hook_dirs = glob.glob(f'{input_dir}/*')
    for hook_dir in hook_dirs:
        first_json = glob.glob(f'{hook_dir}/*exp_daily_5.json')[0]
        print(f'processing {first_json} ...')

        f_json = open(first_json, 'r')
        json_dict = json.load(f_json)

        # hook pose
        hook_pose_7d = json_dict['hook_pose']
        hook_trans = get_matrix_from_pos_rot(hook_pose_7d[:3], hook_pose_7d[3:])
        # hook urdf
        hook_urdf = json_dict['hook_path']
        hook_id, center, scale = load_obj_urdf(hook_urdf, hook_pose_7d[:3], hook_pose_7d[3:])
        # hook ply
        ply_path = os.path.split(hook_urdf)[0] + '/base_single.ply'
        hook_pcd = o3d.io.read_point_cloud(ply_path)
        hook_pcd.transform(hook_trans)
        origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

        # obj pose
        contact_pos = json_dict['contact_info'][0]['contact_point_hook']
        contact_trans = get_matrix_from_pos_rot(contact_pos[:3], [0, 0, 0, 1])
        contact_trans_world = hook_trans @ contact_trans
        # GT target pose 
        kpt_pos_world = contact_trans_world[:3, 3]

        render_affordance_map(hook_pcd, kpt_pos_world, std)
        o3d.visualization.draw_geometries([hook_pcd])

        draw_coordinate(contact_trans_world)

        # while True:
        #     # key callback
        #     keys = p.getKeyboardEvents()            
        #     if ord('q') in keys and keys[ord('q')] & (p.KEY_WAS_TRIGGERED ): 
        #         break
        p.removeBody(hook_id)
        p.removeAllUserDebugItems()


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-dir', '-id', type=str, default='data')
    parser.add_argument('--std', '-std', type=float, default=0.01)
    args = parser.parse_args()
    main(args)