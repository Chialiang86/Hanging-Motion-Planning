import argparse, glob, time, json, imageio, os, cv2
import numpy as np
import open3d as o3d
import pybullet as p
import xml.etree.ElementTree as ET

from sklearn.neighbors import NearestNeighbors

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

def render_affordance_map(points : np.ndarray, center : np.ndarray, std : float=0.01, return_color : bool=False):

    points_diff = np.linalg.norm(points - center, axis=1, ord=2)
    # print(f'the closest point to the center is {np.min(points_diff)}')
    affordance_map = np.exp(-0.5 * (points_diff / std) ** 2)
    affordance_map = (affordance_map - np.min(affordance_map)) / (np.max(affordance_map) - np.min(affordance_map))

    colors = None
    if return_color:
      colors = cv2.applyColorMap((255 * affordance_map).astype(np.uint8), colormap=cv2.COLORMAP_JET).squeeze()
      colors = colors[:,::-1]

    return affordance_map.reshape(-1, 1), colors

def render_part_affordance_map(points : np.ndarray, centers : np.ndarray, std : float=0.01, return_color=False):

    affordance_maps = None
    for center in centers:
      affordance_map, _ = render_affordance_map(points, center, std, return_color=False)
      affordance_maps = affordance_map if affordance_maps is None else np.hstack((affordance_maps, affordance_map))
    affordance_map = np.max(affordance_maps, 1)

    colors = None
    if return_color:
      colors = cv2.applyColorMap((255 * affordance_map).astype(np.uint8), colormap=cv2.COLORMAP_JET).squeeze()
      colors = colors[:,::-1]

    return affordance_map.reshape(-1, 1), colors

def capture_from_viewer(geometries):
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    for geometry in geometries:
        vis.add_geometry(geometry)

    # Updates
    for geometry in geometries:
        vis.update_geometry(geometry)
    vis.poll_events()
    vis.update_renderer()

    o3d_screenshot_mat = vis.capture_screen_float_buffer(do_render=True) # need to be true to capture the image
    o3d_screenshot_mat = (255.0 * np.asarray(o3d_screenshot_mat)).astype(np.uint8)
    o3d_screenshot_mat = cv2.cvtColor(o3d_screenshot_mat,cv2.COLOR_BGR2RGB)
    o3d_screenshot_mat = cv2.resize(o3d_screenshot_mat, (o3d_screenshot_mat.shape[1] // 6, o3d_screenshot_mat.shape[0] // 6))
    vis.destroy_window()

    return o3d_screenshot_mat

def main(args):

    # extract file info
    data_dir = f'{args.data_root}/{args.data_dir}'
    assert os.path.exists(args.data_root), f'{args.data_root} not exists'
    assert os.path.exists(data_dir), f'{data_dir} not exists'
    
    kptraj_dir = f'{args.kptraj_root}/{args.kptraj_dir}'
    assert os.path.exists(args.kptraj_root), f'{args.kptraj_root} not exists'
    assert os.path.exists(kptraj_dir), f'{kptraj_dir} not exists'
    
    std = args.std

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


    for data_dir in tqdm(data_dirs[-20:]):
        
        if not os.path.isdir(data_dir):
          continue
        
        # print(f'processing {data_dir} ...')
        pivot_json = glob.glob(f'{data_dir}/*hanging_exp_daily_5.json')[0]
        # pivot_json = glob.glob(f'{data_dir}/*everyday_objects_50_daily_63.json')[0]

        f_json = open(pivot_json, 'r')
        json_dict = json.load(f_json)

        # hook pose
        hook_pose_7d = json_dict['hook_pose']
        hook_trans = get_matrix_from_pos_rot(hook_pose_7d[:3], hook_pose_7d[3:])
        # hook urdf
        hook_urdf = json_dict['hook_path']
        hook_id, center, scale = load_obj_urdf(hook_urdf, [0, 0, 0], [0, 0, 0])

        ply_paths = glob.glob(f'{os.path.split(hook_urdf)[0]}/base-*.ply')
        ply_paths.sort()
        
        # hook ply
        for ply_path in ply_paths:

          ply_id = os.path.split(ply_path)[-1].split('.')[0].split('-')[-1]
          # if os.path.exists(affordance_path):
          #   print(f'ignore {affordance_path}')
          #   continue

          hook_pcd = o3d.io.read_point_cloud(ply_path)
          hook_pcd_points = np.asarray(hook_pcd.points)

          # # hook affordance map
          # contact_pos = json_dict['contact_info'][0]['contact_point_hook']
          # contact_trans = get_matrix_from_pos_rot(contact_pos[:3], [0, 0, 0, 1])
          # kpt_pos = contact_trans[:3, 3]
          
          # # load one trajectory
          hook_name = ply_path.split('/')[-2]
          kptraj_path = f'{kptraj_dir}/{hook_name}.json'
          kptraj_dict = json.load(open(kptraj_path, 'r'))
          kptraj = kptraj_dict['trajectory'][0][::-1][:100]
          kptraj_first_pos = np.asarray(kptraj)[0, :3].reshape(-1, 3)
          nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(hook_pcd_points)
          distances, indices = nbrs.kneighbors(kptraj_first_pos)
          kpt_pos = hook_pcd_points[indices]

          affordance_map, colors = render_affordance_map(hook_pcd_points, kpt_pos.reshape(-1), std, return_color=args.visualize) # this function will change the color of 'hook_pcd'
          # move contact point to the first point
          cond = np.where(affordance_map == np.max(affordance_map))
          before_ind = list(cond[0]) + [0]
          after_ind = [0] + list(cond[0])
          affordance_map[after_ind] = affordance_map[before_ind]
          hook_pcd_points[after_ind] = hook_pcd_points[before_ind]

          if colors is not None:
            colors[after_ind] = colors[before_ind]
            hook_pcd.colors = o3d.utility.Vector3dVector(colors / 255)
            o3d.visualization.draw_geometries([hook_pcd])

          # part segmentation
          kptraj_pos = np.asarray(kptraj)[:, :3]
          nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(hook_pcd_points)
          distances, indices = nbrs.kneighbors(kptraj_pos)
          centers = hook_pcd_points[indices]
          
          partseg_map, colors = render_part_affordance_map(hook_pcd_points, centers, std, return_color=args.visualize)

          if colors is not None:
            hook_pcd.colors = o3d.utility.Vector3dVector(colors / 255)
            o3d.visualization.draw_geometries([hook_pcd])

          affordance_path = os.path.split(hook_urdf)[0] + f'/affordance-{ply_id}.npy'
          hook_affordance = np.hstack((hook_pcd_points, affordance_map, partseg_map))
          np.save(open(affordance_path, 'wb'), hook_affordance)

          # fusion = (affordance_map + partseg_map) / 2 

          # colors = cv2.applyColorMap((255 * fusion).astype(np.uint8), colormap=cv2.COLORMAP_JET).squeeze()
          # colors = colors[:,::-1]
          # hook_pcd.colors = o3d.utility.Vector3dVector(colors / 255)
          # o3d.visualization.draw_geometries([hook_pcd])

          # rotate_per_frame = np.pi / 2 
          # r = hook_pcd.get_rotation_matrix_from_xyz((0, rotate_per_frame, 0)) # (rx, ry, rz) = (right, up, inner)
          # hook_pcd.rotate(r, center=(0, 0, 0))
          # geometries = [hook_pcd]

          # img = capture_from_viewer(geometries)
          # save_path = f"visualization/affordance/{hook_name}-{ply_id}.jpg"
          # print(f'{save_path} saved')
          # imageio.imsave(save_path, img)

          # break

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
    parser.add_argument('--data-dir', '-id', type=str, default='data_all_new')
    parser.add_argument('--kptraj-root', '-kr', type=str, default='keypoint_trajectory')
    parser.add_argument('--kptraj-dir', '-kd', type=str, default='kptraj_all_new')
    parser.add_argument('--std', '-std', type=float, default=0.005)
    parser.add_argument('--visualize', '-v', action="store_true")
    args = parser.parse_args()
    main(args)