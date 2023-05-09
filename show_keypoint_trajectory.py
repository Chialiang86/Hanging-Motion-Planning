import argparse, json, os, glob, sys
import numpy as np
import pybullet as p
from PIL import Image
from tqdm import tqdm

from utils.bullet_utils import draw_coordinate, get_matrix_from_pos_rot, get_pos_rot_from_matrix, get_pose_from_matrix, get_matrix_from_pose

def main(args):

    assert os.path.exists(args.kptraj_root), f'{args.kptraj_root} not exists' 

    kptraj_dir = f'{args.kptraj_root}/{args.kptraj_dir}' if args.kptraj_dir != '' else f'{args.kptraj_root}/keypoint_trajectory'
    assert os.path.exists(kptraj_dir), f'{kptraj_dir} not exists'

    # Create pybullet GUI
    physics_client_id = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.1,
        cameraYaw=90,
        cameraPitch=0,
        cameraTargetPosition=[0.5, -0.05, 1.3]
    )
    # p.resetSimulation()
    # p.setPhysicsEngineParameter(numSolverIterations=150)
    # sim_timestep = 1.0 / 240
    # p.setTimeStep(sim_timestep)
    # p.setGravity(0, 0, -9.8)

    ignore_list = [ 
        # "aug"
        # 'Hook1.json', 'Hook2.json', 'Hook15.json', 'Hook23.json',  'Hook35.json', 
        # 'Hook40.json', 'Hook47.json', 'Hook57.json', 'Hook67.json', 'Hook84.json', 
        # 'Hook136.json', 'Hook145.json',  'Hook186.json',  'Hook209.json', 
        # 'Hook122.json', 'Hook12.json', 'Hook42.json'
    ]


    hook_pose = [
        0.5, 
        -0.1, 
        1.3,
        4.329780281177466e-17,
        0.7071067811865475,
        0.7071067811865476,
        4.329780281177467e-17
    ]

    object_json = f'{kptraj_dir}/hanging_exp_daily_5.json'
    object_dict = json.load(open(object_json, 'r'))
    object_urdf = object_dict['file']
    object_contact_pose = object_dict['contact_pose']

    object_contact_trans = get_matrix_from_pos_rot(object_contact_pose[:3], object_contact_pose[3:])

    obj_id = p.loadURDF(object_urdf)

    # kpt_trajectory_jsons = glob.glob(f'{kptraj_dir}/Hook*devil.json')
    kpt_trajectory_jsons = glob.glob(f'{kptraj_dir}/Hook_hsr_18_devil.json')
    kpt_trajectory_jsons.sort()
    kpt_trajectory_jsons = kpt_trajectory_jsons[::-1]

    for i, kpt_trajectory_json in enumerate(kpt_trajectory_jsons):
        cont = False
        for ignore_item in ignore_list:
            if ignore_item in kpt_trajectory_json:
                cont = True
                break
        if cont:
            continue

        with open(kpt_trajectory_json, 'r') as f:
            kpt_trajectory_dict = json.load(f)
            hook_urdf = kpt_trajectory_dict['file']
            hook_pos = kpt_trajectory_dict['hook_pose'][:3]
            hook_orientation = kpt_trajectory_dict['hook_pose'][3:]
            hook_id = p.loadURDF(hook_urdf, hook_pos, hook_orientation)
            hook_transform = get_matrix_from_pos_rot(hook_pos, hook_orientation)

            hook_name = kpt_trajectory_json.split('/')[-1].split('.')[0]

            print('processing {}'.format(kpt_trajectory_json))
            for traj_id, trajectory in enumerate(tqdm(kpt_trajectory_dict['trajectory'])):
                # print(f'rendering {trajectory_id}, num of waypoints = {len(trajectory)}')
                if traj_id >= 20:
                    break

                color = np.random.rand(1, 3)
                color = np.repeat(color, 3, axis=0)

                rgbs = []
                for wpt in trajectory[-100:]:

                    relative_transform = get_matrix_from_pos_rot(wpt[:3], wpt[3:])
                    kpt_transform = hook_transform @ relative_transform

                    object_trans = kpt_transform @ np.linalg.inv(object_contact_trans)
                    object_pos, object_rot = get_pos_rot_from_matrix(object_trans)
                    p.resetBasePositionAndOrientation(obj_id, object_pos, object_rot)

                    # draw_coordinate(kpt_transform, size=0.002, color=color)

                    wpt_trans_world = get_matrix_from_pose(hook_pose) @ get_matrix_from_pose(wpt)
                    wpt_world = get_pose_from_matrix(wpt_trans_world)
                    # draw_coordinate(wpt_world, size=0.001)

                    obj_pose = get_pose_from_matrix(wpt_trans_world @ np.linalg.inv(object_contact_trans))
                    p.resetBasePositionAndOrientation(obj_id, obj_pose[:3], obj_pose[3:])

                    cam_info = p.getDebugVisualizerCamera()
                    width = cam_info[0]
                    height = cam_info[1]
                    view_mat = cam_info[2]
                    proj_mat = cam_info[3]
                    img_info = p.getCameraImage(width, height, viewMatrix=view_mat, projectionMatrix=proj_mat)
                    rgbs.append(Image.fromarray(img_info[2]))

                rgbs[0].save(f"visualization/path_smoothness/{hook_name}-after-{traj_id}.gif", save_all=True, append_images=rgbs, duration=20, loop=0)

            while True:
                # key callback
                keys = p.getKeyboardEvents()            
                if ord('q') in keys and keys[ord('q')] & (p.KEY_WAS_TRIGGERED): 
                    break
            
            p.removeAllUserDebugItems()
            p.removeBody(hook_id)
    
    print('process completed...')        
        
    # while True:
    #     # key callback
    #     keys = p.getKeyboardEvents()            
    #     if ord('q') in keys and keys[ord('q')] & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN): 
    #         break


start_msg = \
'''
======================================================================================
this script will output the keypoint pose of a predefined objects into
[output_root]/[output_dir]/[obj_name].json

dependency :
- hook folder that contains [hook_root]/[hook_name]/base.urdf
- object folder that contains [hook_root]/[object_name]/base.urdf
- the source folder that contain [data_root]/[pivot_root]/[hook_name-object_name]/[hook_name-object_name].json
======================================================================================
'''

print(start_msg)

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--kptraj-root', '-kr', type=str, default='keypoint_trajectory')
    parser.add_argument('--kptraj-dir', '-kd', type=str, default='kptraj_all_smooth')
    args = parser.parse_args()

    main(args)