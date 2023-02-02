import argparse, json, os, glob, sys
from pathlib import Path
import numpy as np
import pybullet as p
from tqdm import tqdm

from utils.bullet_utils import draw_coordinate, get_matrix_from_pos_rot, get_pos_rot_from_matrix

def main(args):

    assert os.path.exists(args.kptraj_root), f'{args.kptraj_root} not exists' 

    kptraj_dir = f'{args.kptraj_root}/{args.kptraj_dir}' if args.kptraj_dir != '' else f'{args.kptraj_root}/keypoint_trajectory'
    assert os.path.exists(kptraj_dir), f'{kptraj_dir} not exists'

    # Create pybullet GUI
    physics_client_id = p.connect(p.GUI)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.3,
        cameraYaw=120,
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

    object_json = f'{kptraj_dir}/hanging_exp_daily_5.json'
    object_dict = json.load(open(object_json, 'r'))
    object_urdf = object_dict['file']
    object_contact_pose = object_dict['contact_pose']

    object_contact_trans = get_matrix_from_pos_rot(object_contact_pose[:3], object_contact_pose[3:])

    obj_id = p.loadURDF(object_urdf)

    kpt_trajectory_jsons = glob.glob(f'{kptraj_dir}/Hook_60*.json')
    kpt_trajectory_jsons.sort()

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

            print('processing {}'.format(kpt_trajectory_json))
            for trajectory_id, trajectory in tqdm(enumerate(kpt_trajectory_dict['trajectory'])):
                # print(f'rendering {trajectory_id}, num of waypoints = {len(trajectory)}')
                if trajectory_id >= 50:
                    break

                color = np.random.rand(1, 3)
                color = np.repeat(color, 3, axis=0)

                for waypoint in trajectory:

                    relative_transform = get_matrix_from_pos_rot(waypoint[:3], waypoint[3:])
                    kpt_transform = hook_transform @ relative_transform

                    object_trans = kpt_transform @ np.linalg.inv(object_contact_trans)
                    object_pos, object_rot = get_pos_rot_from_matrix(object_trans)
                    p.resetBasePositionAndOrientation(obj_id, object_pos, object_rot)

                    draw_coordinate(kpt_transform, size=0.001, color=color)

                break
    
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
    parser.add_argument('--kptraj-dir', '-kd', type=str, default='keypoint_trajectory_1104')
    args = parser.parse_args()

    main(args)