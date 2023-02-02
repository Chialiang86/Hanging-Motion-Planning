import argparse, glob, os, json
import numpy as np
import xml.etree.ElementTree as ET

from tqdm import tqdm
from utils.bullet_utils import get_matrix_from_pose, get_pose_from_matrix
from scipy.spatial.transform import Rotation as R

def main(args):
    kptraj_dir = f'{args.kptraj_root}/{args.kptraj_dir}'
    assert os.path.exists(kptraj_dir), f'{kptraj_dir} not exists'

    # config output dir
    output_dir = f'{args.kptraj_root}/{args.kptraj_dir}_origin'
    os.makedirs(output_dir, exist_ok=True)

    kptraj_paths = glob.glob(f'{kptraj_dir}/Hook*.json')
    kptraj_paths.sort()

    for kptraj_path in tqdm(kptraj_paths):

        kptraj_dict = json.load(open(kptraj_path, 'r'))

        kptraj_dict_origin = {
            'file': kptraj_dict['file'],
            'trajectory': [],
            'hook_pose': None
        }

        hook_path = kptraj_dict['file'] # the urdf path of the hook

        # extract the center of the urdf object
        tree = ET.parse(hook_path)
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

        hook_pose_center = np.asarray(kptraj_dict['hook_pose'])
        hook_trans_center = get_matrix_from_pose(hook_pose_center)
        
        hook_pose_origin = np.array(kptraj_dict['hook_pose'])
        hook_pose_mat = R.from_quat(hook_pose_origin[3:]).as_matrix()
        pos_align_to_origin = center @ hook_pose_mat.T
        hook_pose_origin[:3] -= pos_align_to_origin # relative to center => relative to origin shape = (1, 3)
        hook_trans_origin = get_matrix_from_pose(hook_pose_origin)
        kptraj_dict_origin['hook_pose'] = hook_pose_origin.tolist()

        hook_trajectories = kptraj_dict['trajectory']

        for hook_trajectory in hook_trajectories:
            
            kptraj_origin = []
            for wpt_center in hook_trajectory:

                wpt_center_trans = get_matrix_from_pose(wpt_center)
                wpt_origin_trans = np.linalg.inv(hook_trans_origin) @ hook_trans_center @ wpt_center_trans
                wpt_origin = get_pose_from_matrix(wpt_origin_trans)
                kptraj_origin.append(wpt_origin.tolist())
            
            kptraj_dict_origin['trajectory'].append(kptraj_origin)


        # write to output_dir
        output_subpath = os.path.split(kptraj_path)[1]
        output_path = f'{output_dir}/{output_subpath}'
        json.dump(kptraj_dict_origin, open(output_path, 'w'), indent=4)

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--kptraj_root', '-kr', default='keypoint_trajectory')
    parser.add_argument('--kptraj_dir', '-kd', default='keypoint_trajectory_1104')
    args = parser.parse_args()
    main(args)