import argparse, json, numpy, os, glob, quaternion
import numpy as np
import open3d as o3d

from tqdm import tqdm
from scipy.spatial.transform import Rotation as R
from utils.bullet_utils import get_pose_from_matrix, get_matrix_from_pose, wxyz2xyzw, xyzw2wxyz

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

def main(args):

    assert os.path.exists(args.kptraj_root), f'{args.kptraj_root} not exists'

    # data dir
    data_dir = f'data/{args.data_root}'
    assert os.path.exists(data_dir), f'{data_dir} not exists'

    # dir name
    kptraj_dir_name = f'{args.kptraj_root}/{args.kptraj_dir}' if args.kptraj_dir != '' else 'keypoint_trajectory'
    assert os.path.exists(kptraj_dir_name), f'{kptraj_dir_name} not exists'

    hook_jsons = glob.glob(f'{kptraj_dir_name}/Hook*.json')
    hook_jsons.sort()

    refined_trans = None
    object_dict = json.load(open(f'{kptraj_dir_name}/hanging_exp_daily_5.json', 'r'))
    # the contact pose is close to the object surface
    if args.refine_pose == True:
        assert 'contact_pose_modified' in object_dict.keys()
        object_original_pose = object_dict['contact_pose']
        object_modified_pose = object_dict['contact_pose_modified']

        refined_trans = \
            np.linalg.inv(get_matrix_from_pose(object_original_pose)) @ \
            get_matrix_from_pose(object_modified_pose) 

    # data : hook-object pair
    # input_jsons = glob.glob(f'{data_dir}/*/Hook_*-hanging_exp_daily_5.json')

    for hook_json in tqdm(hook_jsons):
        hook_dict = json.load(open(hook_json, 'r'))
        
        hook_trajs = hook_dict['trajectory']

        modified_json_dict = {
            "file": hook_dict["file"],
            "trajectory": [],
            "hook_pose": hook_dict["hook_pose"]
        }

        # load obj_hanging_pose from hook-object info
        hook_name = os.path.splitext(hook_json)[0].split('/')[-1].split('_aug')[0]
        hook_obj_path = f'{data_dir}/{hook_name}-hanging_exp/{hook_name}-hanging_exp_daily_5.json'
        hook_obj_dict = json.load(open(hook_obj_path, 'r'))
        contact_info = hook_obj_dict['contact_info'][0]
        obj_hanging_pose = contact_info['obj_pose'] # object hanging pose

        # last waypoint to contact pose on the object
        obj_contact_trans_world = get_matrix_from_pose(obj_hanging_pose) @ get_matrix_from_pose(object_dict['contact_pose'])
        obj_contact_pose_hook = get_pose_from_matrix(
            np.linalg.inv(get_matrix_from_pose(hook_dict["hook_pose"])) @ obj_contact_trans_world
        )

        # contact pose on the object to the contact pose on the hook
        hook_affordance_file = f'{hook_dict["file"][:-9]}affordance.npy' # .../base.urdf => .../affordance.npy
        hook_points = np.load(hook_affordance_file)[:, :3]
        hook_contact_point = hook_points[0] # the first 3 dim of the first point
        hook_contact_pose_hook = np.hstack((hook_contact_point, obj_contact_pose_hook[3:]))

        # pcd = o3d.geometry.PointCloud()
        # hook_colors = np.zeros(hook_points.shape)
        # hook_colors[0] = np.asarray([1, 0, 0])
        # pcd.points = o3d.utility.Vector3dVector(hook_points)
        # pcd.colors = o3d.utility.Vector3dVector(hook_colors)
        # o3d.visualization.draw_geometries([pcd])

        for hook_traj in hook_trajs:

            modified_wpts = []

            for wpt in hook_traj:

                if refined_trans is not None:
                    modified_wpt = get_pose_from_matrix(get_matrix_from_pose(wpt) @ refined_trans)
                    modified_wpts.append(list(modified_wpt))
                else :
                    modified_wpts.append(wpt)

            # last_to_contact_point = get_dense_waypoints(modified_wpts[-1], obj_contact_pose_hook, resolution=0.001)
            # modified_wpts.extend(last_to_contact_point)

            # contact_point_object_hook = get_dense_waypoints(obj_contact_pose_hook, hook_contact_pose_hook, resolution=0.001)
            # modified_wpts.extend(contact_point_object_hook)

            contact_point_object_hook = get_dense_waypoints(modified_wpts[-1], hook_contact_pose_hook, resolution=0.001)
            modified_wpts.extend(contact_point_object_hook)

            modified_json_dict["trajectory"].append(modified_wpts)

        output_fname = f'{hook_json[:-5]}_modified.json'
        # output_fname = hook_json
        json.dump(modified_json_dict, open(output_fname, 'w'), indent=4)
        print(f'{output_fname} saved')

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--data-root', '-dr', type=str, default='data')
    parser.add_argument('--kptraj-root', '-kr', type=str, default='keypoint_trajectory')
    parser.add_argument('--kptraj-dir', '-kd', type=str, default='')
    parser.add_argument('--refine-pose', '-rp', action="store_true")

    args = parser.parse_args()
    main(args)