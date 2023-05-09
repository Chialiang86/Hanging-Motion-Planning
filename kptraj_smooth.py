import argparse, os, glob, json
import pybullet as p
import numpy as np

from tqdm import tqdm
from PIL import Image
from utils.bullet_utils import draw_coordinate, get_matrix_from_pose, get_pose_from_matrix, pose_6d_to_7d
from utils.bezier_util import Bezier


PENETRATION_THRESHOLD = 0.0003 # 0.00003528

def penetration_score(hook_id : int, obj_id : int):

    p.performCollisionDetection()

    contact_points = p.getContactPoints(bodyA=hook_id, bodyB=obj_id)
    # closest_points = p.getClosestPoints(bodyA=hook_id, bodyB=obj_id, distance=thresh)
    # within_thresh = 1 if len(closest_points) > 0 else 0

    penetration = 0.0
    for contact_point in contact_points:
        # contact distance, positive for separation, negative for penetration
        contact_distance = contact_point[8] 
        penetration = min(penetration, contact_distance) if contact_distance < 0 else 0.0
    
    # return penetration, within_thresh
    return -penetration

def trajectory_scoring(src_traj : list or np.ndarray, hook_id : int, obj_id : int, hook_pose : list or np.ndarray, obj_contact_pose : list or np.ndarray, visualize=False):

    if type(src_traj) == list:
        src_traj = np.asarray(src_traj)
    if type(obj_contact_pose) == list:
        obj_contact_pose = np.asarray(obj_contact_pose)

    assert obj_contact_pose.shape == (4, 4) or obj_contact_pose.shape == (7,), \
             f'the shape of obj_contact_pose must be (4, 4) or (7,), but got {obj_contact_pose.shape}'
    
    if obj_contact_pose.shape == (7,):
        obj_contact_trans = get_matrix_from_pose(obj_contact_pose)
    else :
        obj_contact_trans = obj_contact_pose
    
    hook_trans = get_matrix_from_pose(list(hook_pose))

    score = PENETRATION_THRESHOLD
    penetration_cost = 0.0
    color = np.random.rand(1, 3)
    color = np.repeat(color, 3, axis=0)
    rgbs = []
    cam_info = p.getDebugVisualizerCamera()
    for i, waypoint in enumerate(src_traj):

        relative_trans = get_matrix_from_pose(waypoint)
        world_trans = hook_trans @ relative_trans
        obj_trans = world_trans @ np.linalg.inv(obj_contact_trans)
        obj_pose = get_pose_from_matrix(obj_trans, pose_size=7)
        p.resetBasePositionAndOrientation(obj_id, obj_pose[:3], obj_pose[3:])

        # draw_coordinate(world_trans, size=0.002)

        penetration = penetration_score(hook_id=hook_id, obj_id=obj_id)
        penetration_cost += penetration

        if visualize:
            width = cam_info[0]
            height = cam_info[1]
            view_mat = cam_info[2]
            proj_mat = cam_info[3]
            img_info = p.getCameraImage(width, height, viewMatrix=view_mat, projectionMatrix=proj_mat)
            rgb = img_info[2]
            rgbs.append(Image.fromarray(rgb))

        # score -= waypoint_penetration
        # within_thresh_cnt += within_thresh

    penetration_cost /= src_traj.shape[0]
    score = score - penetration_cost
    
    # score /= within_thresh_cnt if within_thresh_cnt != 0 else 1.0
    # score += PENETRATION_THRESHOLD # hyper param, < 0 : not good
    # ratio = score / PENETRATION_THRESHOLD

    return score, rgbs

def main(args):
    input_dir = args.input_dir
    assert os.path.exists(input_dir), f'{input_dir} not exists'

    output_dir = args.output_dir
    os.makedirs(output_dir, exist_ok=True)

    # Create pybullet GUI
    # p.connect(p.GUI)
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.connect(p.DIRECT)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.1,
        cameraYaw=80,
        cameraPitch=-10,
        cameraTargetPosition=[0.0, 0.0, 0.0]
    )
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=150)
    sim_timestep = 1.0 / 240
    p.setTimeStep(sim_timestep)
    p.setGravity(0, 0, 0)

    traj_fnames = glob.glob(f'{input_dir}/Hook*.json')
    traj_fnames.sort()

    hook_pose = [
        0.0,
        0.0,
        0.0,
        4.329780281177466e-17,
        0.7071067811865475,
        0.7071067811865476,
        4.329780281177467e-17
    ]

    obj_urdf = 'models/geo_data/hanging_exp/daily_5/base.urdf'
    obj_json = f'keypoint_trajectory/kptraj_all_new/hanging_exp_daily_5.json'
    obj_dict = json.load(open(obj_json, 'r'))
    obj_contact_pose = obj_dict['contact_pose']
    obj_id = p.loadURDF(obj_urdf)

    # cam_info = p.getDebugVisualizerCamera()
    # width = cam_info[0]
    # height = cam_info[1]
    # view_mat = cam_info[2]
    # proj_mat = cam_info[3]

    wpt_num_to_smooth = 100

    for traj_fname in tqdm(traj_fnames):

        hook_name = traj_fname.split('/')[-1].split('.')[0]
        hook_urdf = f'models/hook_all_new/{hook_name}/base.urdf'
        hook_id = p.loadURDF(hook_urdf, hook_pose[:3], hook_pose[3:])

        traj_dict = json.load(open(traj_fname, 'r'))
        kptrajs = traj_dict['trajectory']
        kptrajs_out = traj_dict.copy()

        smooth_trajs = []
        for traj_id, kptraj in enumerate(tqdm(kptrajs)):

            kptraj_reverse = []
            for wpt in kptraj[::-1]:
                kptraj_reverse.append(wpt)
            
            t_step = 1.0 / (wpt_num_to_smooth - 1)
            intersections = np.arange(0.0, 1 + 1e-10, t_step)
            
            trajectory3d = np.asarray(kptraj_reverse)[:wpt_num_to_smooth, :3]
            trajectory_smooth3d = Bezier.Curve(intersections, trajectory3d)
            trajectory_smooth = np.asarray(kptraj_reverse)
            trajectory_smooth[:wpt_num_to_smooth, :3] = trajectory_smooth3d

            score, _ = trajectory_scoring(trajectory_smooth, hook_id, obj_id, hook_pose, obj_contact_pose, visualize=False)
            if score > 0:
                out_traj = trajectory_smooth.tolist()[::-1]
                smooth_trajs.append(out_traj)

        if len(smooth_trajs) == 0:
            print(f'no trajs in {traj_fname}')
            continue

        kptrajs_out['trajectory'] = smooth_trajs

        out_traj_json = f'{args.output_dir}/{hook_name}.json'
        json.dump(kptrajs_out, open(out_traj_json, 'w'), indent=4)

            # rgbs = []
            # for wpt in kptraj[::-1]:
                
            #     wpt_trans_world = get_matrix_from_pose(hook_pose) @ get_matrix_from_pose(wpt)
            #     wpt_world = get_pose_from_matrix(wpt_trans_world)
            #     draw_coordinate(wpt_world, size=0.001)

            #     obj_pose = get_pose_from_matrix(wpt_trans_world @ np.linalg.inv(get_matrix_from_pose(obj_contact_pose)))
            #     p.resetBasePositionAndOrientation(obj_id, obj_pose[:3], obj_pose[3:])

            #     img_info = p.getCameraImage(width, height, viewMatrix=view_mat, projectionMatrix=proj_mat)
            #     rgb = img_info[2]
            #     rgbs.append(Image.fromarray(rgb))
            # rgbs[0].save(f"visualization/path_smoothness/{hook_name}-before-{traj_id}.gif", save_all=True, append_images=rgbs, duration=20, loop=0)

            # rgbs = []
            # for wpt in trajectory_smooth[::-1]:
                
            #     wpt_trans_world = get_matrix_from_pose(hook_pose) @ get_matrix_from_pose(wpt)
            #     wpt_world = get_pose_from_matrix(wpt_trans_world)
            #     draw_coordinate(wpt_world, size=0.001)

            #     obj_pose = get_pose_from_matrix(wpt_trans_world @ np.linalg.inv(get_matrix_from_pose(obj_contact_pose)))
            #     p.resetBasePositionAndOrientation(obj_id, obj_pose[:3], obj_pose[3:])
            #     img_info = p.getCameraImage(width, height, viewMatrix=view_mat, projectionMatrix=proj_mat)
            #     rgb = img_info[2]
            #     rgbs.append(Image.fromarray(rgb))

            # rgbs[0].save(f"visualization/path_smoothness/{hook_name}-after-{traj_id}.gif", save_all=True, append_images=rgbs, duration=20, loop=0)

    
        p.removeBody(hook_id)

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--input_dir', '-id', type=str, default='keypoint_trajectory/kptraj_all_new/')
    parser.add_argument('--output_dir', '-od', type=str, default='keypoint_trajectory/kptraj_all_smooth_100/')
    args = parser.parse_args()
    main(args)