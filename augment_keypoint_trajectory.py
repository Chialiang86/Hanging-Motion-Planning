import argparse, os, glob, json, time
import quaternion
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R

from utils.bullet_utils import get_matrix_from_pose, get_pose_from_matrix, draw_coordinate, xyzw2wxyz, wxyz2xyzw
from utils.motion_planning_utils import get_sample7d_fn, get_distance7d_fn, get_extend7d_fn, get_collision7d_fn
from pybullet_planning.motion_planners.meta import smooth_path

MAX_TRAJECTORY_SCORE = 0.001

def down_sample_trajectory(traj : list or np.ndarray,
                            frequency : int = 5) -> np.ndarray :
    
    if type(traj) == list:
        traj = np.asarray(traj)

    assert traj.shape[1] == 7, f'waypoint must be in 7d, but got {traj.shape[1]}'
    
    down_traj = []
    for i in range(0, traj.shape[0], frequency):
        down_traj.append(traj[i])
    
    return np.array(down_traj)

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

def get_dense_trajectory(src_traj : list or np.ndarray, resolution : float = 0.001):

    dense_traj = []
    for i in range(len(src_traj) - 1):
        dense_waypoints = get_dense_waypoints(src_traj[i], src_traj[i+1], resolution=resolution)
        dense_traj.extend(dense_waypoints)
    
    return dense_traj

def augment_next_waypoint(waypoint : list or np.ndarray,
                                direction_vec : list or np.ndarray,
                                length : float,
                                aug_num : int=10,
                                # add to normalized vector
                                noise_pos : float=0.2,
                                # in degree
                                noise_rot : float=1) -> np.ndarray:

    assert len(waypoint) == 7 and len(direction_vec) == 3, \
        f'length of waypoint should be 7 and direction_vec should be 3 but got {len(waypoint)} and {len(direction_vec)}'
    
    base_pos    = np.asarray(waypoint[:3]) # + np.asarray([0.0, 0.0, 0.02]) for testing
    base_rotvec = R.from_quat(waypoint[3:]).as_rotvec()

    deg_to_rad = np.pi / 180.0

    pos_low_limit  = np.full((3,), -noise_pos)
    pos_high_limit = np.full((3,),  noise_pos)
    rot_low_limit  = np.full((3,), -noise_rot * deg_to_rad)
    rot_high_limit = np.full((3,),  noise_rot * deg_to_rad)

    step_direction_vec = direction_vec + np.random.uniform(pos_low_limit, pos_high_limit, (aug_num, 3))
    step_direction_vec /= np.linalg.norm(step_direction_vec, ord=2)
    step_pos = base_pos + length * step_direction_vec
    step_rotvec = base_rotvec + np.random.uniform(rot_low_limit, rot_high_limit, (aug_num, 3)) \
                    if (base_rotvec <  np.pi - noise_rot * deg_to_rad).all() and \
                       (base_rotvec > -np.pi + noise_rot * deg_to_rad).all() \
                    else base_rotvec
    step_quat = R.from_rotvec(step_rotvec).as_quat()
    step_pose = np.hstack((step_pos, step_quat))

    return step_pose

def waypoint_score(hook_id : int, obj_id : int):

    thresh = 0.01

    p.performCollisionDetection()

    # for i in range(5):
    #     time.sleep(0.001)
    #     p.stepSimulation()

    contact_points = p.getContactPoints(bodyA=hook_id, bodyB=obj_id)
    closest_points = p.getClosestPoints(bodyA=hook_id, bodyB=obj_id, distance=thresh)
    within_thresh = 1 if len(closest_points) > 0 else 0

    penetration = 0.0
    for contact_point in contact_points:
        # contact distance, positive for separation, negative for penetration
        contact_distance = contact_point[8] 
        penetration += -contact_distance if contact_distance < 0 else 0.0

    # print(f'num contact points = {len(contact_points)}, penetration score = {penetration}')
    
    return penetration, within_thresh


def trajectory_scoring(src_traj : list or np.ndarray, hook_id : int, obj_id : int, obj_contact_pose : list or np.ndarray):

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
    
    hook_pos, hook_rot = p.getBasePositionAndOrientation(hook_id)
    hook_pose = list(hook_pos) + list(hook_rot)
    hook_trans = get_matrix_from_pose(hook_pose)

    score = 0.0 
    with_thresh_cnt = 0.0
    color = np.random.rand(1, 3)
    color = np.repeat(color, 3, axis=0)
    for waypoint in src_traj:

        relative_trans = get_matrix_from_pose(waypoint)
        world_trans = hook_trans @ relative_trans
        obj_trans = world_trans @ np.linalg.inv(obj_contact_trans)
        obj_pose = get_pose_from_matrix(obj_trans, pose_size=7)
        p.resetBasePositionAndOrientation(obj_id, obj_pose[:3], obj_pose[3:])

        # draw_coordinate(world_trans, size=0.001, color=color)
        
        waypoint_penetration, with_thresh = waypoint_score(hook_id=hook_id, obj_id=obj_id)
        score -= waypoint_penetration
        with_thresh_cnt += with_thresh
    
    score /= with_thresh_cnt if with_thresh_cnt != 0 else 1.0
    score += MAX_TRAJECTORY_SCORE # hyper param, < 0 : not good
    ratio = score / MAX_TRAJECTORY_SCORE if score > 0 else -1.0
    
    return ratio

def smooth_augment_path(src_traj : list, 
                        physicsClientId : int, obj_id : int, hook_id : int, obj_contact_trans : list or np.ndarray):

    if obj_contact_trans.shape == (7,):
        obj_contact_trans = get_matrix_from_pose(obj_contact_trans)
    else :
        obj_contact_trans = obj_contact_trans

    hook_pos, hook_rot = p.getBasePositionAndOrientation(hook_id)
    hook_pose = list(hook_pos) + list(hook_rot)
    hook_trans = get_matrix_from_pose(hook_pose)

    # for path smoothing
    extend7d_fn = get_extend7d_fn()
    collision7d_fn = get_collision7d_fn(physicsClientId, obj_id, obstacles=[hook_id])

    # object pose in world coordinate
    obj_traj = []
    for waypoint in src_traj:

        relative_trans = get_matrix_from_pose(waypoint)
        world_trans = hook_trans @ relative_trans
        obj_trans = world_trans @ np.linalg.inv(obj_contact_trans)
        obj_pose = get_pose_from_matrix(obj_trans, pose_size=7)

        obj_traj.append(obj_pose)

    aug_smooth_traj = smooth_path(obj_traj, extend_fn=extend7d_fn, collision_fn=collision7d_fn)

    # keypoint pose in hook coordinate
    kpt_traj = []
    for waypoint in aug_smooth_traj:
        
        obj_trans = get_matrix_from_pose(waypoint)
        kpt_trans = obj_trans @ obj_contact_trans
        relative_trans = np.linalg.inv(hook_trans) @ kpt_trans
        kpt_pose = get_pose_from_matrix(relative_trans, pose_size=7)

        kpt_traj.append(kpt_pose)

    return kpt_traj

def augment_trajectory_7d(src_traj : list or np.ndarray, 
                            physicsClientId : int, obj_id : int, hook_id : int, contact_trans : list or np.ndarray,
                            aug_num : int=10, 
                            noise_pos : float=0.2, # add to normalized vector
                            noise_rot : float=1, # in degree
                            down_sample_frequency : int=5):

    print(f'trajectory length = {len(src_traj)}')

    if type(src_traj) == list:
        src_traj = np.asarray(src_traj)
    
    assert src_traj.shape[1] == 7, f'waypoint must be in 7d, but got {src_traj.shape[1]}'

    # using down sampled array can generate more diverse waypoints
    src_traj_down = down_sample_trajectory(src_traj, frequency=down_sample_frequency)
    
    src_traj_rev = np.flip(src_traj_down, axis=0) # tgt -> keypose
    traj_len = src_traj_rev.shape[0]

    # relative vector
    relative_pos = src_traj_rev[1:, :3] - src_traj_rev[:-1, :3]
    dist_vec = np.linalg.norm(relative_pos, ord=2, axis=1).reshape(-1, 1)
    direction_vec = np.divide(relative_pos, dist_vec) # normalized vector
    
    # aug_num x num_waypoints x 7
    aug_trajs_rev = np.zeros((aug_num,) + src_traj_rev.shape) 
    aug_trajs_rev[:, 0, :] = src_traj_rev[0]

    for i in range(traj_len - 1):
        # augment next waypoint to `aug_num` trajectories
        next_waypoint = augment_next_waypoint(src_traj_rev[i], direction_vec[i], length=dist_vec[i, 0],
                                        aug_num=aug_num, noise_pos=noise_pos, noise_rot=noise_rot)
        aug_trajs_rev[:, i+1, :] = next_waypoint

    aug_trajs = np.flip(aug_trajs_rev, axis=1)

    # get dense waypoints using the same resolution as src_traj
    aug_dense_trajs = []
    for i in range(aug_trajs.shape[0]):
        aug_smooth_traj = smooth_augment_path(aug_trajs[i], physicsClientId, obj_id, hook_id, contact_trans)
        aug_dense_traj = get_dense_trajectory(aug_smooth_traj)
        score = trajectory_scoring(aug_dense_traj, hook_id, obj_id, obj_contact_pose=contact_trans)
        
        print(f'aug id : {i}, score : {score}')
        if score > 0:
            aug_dense_trajs.append(aug_dense_traj)
    
    return aug_dense_trajs

def main(args):
    

    dir_postfix = args.dir_postfix
    input_dir = f'keypoint_trajectory_{dir_postfix}'
    assert os.path.exists(input_dir), f'{input_dir} not exists'

    all_trajectory_files = glob.glob(f'{input_dir}/Hook*.json')
    trajectory_files = []
    for trajectory_file in all_trajectory_files:
        if 'aug' not in trajectory_file:
            trajectory_files.append(trajectory_file)

    pivot_file = f'{input_dir}/hanging_exp_daily_5.json'

    assert os.path.exists(pivot_file), f'{pivot_file} not exists'

    # Create pybullet GUI
    physicsClientId = p.connect(p.GUI)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.2,
        cameraYaw=120,
        cameraPitch=-10,
        cameraTargetPosition=[0.5, -0.05, 1.27]
    )
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=150)
    sim_timestep = 1.0 / 240
    p.setTimeStep(sim_timestep)
    p.setGravity(0, 0, 0)

    # aug_num
    aug_num = args.aug_num

    # daily_5
    f = open(pivot_file, 'r')
    pivot_json = json.load(f)
    f.close()

    contact_pose = pivot_json['contact_pose']
    contact_trans = get_matrix_from_pose(contact_pose)
    obj_id = p.loadURDF(pivot_json['file'])

    for trajectory_file in trajectory_files:
        
        f = open(trajectory_file, 'r')
        trajectory_json = json.load(f)
        f.close()

        hook_urdf = trajectory_json['file']
        hook_pose = trajectory_json['hook_pose']
        hook_id = p.loadURDF(hook_urdf, hook_pose[:3], hook_pose[3:])

        # new trajectory file
        aug_trajectory_json = {
            'file': trajectory_json['file'],
            'trajectory': [],
            'hook_pose': trajectory_json['hook_pose'],
        }

        for i, trajectory in enumerate(trajectory_json['trajectory']):
            print(f'processing trajectory[{i}] ...')
            color = np.random.rand(1, 3)
            color = np.repeat(color, 3, axis=0)
            
            aug_trajs = augment_trajectory_7d(trajectory, physicsClientId, obj_id, hook_id, contact_trans,
                                                aug_num=aug_num, noise_pos=0.8, noise_rot=1, 
                                                down_sample_frequency=10)
            
            for aug_traj in aug_trajs:
                aug_trajectory_json['trajectory'].append(list(aug_traj))
            
            p.removeAllUserDebugItems()

    
        # write to file
        path = f'{os.path.splitext(trajectory_file)[0]}_aug.json'
        f = open(path, 'w')
        json.dump(aug_trajectory_json, f, indent=4)
        f.close()
        print(f'has been written to {path} ...')
        
        p.removeBody(hook_id)

    # while True:
    #     # key callback
    #     keys = p.getKeyboardEvents()            
    #     if ord('q') in keys and keys[ord('q')] & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN): 
    #         break

if __name__=="__main__":
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir-postfix', '-dp', type=str, default='1104')
    parser.add_argument('--aug-num', '-an', type=int, default=10)
    args = parser.parse_args()
    
    main(args)