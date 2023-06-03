import torch
import json, os, inspect
import numpy as np
import open3d as o3d
import pybullet as p
import quaternion
import glob
import copy

from scipy.spatial.transform import Rotation as R

from itertools import product
from pybullet_planning.interfaces.robots.collision import pairwise_link_collision
from pybullet_planning.interfaces.robots.body import set_pose
from pybullet_planning.interfaces.robots.link import get_all_links
from pybullet_planning.interfaces.debug_utils.debug_utils import draw_collision_diagnosis

from utils.bullet_utils import get_matrix_from_pose, get_pose_from_matrix, get_matrix_from_pos_rot

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

def get_matrix_from_pose(pose : list or tuple or np.ndarray) -> np.ndarray:
    assert len(pose) == 6 or len(pose) == 7, f'pose must contain 6 or 7 elements, but got {len(pose)}'
    pos_m = np.asarray(pose[:3])
    rot_m = np.identity(3)

    if len(pose) == 6:
        rot_m = R.from_rotvec(pose[3:]).as_matrix()
    elif len(pose) == 7:
        rot_m = R.from_quat(pose[3:]).as_matrix()
            
    ret_m = np.identity(4)
    ret_m[:3, :3] = rot_m
    ret_m[:3, 3] = pos_m

    return ret_m

def get_pose_from_matrix(matrix : list or tuple or np.ndarray, 
                        pose_size : int = 7) -> np.ndarray:

    mat = np.array(matrix)
    assert mat.shape == (4, 4), f'pose must contain 4 x 4 elements, but got {mat.shape}'
    
    pos = matrix[:3, 3]
    rot = None

    if pose_size == 6:
        rot = R.from_matrix(matrix[:3, :3]).as_rotvec()
    elif pose_size == 7:
        rot = R.from_matrix(matrix[:3, :3]).as_quat()
            
    pose = list(pos) + list(rot)

    return np.array(pose)

def get_sample7d_fn(target_conf : list or tuple or np.ndarray,
                    low_limit : list or tuple or np.ndarray,
                    high_limit : list or tuple or np.ndarray, 
                    ratio_to_target=0.1):

    assert len(low_limit) == 3 and len(high_limit) == 3, 'illegal size of limit, len(limit) must be 3'

    # Add on 2023/3/5 template trajectories
    template_dir = 'keypoint_trajectory/template'
    template_paths = glob.glob(f'{template_dir}/*.json')
    template_paths.sort()

    hook_pose = [
        0.5,
        -0.1,
        1.3,
        4.329780281177466e-17,
        0.7071067811865475,
        0.7071067811865476,
        4.329780281177467e-17
    ]
    hook_trans = get_matrix_from_pose(hook_pose)

    template_trajs = []
    for template_path in template_paths:
        traj = json.load(open(template_path, 'r'))['trajectory'][0][::-1][:50] # using first trajectory
        template_wpts = []
        for template_wpt in traj:
            world_wpts = get_pose_from_matrix(hook_trans @ get_matrix_from_pose(template_wpt), pose_size=6)
            template_wpts.append(list(world_wpts))

        template_trajs.append(template_wpts)

    deg2rad = np.pi / 180
    
    def sample7d_fn():
        rand_val = np.random.random()
        ret = None
        if rand_val < ratio_to_target:
            ret = target_conf
        else:
            pos_euler = []
            pos_euler.append(np.random.uniform(low_limit[0], high_limit[0])) # x
            pos_euler.append(np.random.uniform(low_limit[1], high_limit[1])) # y 
            pos_euler.append(np.random.uniform(low_limit[2], high_limit[2])) # z
            for i in range(3, 6):
                pos_euler.append(np.random.uniform(-np.pi / 2, np.pi / 2)) # row

            # # 20 * np.pi / 180, 90 * np.pi / 180, -60 * np.pi / 180
            # pos_euler.append( 0 * np.pi / 180 + np.random.uniform( -60 * np.pi / 180, 60 * np.pi / 180)) # roll
            # pos_euler.append( 0 * np.pi / 180 + np.random.uniform(-120 * np.pi / 180, 60 * np.pi / 180)) # pitch
            # pos_euler.append(90 * np.pi / 180 + np.random.uniform( -60 * np.pi / 180, 60 * np.pi / 180)) # yew
            ret = pos_euler[:3] + list(R.from_rotvec(pos_euler[3:]).as_quat())

            # template_traj_id = np.random.randint(0, len(template_trajs))
            # template_traj = template_trajs[template_traj_id]

            # template_wpt_id = np.random.randint(0, len(template_traj))
            # template_wpt = template_traj[template_wpt_id]

            # template_pos = np.asarray(template_wpt[:3])
            # template_euler = np.asarray(template_wpt[3:])

            # template_pos += np.random.uniform([-0.04, -0.04, -0.03], [0.04, 0.04, 0.07])
            # template_euler += np.random.uniform(
            #                     [-20 * deg2rad, -50 * deg2rad, -20 * deg2rad], 
            #                     [ 20 * deg2rad,  50 * deg2rad,  20 * deg2rad]
            #                 )
            # template_euler = ((template_euler + np.pi) % (2 * np.pi)) - np.pi
            # template_quat = R.from_rotvec(template_euler).as_quat()

            # ret = list(np.hstack((template_pos, template_quat)))
            
        return tuple(ret)
    return sample7d_fn

def get_distance7d_fn():

    def distance7d_fn(q1, q2):
        assert len(q1) == 7 and len(q2) == 7

        q1_pos = np.asarray(q1[:3])
        q2_pos = np.asarray(q2[:3])
        q1_rot = R.from_quat(np.asarray(q1[3:])).as_rotvec()
        q2_rot = R.from_quat(np.asarray(q2[3:])).as_rotvec()

        diff_pos = q1_pos - q2_pos
        diff_rot = np.array([min((r1 - r2) ** 2, (r2 - r1) ** 2) for r1, r2 in zip(q1_rot, q2_rot)])

        return 1.0 * np.sum(diff_pos ** 2) + 2.0 * np.sum(diff_rot ** 2)
    return distance7d_fn


def xyzw2wxyz(quat : np.ndarray):
    assert len(quat) == 4, f'quaternion size must be 4, got {len(quat)}'
    return np.asarray([quat[3], quat[0], quat[1], quat[2]])

def wxyz2xyzw(quat : np.ndarray):
    assert len(quat) == 4, f'quaternion size must be 4, got {len(quat)}'
    return np.asarray([quat[1], quat[2], quat[3], quat[0]])
    
def get_extend7d_fn(resolution = 0.001):
    
    def extend7d_fn(q1, q2):
        assert len(q1) == 7 and len(q2) == 7

        # q1_pos = np.asarray(q1[:3])
        # q2_pos = np.asarray(q2[:3])
        # q1_rot = np.asarray(q1[3:])
        # q2_rot = np.asarray(q2[3:])

        # d12 = q2_pos - q1_pos

        # r12_rotvec = R.from_quat(q2_rot).as_rotvec() - R.from_quat(q1_rot).as_rotvec()

        # r12 = np.linalg.inv(R.from_quat(q1_rot).as_matrix()) @ R.from_quat(q2_rot).as_matrix() # from R1 -> R2
        # r12_rotvec = R.from_matrix(r12).as_rotvec()

        # diff_q1_q2 = np.concatenate((d12, r12_rotvec))
        # steps = int(np.ceil(np.linalg.norm(np.divide(diff_q1_q2, resolution), ord=2)))

        d12 = np.asarray(q2[:3]) - np.asarray(q1[:3])
        r12 = np.asarray(q2[3:]) - np.asarray(q1[3:])
        diff_q1_q2 = np.concatenate((d12, r12))
        steps = int(np.ceil(np.linalg.norm(np.divide(diff_q1_q2, resolution), ord=2)))
        obj_init_quat = quaternion.as_quat_array(xyzw2wxyz(q1[3:]))
        obj_tgt_quat = quaternion.as_quat_array(xyzw2wxyz(q2[3:]))

        # generate collision check sequence
        yield q1
        for i in range(steps):
            ratio = (i + 1) / steps
            pos = ratio * d12 + np.asarray(q1[:3])
            quat = quaternion.slerp_evaluate(obj_init_quat, obj_tgt_quat, ratio)
            quat = wxyz2xyzw(quaternion.as_float_array(quat))
            positions7d = tuple(pos) + tuple(quat)

            # positions6d = (i + 1) / steps * diff_q1_q2 + np.concatenate((q1_pos, R.from_quat(q1_rot).as_rotvec()))
            # positions7d = tuple(positions6d[:3]) + tuple(R.from_rotvec(positions6d[3:]).as_quat())
            yield positions7d

        # # testing code
        # diff_quat = p.getDifferenceQuaternion(q1[3:], q2[3:])
        # diff_rotvec = R.from_quat(diff_quat).as_rotvec()
        # print('diff = {}'.format(r12_as_rotvec))

        # # rotmat = R.from_quat(diff_quat).as_matrix()
        # rotmat1 = R.from_quat(q1[3:]).as_matrix()
        # rotmat2 = R.from_quat(q2[3:]).as_matrix()
        # print('{} = {}'.format(rotmat1 @ r12, rotmat2))
    
    return extend7d_fn


def expand_links(body):
    """expand all links of a body

    TODO: [REFACTOR] move to body or link modules?

    Parameters
    ----------
    body : int
        [description]

    Returns
    -------
    body : int
        [description]
    links : list of int
        [description]
    """
    body, links = body if isinstance(body, tuple) else (body, None)
    if links is None:
        links = get_all_links(body)
    return body, links

def pairwise_link_collision_info(physicsClientId, body1, link1, body2, link2=-1.0, max_distance=0.0, **kwargs): # 10000
    return p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance,
                              linkIndexA=link1, linkIndexB=link2, physicsClientId=physicsClientId)


def set_pose(physicsClientId, body, pose):
    point = pose[:3]
    quat = pose[3:]
    p.resetBasePositionAndOrientation(body, point, quat, physicsClientId=physicsClientId)

def get_collision7d_fn(physicsClientId, body, obstacles=[], attachments=[], disabled_collisions={}, **kwargs):
    """get collision checking function collision_fn(joint_values) -> bool for a floating body (no movable joint).

    Parameters
    ----------
    body : int
        the main moving body (usually the robot). We refer to this body as 'the main body'
        in this function's docstring.
    obstacles : list of int
        body indices for collision objects, by default []
    attachments : list of Attachment, optional
        list of attachment, by default []
    disabled_collisions : set of tuples, optional
        list of tuples for specifying disabled collisions, the tuple must be of the following format:
            ((int, int), (int, int)) : (body index, link index), (body index, link index)
        If the body considered is a single-link (floating) body, assign the link index to BASE_LINK.
        reversing the order of the tuples above is also acceptable, by default {}

    Returns
    -------
    function handle
        collision_fn: (conf, diagnosis) -> False if no collision found, True otherwise.
        if need diagnosis information for the collision, set diagnosis to True will help you visualize
        which link is colliding to which.
    """

    attached_bodies = [attachment.child for attachment in attachments]
    moving_bodies = [body] + attached_bodies
    # * body pairs
    check_body_pairs = list(product(moving_bodies, obstacles))  # + list(combinations(moving_bodies, 2))
    check_body_link_pairs = []
    for body1, body2 in check_body_pairs:
        body1, links1 = expand_links(body1)
        body2, links2 = expand_links(body2)
        bb_link_pairs = product(links1, links2)
        for bb_links in bb_link_pairs:
            bbll_pair = ((body1, bb_links[0]), (body2, bb_links[1]))
            if bbll_pair not in disabled_collisions and bbll_pair[::-1] not in disabled_collisions:
                check_body_link_pairs.append(bbll_pair)

    def collision7d_fn(pose, diagnosis=False):
        set_pose(physicsClientId, body, pose)
        # * body - body check
        for (body1, link1), (body2, link2) in check_body_link_pairs:
            if pairwise_link_collision(body1, link1, body2, link2, **kwargs):
                if diagnosis:
                    # warnings.warn('moving body - body collision!', UserWarning)
                    cr = pairwise_link_collision_info(physicsClientId, body1, link1, body2, link2)
                    draw_collision_diagnosis(cr)
                return True
        return False
    return collision7d_fn

def get_collision7d_nce_fn(physicsClientId, body_pcd=None, obstacle_pcd=None, hook_trans=None, nce_weights=None):

    obj_pcd_static = copy.deepcopy(body_pcd) # (4 x N)
    hook_pcd_static = copy.deepcopy(obstacle_pcd) # (4 x N)
    hook_points = hook_pcd_static.T[:, :3]
    hook_points_down_4d = np.hstack((hook_points, np.ones((hook_points.shape[0], 1))))

    def collision7d_nce_fn(pose, diagnosis=False):
        obj_trans = get_matrix_from_pose(pose)
        obj_points = (obj_trans @ obj_pcd_static).T[:,:3]

        obj_points_down_4d  = np.hstack((obj_points, np.zeros((obj_points.shape[0], 1))))
        obj_points_down_4d[:, :3] -= hook_trans[:3, 3]
        
        pared_points = np.vstack((hook_points_down_4d, obj_points_down_4d)).astype(np.float32)

        # tmp_pcd = o3d.geometry.PointCloud()
        # colors = np.zeros(pared_points[:,:3].shape)
        # colors[:hook_points_down_4d.shape[0]] = np.array([1, 0, 0])
        # colors[hook_points_down_4d.shape[0]:] = np.array([0, 0, 1])
        # tmp_pcd.points = o3d.utility.Vector3dVector(pared_points[:, :3])
        # tmp_pcd.colors = o3d.utility.Vector3dVector(colors)
        # coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        # o3d.visualization.draw_geometries([coor, tmp_pcd])

        point_batch = torch.from_numpy(pared_points).unsqueeze(0).to('cuda').contiguous()
        pred = nce_weights.inference(point_batch) 

        return pred[0,0].item() > 0.95

        # if collision:
        #     return True
        # return False
    return collision7d_nce_fn


            

        
