
import numpy as np
import pybullet as p

from scipy.spatial.transform import Rotation as R

from itertools import product
from pybullet_planning.interfaces.robots.collision import pairwise_link_collision
from pybullet_planning.interfaces.robots.body import set_pose
from pybullet_planning.interfaces.robots.link import get_all_links
from pybullet_planning.interfaces.debug_utils.debug_utils import draw_collision_diagnosis

def get_sample7d_fn(target_conf : list or tuple or np.ndarray, ratio_to_target=0.5):
    
    def sample7d_fn():
        rand_val = np.random.random()
        ret = None
        if rand_val < ratio_to_target:
            ret = target_conf
        else:
            pos_euler = []
            # pos_euler.append(np.random.uniform(-1, 1))
            # pos_euler.append(np.random.uniform(-1, 1))
            # pos_euler.append(np.random.uniform(-1, 1))

            pos_euler.append(np.random.uniform(0.3, 1.0)) # x
            pos_euler.append(np.random.uniform(-0.6, 1.0)) # y 
            pos_euler.append(np.random.uniform(0.5, 1.2)) # z
            for i in range(3, 6):
                pos_euler.append(np.random.uniform(-np.pi, np.pi))
            ret = pos_euler[:3] + list(R.from_rotvec(pos_euler[:3]).as_quat())
            
        return tuple(ret)
    return sample7d_fn

def get_distance7d_fn():

    def distance7d_fn(q1, q2):
        assert len(q1) == 7 and len(q2) == 7

        q1_pos = q1[:3]
        q2_pos = q2[:3]

        diff = np.linalg.norm(np.asarray(q1_pos) - np.asarray(q2_pos))
        return diff 
    return distance7d_fn

def get_extend7d_fn(resolution = 0.005):
    
    
    def extend7d_fn(q1, q2):
        assert len(q1) == 7 and len(q2) == 7

        q1_pos = np.asarray(q1[:3])
        q2_pos = np.asarray(q2[:3])
        q1_rot = np.asarray(q1[3:])
        q2_rot = np.asarray(q2[3:])

        d12 = q2_pos - q1_pos
        r12_rotvec = R.from_quat(q2_rot).as_rotvec() - R.from_quat(q1_rot).as_rotvec()

        # r12 = np.linalg.inv(R.from_quat(q1_rot).as_matrix()) @ R.from_quat(q2_rot).as_matrix() # from R1 -> R2
        # r12_rotvec = R.from_matrix(r12).as_rotvec()

        diff_q1_q2 = np.concatenate((d12, r12_rotvec))
        steps = int(np.ceil(np.linalg.norm(np.divide(diff_q1_q2, resolution), ord=1)))

        # generate collision check sequence
        yield q1
        for i in range(steps):
            positions6d = (i + 1) / steps * diff_q1_q2 + np.concatenate((q1_pos, R.from_quat(q1_rot).as_rotvec()))
            positions7d = tuple(positions6d[:3]) + tuple(R.from_rotvec(positions6d[3:]).as_quat())
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


            

        
