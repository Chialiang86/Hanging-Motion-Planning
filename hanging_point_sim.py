# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.

import os, inspect
import argparse
import glob
import time
import json
from matplotlib.pyplot import draw
import numpy as np
from sklearn.cluster import DBSCAN
import os

from utils.bullet_utils import get_matrix_from_pos_rot, draw_coordinate
from scipy.spatial.transform import Rotation as R
import xml.etree.ElementTree as ET

from tqdm import tqdm
from utils.renderer import Renderer

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print(currentdir)
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import pybullet as p
import pybullet_data
from pybullet_robot_envs.envs.panda_envs.panda_env import pandaEnv


def reset_pose(obj_id, x_offset=0.1, y_offset=0., z_offset=1.):
    x = (np.random.rand() - 0.5) * 0.1 + x_offset
    y = (np.random.rand() - 0.5) * 0.4 + y_offset
    z = (np.random.rand() - 0.5) * 0.4 + z_offset

    roll = np.random.rand() * np.pi * 2
    pitch = np.random.rand() * np.pi * 2
    yaw = np.random.rand() * np.pi * 2
    p.setGravity(0, 0, 0)
    p.resetBasePositionAndOrientation(
        obj_id,
        [x, y, z],
        p.getQuaternionFromEuler([roll, pitch, yaw]))

# def load_obj(obj_file, pos=[0,0,0.4], rot=[np.pi/2, 0, np.pi/2], shift=[0, -0.02, 0], scale=[1, 1, 1]):
#     visual_shape_id = p.createVisualShape(
#         shapeType=p.GEOM_MESH,
#         fileName=obj_file,
#         rgbaColor=[1, 1, 1, 1],
#         specularColor=[0.4, 0.4, 0],
#         visualFramePosition=shift,
#         meshScale=scale,
#     )
#     collision_shape_id = p.createCollisionShape(
#         shapeType=p.GEOM_MESH,
#         fileName=obj_file,
#         collisionFramePosition=shift,
#         meshScale=scale,
#     )
#     obj_id = p.createMultiBody(
#         baseMass=0,
#         baseCollisionShapeIndex=collision_shape_id,
#         baseVisualShapeIndex=visual_shape_id,
#         basePosition=[0, 0, 0],
#         useMaximalCoordinates=True
#     )
#     obj_pos = pos
#     obj_orientation = p.getQuaternionFromEuler(rot)
#     p.resetBasePositionAndOrientation(obj_id, obj_pos, obj_orientation)

#     return obj_id


def load_obj_urdf(urdf_path, pos=[0, 0, 0], rot=[0, 0, 0]):

    tree = ET.parse(urdf_path)
    root = tree.getroot()
    center = np.array([float(i) for i in root[0].find(
        "inertial").find("origin").attrib['xyz'].split(' ')])
    obj_id = p.loadURDF(urdf_path, pos, p.getQuaternionFromEuler(rot))
    return obj_id, center

def render(pos, rot):
    rot_matrix = p.getMatrixFromQuaternion(rot)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)

    # camera params
    height = 640
    width = 480
    fx, fy = 596.6278076171875, 596.6278076171875
    cx, cy = 311.98663330078125, 236.76170349121094
    near, far = 0.1, 10

    camera_vector = rot_matrix.dot((0, 0, 1))
    up_vector = rot_matrix.dot((0, -1, 0))

    camera_eye_pos = np.array(pos)
    camera_target_position = camera_eye_pos + 0.2 * camera_vector

    view_matrix = p.computeViewMatrix(camera_eye_pos, camera_target_position, up_vector)

    proj_matrix = (2.0 * fx / width, 0.0, 0.0, 0.0,
                   0.0, 2.0 * fy / height, 0.0, 0.0,
                   1.0 - 2.0 * cx / width, 2.0 * cy / height - 1.0, (far + near) / (near - far), -1.0,
                   0.0, 0.0, 2.0 * far * near / (near - far), 0.0)

    return p.getCameraImage(width=width, height=height, viewMatrix=view_matrix, projectionMatrix=proj_matrix,
                     renderer=p.ER_BULLET_HARDWARE_OPENGL)  # renderer=self._p.ER_TINY_RENDERER)


def reset_pose(obj_id, x_offset=0.1, y_offset=0., z_offset=1.):
    x = (np.random.rand() - 0.5) * 0.2 + x_offset
    y = (np.random.rand() - 0.5) * 0.1 + y_offset
    z = (np.random.rand() - 0.5) * 0.2 + z_offset

    roll = np.random.rand() * np.pi * 2
    pitch = np.random.rand() * np.pi * 2
    yaw = np.random.rand() * np.pi * 2
    p.setGravity(0, 0, 0)
    p.resetBasePositionAndOrientation(
        obj_id,
        [x, y, z],
        p.getQuaternionFromEuler([roll, pitch, yaw]))

def update_debug_param(robot : pandaEnv):

    p.removeAllUserParameters()

    param_ids = []
    joint_ids = []
    num_joints = p.getNumJoints(robot.robot_id)

    joint_states = p.getJointStates(robot.robot_id, range(0, num_joints))
    joint_poses = [x[0] for x in joint_states]

    for i in range(num_joints):
        joint_info = p.getJointInfo(robot.robot_id, i)
        joint_name = joint_info[1]
        joint_type = joint_info[2]

        if joint_type is p.JOINT_REVOLUTE or joint_type is p.JOINT_PRISMATIC:
            joint_ids.append(i)
            param_ids.append(
                p.addUserDebugParameter(joint_name.decode("utf-8"), joint_info[8], joint_info[9], joint_poses[i]))
    
    return param_ids    

def extract_contact_point(candidate_pts : np.ndarray, eps=0.002, min_samples=2):

    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(candidate_pts)
    clustering_labels = clustering.labels_

    # compute cluster height
    cluster_means_z = []
    for i in range(np.max(clustering_labels) + 1):
        cond = np.where(clustering_labels == i)
        cluster_pts = candidate_pts[cond]
        
        cluster_mean_z = np.mean(cluster_pts, axis=0)[2]
        cluster_means_z.append(cluster_mean_z)
    
    # no clustering
    if len(cluster_means_z) == 0:
        return list(sorted(candidate_pts, key=lambda x: x[2])[0])

    cluster_means_z = np.asarray(cluster_means_z)
    
    # get the highest cluster
    highest_cluster_id = np.argsort(-cluster_means_z)[0]
    
    # the lowest point in the highest cluster
    cond = np.where(clustering_labels == highest_cluster_id)
    highest_pts = candidate_pts[cond]
    lphc_id = np.argsort(-highest_pts[:, 1])[0]
    lphc = highest_pts[lphc_id]

    return lphc

def main(args):

    # ----------------------- #
    # --- config io paths --- #
    # ----------------------- #

    # check output root exists
    assert os.path.exists(args.output_root), f'{args.output_root} not exists'

    hook_dir = os.path.join(args.hook_root, args.hook)
    if not os.path.exists(hook_dir):
        print(f'{hook_dir} not exists')
        return

    obj_dir = os.path.join(args.object_root, args.obj)
    if not os.path.exists(obj_dir):
        print(f'{obj_dir} not exists')
        return

    output_dir = os.path.join(args.output_root, args.output_dir)
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
    if not os.path.exists(os.path.join(output_dir, f'{args.hook}-{args.obj}')):
        os.mkdir(os.path.join(output_dir, f'{args.hook}-{args.obj}'))
    
    hook_path = os.path.join(hook_dir, 'base.urdf')
    obj_paths = glob.glob(f'{obj_dir}/*/base.urdf')

    # ------------------------ #
    # --- Setup simulation --- #
    # ------------------------ #

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

    # Load plane contained in pybullet_data
    planeId = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))

    # Set gravity for simulation
    p.setGravity(0, 0, gravity)
    p.stepSimulation()

    # -------------------------- #
    # --- Load other objects --- #
    # -------------------------- #

    p.loadURDF(os.path.join(pybullet_data.getDataPath(), "table/table.urdf"), [1, 0.0, 0.0])
    # obj_id = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "lego/lego.urdf"), [0.5, 0.0, 0.8])
    # obj_id = p.loadURDF(os.path.join(ycb_objects.getDataPath(), 'YcbBanana', "model.urdf"), [0.5, 0.0, 0.8])

    # wall
    # wall_pos = [0.8, -0.21, 0.8]
    # wall_orientation = p.getQuaternionFromEuler([0, 0, 0])
    # wall_id = p.loadURDF("models/wall/wall.urdf", wall_pos, wall_orientation)

    # stop critiria
    max_contact = 1
    max_iter = 1000

    # load hook
    # hook_pos=[0.8, -0.2, 1.0] for hook_90
    # hook_pos=[0.8, -0.2, 1.0]
    hook_pos=[0.5, -0.1, 1.3]
    hook_rot=[0, 0, -np.pi/2]
    if 'Hook12/' in hook_path or 'Hook12-' in hook_path:
        hook_rot=[np.pi/2, 0, -np.pi/2]
    elif 'Hook122/' in hook_path or 'Hook122-' in hook_path:
        hook_rot=[np.pi, 0, -np.pi/2]
    elif 'Hook186/' in hook_path or 'Hook186-' in hook_path:
        hook_rot=[-np.pi/2, np.pi/2, 0]
    elif 'Hook48/' in hook_path or 'Hook48-' in hook_path:
        hook_rot=[np.pi/2, -np.pi/2, 0]
    elif 'Hook_' in hook_path:
        hook_rot=[np.pi/2, 0, np.pi]
    hook_id = p.loadURDF(hook_path, hook_pos, p.getQuaternionFromEuler(hook_rot))
    
    # ignore_list = []
    ignore_list = [ 
        "daily_5/",
        "bag_5/",  
        "scissor_4/", "mug_59/", "wrench_1/", 
        "bag_6/", "bag_70/",
        "daily_11/", "daily_114/", "daily_115/", "daily_2/", "daily_23/",  
        "daily_42/", "daily_57/", "daily_63/", "daily_84/", "daily_7/", "daily_71/", "daily_72/",
        "daily_85/", "daily_97/", "daily_8/", "daily_106/", "daily_41/",
        "mug_118/",
        # "mug_19/", 
        "mug_100/", "mug_11/", "mug_112/", "mug_113/", "mug_115/",  "mug_123/", "mug_126/", "mug_128/", 
        "mug_132/", "mug_67/", "mug_70/", "mug_80/", "mug_82/", "mug_90/", "mug_135/", "mug_199/", "mug_73/", "mug_129/",
        "mug_142/", "mug_146/", "mug_147/", "mug_149/", "mug_150/", "mug_159/", "mug_166/", "mug_184/",
        "mug_173/", "mug_181/", "mug_193/", "mug_204/", "mug_205/", "mug_43/", "mug_145/", "mug_64/",
        "scissor_101/", "scissor_12/", "scissor_14/", "scissor_19/", "scissor_22/", "scissor_27/", "scissor_39/", 
        "scissor_48/", "scissor_58/", "scissor_62/", "scissor_74/", "scissor_79/", "scissor_8/", "scissor_92/", 
        "scissor_95/", "scissor_98/", "scissor_31/",
        "wrench_10/", "wrench_12/",  "wrench_17/", "wrench_35/", "wrench_25/", 
        "wrench_27/", "wrench_31/", "wrench_32/", "wrench_36/", "wrench_6/"
    ]   
    # focus_list = ['scissor_39/", 'wrench_27/", 'mug_67']

    mug_good_quat = [
                0.5704888371189183,
                -0.4522103472574786,
                0.5152682036576006,
                0.4522686887851561
            ]
    mug_good_rot = R.from_quat(mug_good_quat).as_rotvec()

    height_thresh = 0.8
    for obj_path in obj_paths:

        # ignore some files if needed
        cont = False
        for ignore_item in ignore_list:
            if ignore_item in obj_path:
                print(f'ignore : {ignore_item}')
                cont = True
        if cont:
            continue

        # config object hook info
        obj_id, center = load_obj_urdf(obj_path)
        obj_name = args.obj + '_' + obj_path.split('/')[-2]
        hook_name = args.hook

        # config output json dict and filename
        result_path = os.path.join(output_dir, f'{args.hook}-{args.obj}', f'{hook_name}-{obj_name}.json')
        if os.path.exists(result_path):
            f = open(result_path, 'r')
            result_json = json.load(f)
            result_json['contact_info'] = [] # clear
            f.close()
        else:
            result_json = {
                'hook_path': hook_path,
                'obj_path': obj_path,
                'hook_pose': hook_pos + list(p.getQuaternionFromEuler(hook_rot)),
                'contact_info': []
            }


        print(f'processing {result_path} -> {obj_path}...')
        contact_cnt = 0
        try_num = 0

        # forward simulation loop
        while contact_cnt < max_contact :# and try_num < max_iter:
            try_num += 1
            failed = False

            p.setGravity(0, 0, 0)

            # sample inital object pose
            if args.hook == 'Hook_bar':
                reset_pose(obj_id, x_offset=0.5, y_offset=0.0, z_offset=1.25) # for hook_bar
            elif args.hook == 'Hook_180':
                reset_pose(obj_id, x_offset=0.5, y_offset=0.0, z_offset=1.4) # for hook_bar
            elif args.hook == 'Hook_90':
                reset_pose(obj_id, x_offset=0.5, y_offset=0.0, z_offset=1.35) # for hook_90
            elif args.hook == 'Hook_60':
                reset_pose(obj_id, x_offset=0.5, y_offset=0.0, z_offset=1.35) # for hook_60
            elif args.hook == 'Hook_skew':
                reset_pose(obj_id, x_offset=0.5, y_offset=0.0, z_offset=1.3) # for hook_60
            else:
                reset_pose(obj_id, x_offset=0.5, y_offset=0.0, z_offset=1.35) # for hook_60

            p.stepSimulation()
            contact_points = p.getContactPoints(obj_id, hook_id)
            # collision on hooks
            if contact_points:
                continue
            
            # tossing force
            p.resetBaseVelocity(obj_id, [0.0, -0.2, -0.1])

            for _ in range(500):
                p.stepSimulation()
                pos, rot = p.getBasePositionAndOrientation(obj_id)
                if pos[2] < height_thresh:
                    failed = True
                    break
            if failed:
                continue
            
            # direct force
            p.resetBaseVelocity(obj_id, [0, 0, 0])
            p.setGravity(0, 0, gravity)
            for _ in range(1000):
                p.stepSimulation()
                pos, rot = p.getBasePositionAndOrientation(obj_id)
                if pos[2] < height_thresh:
                    failed = True
                    break

            if failed:
                continue

            contact_points = p.getContactPoints(obj_id, hook_id)
            if len(contact_points) == 0:
                continue
            
            # left force
            p.setGravity(2, 0, -5)
            for _ in range(1000):
                pos, rot = p.getBasePositionAndOrientation(obj_id)
                if pos[2] < height_thresh:
                    failed = True
                    break
                p.stepSimulation()
            if failed:
                continue

            # right force
            p.setGravity(-2, 0, -5)
            for _ in range(1000):
                pos, rot = p.getBasePositionAndOrientation(obj_id)
                if pos[2] < height_thresh:
                    failed = True
                p.stepSimulation()
            if failed:
                continue
            
            # make the object stable on the hook
            p.setGravity(0, 0, gravity)
            for _ in range(20000):
                pos, rot = p.getBasePositionAndOrientation(obj_id)
                if pos[2] < height_thresh:
                    failed = True
                    break

                p.stepSimulation()
            if failed:
                continue
            
            # check contact
            contact_points = p.getContactPoints(obj_id, hook_id)
            if len(contact_points) < 3:
                continue

            # special : check mug rotation
            if 'mug' in obj_path:
                mug_rot = R.from_quat(rot).as_rotvec()
                rot_diff = np.sum((np.asarray(mug_rot) - np.asarray(mug_good_rot))**2)
                print(rot_diff)
                if rot_diff > 1.0:
                    print('================ the rotation of mug is not good!!! ================')
                    continue
            
            # add candidate contact points
            p.removeAllUserDebugItems()
            candidate_pts = []
            for contact_point in contact_points:
                candidate_pts.append(contact_point[5])
            candidate_pts = np.asarray(candidate_pts)
            
            # relative homogeneous contact point
            contact_point = extract_contact_point(candidate_pts, eps=0.01, min_samples=3)
            contact_point_homo = np.concatenate((contact_point, [1]))

            # relative transform (hook, object)
            hook_transform = get_matrix_from_pos_rot(hook_pos, p.getQuaternionFromEuler(hook_rot))
            obj_transform = get_matrix_from_pos_rot(pos, rot)
            contact_point_hook = np.linalg.inv(hook_transform) @ contact_point_homo
            contact_point_obj = np.linalg.inv(obj_transform) @ contact_point_homo

            # draw contact point
            contact_point_homo_hook = hook_transform @ contact_point_hook
            contact_point_homo_obj = obj_transform @ contact_point_obj
            contact_point_pose_hook = list(contact_point_homo_hook[:3]) + [0, 0, 0, 1]
            contact_point_pose_obj = list(contact_point_homo_obj[:3]) + [0, 0, 0, 1]
            draw_coordinate(contact_point_pose_hook)
            draw_coordinate(contact_point_pose_obj)

            # check result by human
            ok = True
            while True:
                p.stepSimulation()
                keys = p.getKeyboardEvents()
                if ord('y') in keys and keys[ord('y')] == 3 and p.KEY_IS_DOWN: 
                    print('===================== save =====================')
                    break
                if ord('n') in keys and keys[ord('n')] == 3 and p.KEY_IS_DOWN: 
                    print('===================== continue =====================')
                    ok = False
                    break
            if ok == False:
                continue
            
            # write to json dict
            contact_info = {
                'contact_point_hook': contact_point_hook.tolist(),
                'obj_pose': list(pos + rot),
                'contact_point_obj': contact_point_obj.tolist(),
            }
            contact_cnt+=1
            result_json['contact_info'].append(contact_info)

        p.removeBody(obj_id)
        if len(result_json['contact_info']) > 0:
            with open(result_path, 'w') as f:
                json.dump(result_json, f, indent=4)
                print(f'{result_path} saved')
                f.close()
        else :
            print(f'no pose : {obj_path}')
    
# start message
start_msg = \
'''
======================================================================================
this script will collect contact points of a object and a hook by forward simulation
dependency :
- object folder that contains /[object_name]/base.urdf
- hook folder that contains /[hook_name]/base.urdf
- root directory of the contact points output
note :
- you can run it using ./run.sh hangsim (with predefined hook_id)
======================================================================================
'''

print(start_msg)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root', '-or', type=str, default='data')
    parser.add_argument('--output-dir', '-od', type=str, default='data_1205')
    parser.add_argument('--object-root', '-ir', type=str, default='models/geo_data')
    parser.add_argument('--hook-root', '-hr', type=str, default='models/hook')
    parser.add_argument('--obj', '-o', type=str, default='hanging_exp')
    parser.add_argument('--hook', '-ho', type=str, default='Hook_180')
    args = parser.parse_args()
    main(args)