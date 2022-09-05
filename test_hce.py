import os, glob
import numpy as np
import argparse
import open3d as o3d
import pybullet as p
import pybullet_data

from scipy.spatial.transform import Rotation as R
from sklearn.neighbors import NearestNeighbors


def get_matrix_from_pos_rot(pos : list or tuple, rot : list or tuple):
    assert (len(pos) == 3 and len(rot) == 4) or (len(pos) == 3 and len(rot) == 3)
    pos_m = np.asarray(pos)
    if len(rot) == 3:
        rot_m = R.from_rotvec(rot).as_matrix()
    elif len(rot) == 4:
        rot_m = R.from_quat(rot).as_matrix()
    ret_m = np.identity(4)
    ret_m[:3, :3] = rot_m
    ret_m[:3, 3] = pos_m
    return ret_m

def HCE(obj_pcd : o3d.geometry.PointCloud, obstacle_pcd : o3d.geometry.PointCloud, thresh : float = 0.0):

    obj_pcd_down = obj_pcd.voxel_down_sample(voxel_size=0.002)
    obstacle_pcd_down = obstacle_pcd.voxel_down_sample(voxel_size=0.002)
    obj_points = np.asarray(obj_pcd_down.points)
    obstacle_points = np.asarray(obstacle_pcd_down.points)
    obstacle_normals = np.asarray(obstacle_pcd_down.normals)

    # obstacle_to_obj
    neigh = NearestNeighbors(n_neighbors=1, algorithm='kd_tree')
    neigh.fit(obstacle_points)
    distances, indices = neigh.kneighbors(obj_points)
    distances = distances.squeeze()
    indices = indices.squeeze()

    obstacle_to_obj = obj_points - obstacle_points[indices]
    obstacle_to_obj = (obstacle_to_obj.T / np.linalg.norm(obstacle_to_obj, ord=2, axis=1)).T 
    ret_obstacle_to_obj = np.sum(obstacle_to_obj * obstacle_normals[indices])

    ratio = ret_obstacle_to_obj / obj_points.shape[0]

    return ratio < thresh

def random_pose(low_limits=[-0.2, -0.2, -0.2, -np.pi, -np.pi, -np.pi], 
                high_limits=[0.2,  0.2,  0.2, np.pi, np.pi, np.pi]):
    pos = np.random.uniform(low_limits[:3], high_limits[:3], size=3)
    rot_vec = np.random.uniform(low_limits[:3], high_limits[:3], size=3)
    rot = R.from_rotvec(rot_vec).as_quat()

    return list(pos), list(rot)

def main(args):

    obj_dir = args.obj
    hook_dir = args.hook
    assert os.path.exists(obj_dir), f'{obj_dir} not exists'
    assert os.path.exists(hook_dir), f'{hook_dir} not exists'

    p.connect(p.DIRECT)
    # p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.resetDebugVisualizerCamera(
        cameraDistance=0.3,
        cameraYaw=0,
        cameraPitch=0,
        cameraTargetPosition=[0.0, 0.0, 0.0]
    )

    deg2euler = 1 / 180 * np.pi
    collision_max = 1000
    collision_cnt = 0
    uncollision_max = 1000
    uncollision_cnt = 0
    low_limits = [ -0.1, -0.2,    0, -np.pi, -np.pi, -np.pi]
    high_limits = [ 0.1,  0.2,  0.2,  np.pi,  np.pi,  np.pi]

    obj_pcd_paths = glob.glob(f'{obj_dir}/*/base.ply')
    obj_urdf_paths = glob.glob(f'{obj_dir}/*/base.urdf')
    hook_pcd_paths = glob.glob(f'{hook_dir}/*/base.ply')
    hook_urdf_paths = glob.glob(f'{hook_dir}/*/base.urdf')

    assert len(obj_pcd_paths) == len(obj_urdf_paths)
    assert len(hook_pcd_paths) == len(hook_urdf_paths)

    gt = []
    pred = []
    for obj_i in range(len(obj_pcd_paths)):
        for hook_i in range(len(hook_pcd_paths)):
            obj_id = p.loadURDF(obj_urdf_paths[obj_i], [0, 0, 0])
            hook_id = p.loadURDF(hook_urdf_paths[hook_i], [0, 0, 0])
            obj_pcd = o3d.io.read_point_cloud(obj_pcd_paths[obj_i])
            obstacle_pcd = o3d.io.read_point_cloud(hook_pcd_paths[hook_i])
            collision_cnt = 0
            uncollision_cnt = 0
            while collision_cnt < collision_max or uncollision_cnt < uncollision_max:

                pos, rot = random_pose(low_limits=low_limits, high_limits=high_limits)

                p.resetBasePositionAndOrientation(obj_id, pos, rot)
                p.performCollisionDetection()
                contact = p.getContactPoints(obj_id, hook_id)

                if contact != ():
                    if collision_cnt < collision_max:
                        gt.append(1)
                        collision_cnt += 1
                        print(f'collision : {collision_cnt}')
                    else :
                        continue
                else :
                    if uncollision_cnt < uncollision_max:
                        gt.append(0)
                        uncollision_cnt += 1
                        print(f'no collision : {uncollision_cnt}')
                    else :
                        continue

                extrinsic = get_matrix_from_pos_rot(pos, rot)
                obj_pcd.transform(extrinsic)

                # origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
                ret = HCE(obj_pcd=obj_pcd, obstacle_pcd=obstacle_pcd, thresh=args.thresh)
                if ret :
                    pred.append(1)
                else :
                    pred.append(0)

                # o3d.visualization.draw_geometries([origin, obj_pcd, obstacle_pcd], point_show_normal=True)
                obj_pcd.transform(np.linalg.inv(extrinsic))
            p.removeBody(obj_id)
            p.removeBody(hook_id)
    
    gt = np.asarray(gt)
    pred = np.asarray(pred)
    diff = gt - pred

    pred_positive_cond = np.where(pred == 1)[0]
    gt_positive_cond = np.where(gt == 1)[0]

    correct_cond = np.where(diff == 0)[0]
    accuracy = len(correct_cond) / len(diff)
    precision = np.sum(gt[pred_positive_cond]) / len(pred_positive_cond)
    recall = np.sum(pred[gt_positive_cond]) / len(gt_positive_cond)

    path = f'hce_test/hce_{args.thresh}.txt'
    with open(path, 'w') as f:
        f.write('-------------------------\n')
        f.write(f'| accuracy : {accuracy}\n')
        f.write(f'| precision : {precision}\n')
        f.write(f'| recall : {recall}\n')
        f.write('-------------------------\n')
        f.close()


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--obj', type=str, default='')
    parser.add_argument('--hook', type=str, default='')
    parser.add_argument('--thresh', type=float, default=-0.15)
    args = parser.parse_args()
    main(args)