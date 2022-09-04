import numpy as np
import open3d as o3d

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

def HCE(obj_pcd : o3d.geometry.PointCloud, obstacle_pcd : o3d.geometry.PointCloud):
    obj_points = np.asarray(obj_pcd.points)
    obstacle_points = np.asarray(obstacle_pcd.points)
    obstacle_normals = np.asarray(obstacle_pcd.normals)

    neigh = NearestNeighbors(n_neighbors=1, algorithm='kd_tree')
    neigh.fit(obstacle_points)
    distances, indices = neigh.kneighbors(obj_points)
    distances = distances.squeeze()
    indices = indices.squeeze()

    obstacle_to_obj = obj_points - obstacle_points[indices]
    obstacle_to_obj = (obstacle_to_obj.T / np.linalg.norm(obstacle_to_obj, ord=2, axis=1)).T 
    print(np.mean(obstacle_to_obj * obstacle_normals[indices], axis=0))
    ret = np.sum(obstacle_to_obj * obstacle_normals[indices])
    print(f'{ret}/{obj_points.shape[0]}')


    return True

deg2euler = 1 / 180 * np.pi

obj_pcd = o3d.io.read_point_cloud('models/geo_data/hanging_exp/bag_5/base.ply')
obstacle_pcd = o3d.io.read_point_cloud('models/hook/Hook_180/base.ply')

pos = [0, 0, 0]
rot = R.from_rotvec([0, 0, 0]).as_quat()
obj_pcd.transform(get_matrix_from_pos_rot(pos, rot))

extrinsic_rot = R.from_rotvec([0, -90 * deg2euler, 0]).as_matrix()
extrinsic = np.identity(4)
extrinsic[:3, :3] = extrinsic_rot
extrinsic[:3, 3] = [0, 0, 0.1]
obj_pcd.transform(extrinsic)

origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
ret = HCE(obj_pcd=obj_pcd, obstacle_pcd=obstacle_pcd)

o3d.visualization.draw_geometries([origin, obj_pcd, obstacle_pcd], point_show_normal=True)