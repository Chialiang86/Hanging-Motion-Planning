import glob
import os
import time
import argparse
import pybullet as p
import math
import numpy as np
import pybullet_data
import scipy.io as sio
import open3d as o3d
from scipy.spatial.transform import Rotation as R


def get_matrix_from_pos_rot(pos : list or tuple, rot : list or tuple):
    assert (len(pos) == 3 and len(rot) == 4) or (len(pos) == 3 and len(rot) == 3)
    pos_m = np.asarray(pos)
    if len(rot) == 3:
        rot_m = np.asarray(p.getMatrixFromQuaternion(p.getQuaternionFromEuler(rot))).reshape((3, 3))
    elif len(rot) == 4:
        rot_m = np.asarray(p.getMatrixFromQuaternion(rot)).reshape((3, 3))
    ret_m = np.identity(4)
    ret_m[:3, :3] = rot_m
    ret_m[:3, 3] = pos_m
    return ret_m

def create_rgbd(rgb, depth, intr, extr, dscale, depth_threshold=2.0):
    assert rgb.shape[:2] == depth.shape
    (h, w) = depth.shape
    fx, fy, cx, cy = intr[0, 0], intr[1, 1], intr[0, 2], intr[1, 2]

    ix, iy =  np.meshgrid(range(w), range(h))

    x_ratio = (ix.ravel() - cx) / fx
    y_ratio = (iy.ravel() - cy) / fy

    z = depth.ravel() / dscale
    x = z * x_ratio
    y = z * y_ratio

    cond = np.where(z < depth_threshold)
    x = x[cond]
    y = y[cond]
    z = z[cond]

    points = np.vstack((x, y, z)).T
    center = np.mean(points, axis=0)
    colors = np.reshape(rgb,(depth.shape[0] * depth.shape[1], 3))
    colors = np.array([colors[:,2], colors[:,1], colors[:,0]]).T / 255.

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
    pcd.transform(extr)

    return pcd


def pcd_adjustment(pcd : o3d.geometry.PointCloud, pitch : float, yaw : float):
  deg2euler = 1 / 180 * np.pi

  if np.abs(pitch) >= 90:
    extrinsic_rot = R.from_rotvec([-(pitch + 90) * deg2euler, 0, 0]).as_matrix()
    extrinsic = np.identity(4)
    extrinsic[:3, :3] = extrinsic_rot
    pcd.transform(extrinsic)
    extrinsic_rot = R.from_rotvec([0, np.pi, 0]).as_matrix()
    extrinsic = np.identity(4)
    extrinsic[:3, :3] = extrinsic_rot
    pcd.transform(extrinsic)
  else :
    extrinsic_rot = R.from_rotvec([(pitch - 90) * deg2euler, 0, 0]).as_matrix()
    extrinsic = np.identity(4)
    extrinsic[:3, :3] = extrinsic_rot
    pcd.transform(extrinsic)

  extrinsic_rot = R.from_rotvec([0, 0, yaw * deg2euler]).as_matrix()
  extrinsic = np.identity(4)
  extrinsic[:3, :3] = extrinsic_rot
  pcd.transform(extrinsic)

  return pcd

def reset_camera(yaw : float, pitch : float, cameraDistance : float):
  p.resetDebugVisualizerCamera(
      cameraDistance=cameraDistance,
      cameraYaw=yaw,
      cameraPitch=pitch,
      cameraTargetPosition=[0.0, 0.0, 0.0]
  )

def adjust_normals(pcd : o3d.geometry.PointCloud()):
  normals = np.asarray(pcd.normals)
  points = np.asarray(pcd.points)
  mass_center = np.mean(points, axis=0)

  pos_relative_to_center = points - mass_center
  inner_product_with_normals = np.sum(pos_relative_to_center * normals, axis=1)
  print(inner_product_with_normals.shape)

  cond = np.where(inner_product_with_normals < 0)
  normals[cond] *= -1

def main(args):

  input_dir = args.input_dir
  if not os.path.exists(input_dir):
    print(f'{input_dir} not exists')
    return 


  p.connect(p.GUI)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())

  debug_cam_param = [
    [45, -120, 0.3],
    [225, -120, 0.3],
  ]

  width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera()
  # fov = 2 * math.atan(1 / projMat[0])
  fx = fy = (0.5 * height) * (projMat[0]) # (height / 2) * (1 / tan(FOV/2))
  cx = 0.5 * width
  cy = 0.5 * height
  intrinsic = np.identity(3)
  intrinsic[0, 0] = fx
  intrinsic[1, 1] = fy 
  intrinsic[0, 2] = cx 
  intrinsic[1, 2] = cy

  far = 1000.
  near = 0.01
  depth_threshold = 2.0
  deg2euler = 1 / 180 * np.pi
  
  origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
  urdl_files = glob.glob(f'{input_dir}/*/base.urdf')
  for urdl_file in urdl_files:
    print(f'processing {urdl_file}')

    # if 'mug' not in urdl_file:
    #   continue

    pcds = []
    obj_id = p.loadURDF(urdl_file, [0, 0, 0])

    for (yaw, pitch, cameraDistance) in debug_cam_param:

      # reset_camera(yaw, pitch, cameraDistance)
      print((yaw, pitch, cameraDistance))
      p.resetDebugVisualizerCamera(
          cameraDistance=cameraDistance,
          cameraYaw=yaw,
          cameraPitch=pitch,
          cameraTargetPosition=[0.0, 0.0, 0.0]
      )

      time.sleep(1)

      img = p.getCameraImage(width, height, renderer=p.ER_BULLET_HARDWARE_OPENGL)
      rgbBuffer = np.reshape(img[2], (height, width, 4))[:,:,:3]
      depthBuffer = np.reshape(img[3], [height, width])

      # get real depth
      depthBuffer = far * near / (far - (far - near) * depthBuffer)

      # adjustment
      extrinsic_trans = np.identity(4)
      extrinsic_trans[:3, 3] = [0, 0, -cameraDistance]
      pcd = create_rgbd(rgbBuffer, depthBuffer, intrinsic, extrinsic_trans, dscale=1, depth_threshold=depth_threshold)
      pcd = pcd_adjustment(pcd, pitch, yaw)
      pcds.append(pcd)
      # o3d.visualization.draw_geometries([pcd, origin], point_show_normal=False)

    # merge N point clouds
    pcd_merged = o3d.geometry.PointCloud()
    assert len(pcds) > 0
    points = np.asarray(pcds[0].points)
    for i in range(1, len(pcds)):
      points = np.vstack((points, np.asarray(pcds[i].points)))
    pcd_merged.points = o3d.utility.Vector3dVector(points)

    pcd_merged.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.03,
                                                          max_nn=30))
    # adjust_normals(pcd_merged)

    # pcd.transform(pose)
    mesh_file = os.path.splitext(urdl_file)[0] + '.obj'
    pcd_ori = o3d.io.read_triangle_mesh(mesh_file)
    scale = 1.0
    if 'mug' in urdl_file:
      scale = 1.2
    if 'scissor' in urdl_file:
      scale = 1.5
    if 'wrench' in urdl_file:
      scale = 3.0
    pcd_ori.scale(scale, [0., 0., 0.,])
    
    o3d.visualization.draw_geometries([origin, pcd_merged], point_show_normal=True)

    output_path = os.path.splitext(urdl_file)[0] + '_merged.ply'
    o3d.io.write_point_cloud(output_path, pcd_merged)
    print(f'{output_path} saved')
    p.removeBody(obj_id)


if __name__=="__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--input-dir', '-id', type=str, default='models/geo_data/hanging_exp')
  args = parser.parse_args()

  main(args)



