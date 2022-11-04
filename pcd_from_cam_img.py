import glob
import os
import time
import argparse
import pybullet as p
import numpy as np
import pybullet_data
import scipy.io as sio
import open3d as o3d
import xml.etree.ElementTree as ET

from PIL import Image
from scipy.spatial.transform import Rotation as R

def load_obj_urdf(urdf_path, pos=[0, 0, 0], rot=[0, 0, 0]):

    tree = ET.parse(urdf_path)
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
    scale = np.array(
      [
        float(i) for i in root[0].find(
          "visual"
        ).find(
          "geometry"
        ).find(
          "mesh"
        ).attrib["scale"].split(" ")
      ]
    )
    assert scale[0] == scale[1] and scale[0] == scale[2] and scale[1] == scale[2], f"scale is not uniformed : {scale}"
    obj_id = p.loadURDF(urdf_path, pos, p.getQuaternionFromEuler(rot))
    return obj_id, center, scale[0]

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
  file_token = args.file_token
  if not os.path.exists(input_dir):
    print(f'{input_dir} not exists')
    return 

  p.connect(p.GUI)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())

  p.resetDebugVisualizerCamera(
      cameraDistance=0.3,
      cameraYaw=120,
      cameraPitch=-30,
      cameraTargetPosition=[0.0, 0.0, 0.0]
  )
  p.resetSimulation()
  p.setPhysicsEngineParameter(numSolverIterations=150)
  sim_timestep = 1.0 / 240
  p.setTimeStep(sim_timestep)
  p.setGravity(0, 0, -9.8)

  pcd_cam_param = {
    'cameraEyePosition': [-0.24, 0.16, 0.0],
    'cameraTargetPosition': [0.0, 0.0, 0.0],
    'cameraUpVector': [0.0, 0.0, 1.0],
  }

  rgb_cam_param = {
    'cameraEyePosition': [-0.12, 0.08, 0.0],
    'cameraTargetPosition': [0.0, 0.0, 0.0],
    'cameraUpVector': [0.0, 0.0, 1.0],
  }

  # intrinsic matrix
  width, height = 640, 480
  fx = fy = 605
  cx = 0.5 * width
  cy = 0.5 * height
  intrinsic = np.array([
    [fx, 0., cx],
    [0., fy, cy],
    [0., 0., 1.]
  ])

  # config camera params : https://www.intelrealsense.com/depth-camera-d435/
  # D435 serial_num=141722071222, fx=605, fy=605, cx=323, cy=240
  # fov = 2 * atan(480/605) ~= 1.34139295 rad ~= 76.8561560146
  far = 10.
  near = .11
  fov = 2 * np.arctan(height / fy) * 180.0 / np.pi
  depth_threshold = 2.0

  pcd_view_matrix = p.computeViewMatrix(
                  cameraEyePosition=pcd_cam_param['cameraEyePosition'],
                  cameraTargetPosition=pcd_cam_param['cameraTargetPosition'],
                  cameraUpVector=pcd_cam_param['cameraUpVector']
                )

  rgb_view_matrix = p.computeViewMatrix(
                  cameraEyePosition=rgb_cam_param['cameraEyePosition'],
                  cameraTargetPosition=rgb_cam_param['cameraTargetPosition'],
                  cameraUpVector=rgb_cam_param['cameraUpVector']
                )
  
  projection_matrix = p.computeProjectionMatrixFOV(
                        fov=fov,
                        aspect=1.0,
                        nearVal=near,
                        farVal=far
                      )
  
  origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
  urdl_files = glob.glob(f'{input_dir}/*/{file_token}.urdf')
  urdl_files.sort()
  for urdl_file in urdl_files:

    if len(urdl_file.split('#')) > 1:
      serial_num = int(urdl_file.split('/')[-2].split('#')[1])
      if serial_num < 0:
        continue
    print(f'processing {urdl_file}')
    obj_id, center, scale = load_obj_urdf(urdl_file, [0, 0, 0])


    # reset_camera(yaw, pitch, cameraDistance)

    time.sleep(0.05)
    img = p.getCameraImage(width, height, viewMatrix=pcd_view_matrix, projectionMatrix=projection_matrix)
    rgb_buffer = np.reshape(img[2], (height, width, 4))[:,:,:3]
    depth_buffer = np.reshape(img[3], [height, width])

    # get real depth
    depth_buffer = far * near / (far - (far - near) * depth_buffer)

    # adjustment
    extrinsic_trans = np.identity(4)
    extrinsic_trans[:3, 3] = np.asarray(pcd_cam_param['cameraTargetPosition']) - \
                             np.asarray(pcd_cam_param['cameraEyePosition'])
    pcd = create_rgbd(rgb_buffer, depth_buffer, intrinsic, extrinsic_trans, dscale=1, depth_threshold=depth_threshold)

    # mesh_file = os.path.splitext(urdl_file)[0] + '.obj'
    # pcd_ori = o3d.io.read_triangle_mesh(mesh_file)
    # pcd_ori.scale(scale, [0., 0., 0.,])
    
    # save ply
    # o3d.visualization.draw_geometries([origin, pcd_merged], point_show_normal=False)
    output_ply_path = os.path.splitext(urdl_file)[0] + '.ply'
    o3d.io.write_point_cloud(output_ply_path, pcd)

    img = p.getCameraImage(height, height, viewMatrix=rgb_view_matrix, projectionMatrix=projection_matrix)
    rgb_buffer = np.reshape(img[2], (height, height, 4))[:,:,:3]

    # save image
    # output_jpg_path = os.path.splitext(urdl_file)[0] + '.jpg'
    sub_dir = urdl_file.split('/')[-2]
    output_jpg_path = f'models/hook/All_img/{sub_dir}.jpg'
    pil_img = Image.fromarray(rgb_buffer)
    pil_img.save(output_jpg_path)

    print(f'{output_ply_path} and {output_jpg_path} saved')
    p.removeBody(obj_id)

if __name__=="__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--input-dir', '-id', type=str, default='models/geo_data/hanging_exp')
  parser.add_argument('--file-token', '-ft', type=str, default='base')
  args = parser.parse_args()

  main(args)