import glob, os, time, argparse, cv2
from  tqdm import tqdm
import pybullet as p
import numpy as np
import pybullet_data
import scipy.io as sio
import open3d as o3d
import xml.etree.ElementTree as ET

from PIL import Image
from scipy.spatial.transform import Rotation as R

def load_obj_urdf(urdf_path, pos=[0, 0, 0], rot=[0, 0, 0, 1]):

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
    obj_id = p.loadURDF(urdf_path, pos, rot)
    return obj_id, center, scale[0]

def cross(a:np.ndarray,b:np.ndarray)->np.ndarray:
    return np.cross(a,b)

def get_projmat_and_intrinsic(width, height, fx, fy, far, near):

  cx = width / 2
  cy = height / 2
  fov = 2 * np.arctan(height / (2 * fy)) * 180.0 / np.pi

  project_matrix = p.computeProjectionMatrixFOV(
                      fov=fov,
                      aspect=width/height,
                      nearVal=near,
                      farVal=far
                    )
  
  intrinsic = np.array([
                [ fx, 0.0,  cx],
                [0.0,  fy,  cy],
                [0.0, 0.0, 1.0],
              ])
  
  return project_matrix, intrinsic

def get_viewmat_and_extrinsic(cameraEyePosition, cameraTargetPosition, cameraUpVector):

    view_matrix = p.computeViewMatrix(
                    cameraEyePosition=cameraEyePosition,
                    cameraTargetPosition=cameraTargetPosition,
                    cameraUpVector=cameraUpVector
                  )

    # rotation vector extrinsic
    z = np.asarray(cameraTargetPosition) - np.asarray(cameraEyePosition)
    norm = np.linalg.norm(z, ord=2)
    assert norm > 0, f'cameraTargetPosition and cameraEyePosition is at same location'
    z /= norm
   
    y = -np.asarray(cameraUpVector)
    y -= (np.dot(z, y)) * z
    norm = np.linalg.norm(y, ord=2)
    assert norm > 0, f'cameraUpVector is parallel to z axis'
    y /= norm
    
    x = cross(y, z)

    # extrinsic
    extrinsic = np.identity(4)
    extrinsic[:3, 0] = x
    extrinsic[:3, 1] = y
    extrinsic[:3, 2] = z
    extrinsic[:3, 3] = np.asarray(cameraEyePosition)

    return view_matrix, extrinsic

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
    # print(f'There are {len(cond[0])} points in point clouds')
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

def render_affordance_map(pcd : o3d.geometry.PointCloud, center : np.ndarray, std : float=0.01):
    points = np.asarray(pcd.points)
    print(f'there are {points.shape[0]} in point cloud')
    points_diff = np.linalg.norm(points - center, axis=1, ord=2)
    points_gaussian = np.exp(-0.5 * (points_diff / std) ** 2)
    points_gaussian = (points_gaussian - np.min(points_gaussian)) / (np.max(points_gaussian) - np.min(points_gaussian))
    # points_gaussian = 0 * (points_gaussian - np.min(points_gaussian)) / (np.max(points_gaussian) - np.min(points_gaussian))
    colors = cv2.applyColorMap((255 * points_gaussian).astype(np.uint8), colormap=cv2.COLORMAP_JET).squeeze()
    colors = colors[:,::-1]
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # check the point cloud contain the contact point
    assert 1.0 - np.max(points_gaussian) < 1e-10, f'the point cloud doesn\'t contain contact point'
    cond = np.where(points_gaussian == np.max(points_gaussian))

    # move contact point to the first point
    val = points_gaussian[cond[0]]
    points_gaussian = np.delete(points_gaussian, cond[0], axis=0)
    points_gaussian = np.insert(points_gaussian, 0, val, axis=0)
    val = points[cond[0]]
    points = np.delete(points, cond[0], axis=0)
    points = np.insert(points, 0, val, axis=0)

    affordance_map = np.hstack((points, points_gaussian.reshape(-1, 1)))

    return affordance_map

def main(args):

  object_dir = args.object_dir
  if not os.path.exists(object_dir):
    print(f'{object_dir} not exists')
    return 
  if not os.path.exists(f'{object_dir}/report'):
    os.mkdir(f'{object_dir}/report')

  p.connect(p.GUI)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())

  p.resetDebugVisualizerCamera(
      cameraDistance=0.3,
      cameraYaw=60,
      cameraPitch=-30,
      cameraTargetPosition=[0.0, 0.0, 0.0]
  )
  p.resetSimulation()
  p.setPhysicsEngineParameter(numSolverIterations=150)
  sim_timestep = 1.0 / 240
  p.setTimeStep(sim_timestep)
  p.setGravity(0, 0, -9.8)

  # # config camera params : https://www.intelrealsense.com/depth-camera-d435/
  # # D435 serial_num=141722071222, fx=605, fy=605, cx=323, cy=240
  # intrinsic matrix
  depth_threshold = 2.0
  # width, height = 512, 512
  width, height = 640, 480
  fx = fy = 605
  fov = 2 * np.arctan(height / fy) * 180.0 / np.pi
  far = 1000.
  near = 0.01
  cx = 0.5 * width
  cy = 0.5 * height
  intrinsic = np.array([
    [fx, 0., cx],
    [0., fy, cy],
    [0., 0., 1.]
  ])
  projection_matrix, intrinsic = get_projmat_and_intrinsic(width, height, fx, fy, far, near)

  # extrinsic matrix for hanging pose
  cam_dist = 0.25
  cameraEyePositions = []
  # cam_angles = [
  #   30 * np.pi / 180,  35 * np.pi / 180, 40 * np.pi / 180, 45 * np.pi / 180, 50 * np.pi / 180, 55 * np.pi / 180,  60 * np.pi / 180, 
  #   120 * np.pi / 180, 125 * np.pi / 180, 130 * np.pi / 180, 135 * np.pi / 180, 140 * np.pi / 180, 145 * np.pi / 180, 150 * np.pi / 180
  # ]
  cam_angles = [
    0 * np.pi / 180,  90 * np.pi / 180, 180 * np.pi / 180
  ]

  # up =  [[-cam_dist * np.cos(i), 0.015, cam_dist * np.sin(i)] for i in cam_angles]
  # mid =  [[-cam_dist * np.cos(i), 0.01, cam_dist * np.sin(i)] for i in cam_angles]
  # down = [[-cam_dist * np.cos(i),  0.05, cam_dist * np.sin(i)] for i in cam_angles]
  up =  [[-cam_dist * np.cos(i), 0.1, cam_dist * np.sin(i)] for i in cam_angles]
  mid =  [[-cam_dist * np.cos(i), 0.05, cam_dist * np.sin(i)] for i in cam_angles]
  down = [[-cam_dist * np.cos(i),  -0.1, cam_dist * np.sin(i)] for i in cam_angles]
  cameraEyePositions.extend(up)
  # cameraEyePositions.extend(mid)
  cameraEyePositions.extend(down)

  cameraTargetPositions = [[0.0, 0.0, 0.0] for _ in range(len(cameraEyePositions))]
  cameraUpVectors = [[0.0, 1.0, 0.0] for _ in range(len(cameraEyePositions))]
  cam_extr_num = len(cameraEyePositions)

  # cameraEyePosition = [-0.12, 0.08, 0.0]
  # cameraTargetPosition = [0.0, 0.0, 0.0]
  # cameraUpVector =  [0.0, 0.0, 1.0]
  # cameraEyePosition = [0.0, 0.36, -0.24]
  # cameraTargetPosition = [0.0, 0.0, 0.0]
  # cameraUpVector =  [1.0, 0.0, 0.0]
  # rgb_view_matrix, rgb_extrinsic = get_viewmat_and_extrinsic(cameraEyePosition, cameraTargetPosition, cameraUpVector)

  origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
  urdf_files = glob.glob(f'{object_dir}/*/base.urdf')
  urdf_files.sort()
  for urdf_file in tqdm(urdf_files):
    print(urdf_file)
    
    # TODO:
    # in the future, add an option [hook/object]
    # hook: set the pose by using p.loadURDF
    # object: set the pose by using p.resetBasePositionAndOrientation
    print(f'processing {urdf_file}')
    pos = [0, 0, 0]
    rot = [0, 0, 0, 1]
    obj_id, center, scale = load_obj_urdf(urdf_file, pos, rot)

    pcd_extrinsics = []
    
    pcd_points = []
    pcd_colors = []
    for cam_id in range(cam_extr_num):

      output_ply_path = os.path.splitext(urdf_file)[0] + f'-{cam_id}.ply'
      # if os.path.exists(output_ply_path):
      #   print(f'ignore {output_ply_path}')
      #   continue

      pcd_view_matrix, pcd_extrinsic = get_viewmat_and_extrinsic(cameraEyePositions[cam_id], cameraTargetPositions[cam_id], cameraUpVectors[cam_id])

      pcd_extrinsics.append(pcd_extrinsic)

      time.sleep(0.01)
      img = p.getCameraImage(width, height, viewMatrix=pcd_view_matrix, projectionMatrix=projection_matrix)
      rgb_buffer = np.reshape(img[2], (height, width, 4))[:,:,:3]
      depth_buffer = np.reshape(img[3], [height, width])

      # get real depth
      depth_buffer = far * near / (far - (far - near) * depth_buffer)

      # adjustment
      pcd = create_rgbd(rgb_buffer, depth_buffer, intrinsic, pcd_extrinsic, dscale=1, depth_threshold=depth_threshold)
      pcd_point = np.asarray(pcd.points)
      pcd_color = np.asarray(pcd.colors)
      pcd_points.append(pcd_point)
      pcd_colors.append(pcd_color)

      # for visualization
      # mesh_file = os.path.splitext(urdf_file)[0] + '.obj'
      # pcd_ori = o3d.io.read_triangle_mesh(mesh_file)
      # pcd_ori.scale(scale, [0., 0., 0.,])
      # o3d.visualization.draw_geometries([origin, pcd_ori, pcd], point_show_normal=False)
      
      # save ply
      # o3d.io.write_point_cloud(output_ply_path, pcd)

      # img = p.getCameraImage(height, height, viewMatrix=pcd_view_matrix, projectionMatrix=projection_matrix)
      # rgb_buffer = np.reshape(img[2], (height, height, 4))[:,:,:3]

      # img = p.getCameraImage(height, height, viewMatrix=rgb_view_matrix, projectionMatrix=projection_matrix)
      # rgb_buffer = np.reshape(img[2], (height, height, 4))[:,:,:3]

      # # save image
      # # output_jpg_path = os.path.splitext(urdf_file)[0] + '.jpg'
      # sub_dir = urdf_file.split('/')[-2]
      # # output_jpg_path = f'{object_dir}/All_img/{sub_dir}.jpg'
      # output_jpg_path = f'{object_dir}/report/{sub_dir}.jpg'
      # pil_img = Image.fromarray(rgb_buffer)
      # pil_img.save(output_jpg_path)

      # print(f'{output_ply_path} and {output_jpg_path} saved')
    
    # mesh_file = os.path.splitext(urdf_file)[0] + '.obj'
    # pcd_ori = o3d.io.read_triangle_mesh(mesh_file)
    # pcd_ori.scale(scale, [0., 0., 0.,])
    # geometries = [pcd_ori]
    # for extr in pcd_extrinsics:
    #   origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
    #   origin.transform(extr)
    #   geometries.append(origin)
    # o3d.visualization.draw_geometries(geometries, point_show_normal=False)

    pcd_fullview = o3d.geometry.PointCloud()
    pcd_fullview_points = np.vstack(pcd_points)
    pcd_fullview_colors = np.vstack(pcd_colors)
    pcd_fullview.points = o3d.utility.Vector3dVector(pcd_fullview_points)
    pcd_fullview.colors = o3d.utility.Vector3dVector(pcd_fullview_colors)
    # o3d.visualization.draw_geometries([pcd_fullview], point_show_normal=False)

    output_ply_path = os.path.splitext(urdf_file)[0] + f'-fullview.ply'
    o3d.io.write_point_cloud(output_ply_path, pcd_fullview)

    p.removeBody(obj_id)

start_msg = \
'''
======================================================================================
this script will create point cloud of the given objects using virtual RGBD camera 
and wirte them to the object folder [root]/[obj_name]/base.urdf

dependency :
- [root]/[obj_name]/base.urdf
======================================================================================
'''

print(start_msg)

if __name__=="__main__":
  parser = argparse.ArgumentParser()
  # TODO: 
  # if object is hook => the pose should be assigned by p.loadURDF
  # if object is everyday object => the pose should be assigned by p.resetBasePositionAndOrientation
  parser.add_argument('--object-dir', '-id', type=str, default='models/hook_all_new')
  parser.add_argument('--file-token', '-ft', type=str, default='base')
  args = parser.parse_args()

  main(args)