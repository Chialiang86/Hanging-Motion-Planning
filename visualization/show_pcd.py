import glob, os, cv2
import numpy as np
import open3d as o3d
import argparse
import time
from scipy.spatial.transform import Rotation as R

# Visualize Point Cloud
vis = o3d.visualization.Visualizer()
vis.create_window()

def render(geometries, path):

  for geometrie in geometries:
    vis.add_geometry(geometrie)
    vis.update_geometry(geometrie)

  # Updates
  vis.poll_events()
  vis.update_renderer()

  # Capture image
  vis.capture_screen_image(path)

  for geometrie in geometries:
    vis.remove_geometry(geometrie)

def main(args):

  pcd_dir = args.input_dir
  assert os.path.exists(pcd_dir), f'{pcd_dir} not exists'

  # pcd_paths = glob.glob(f'{pcd_dir}/*/base-0.ply')
  pcd_paths = glob.glob(f'{pcd_dir}/*/*-0.ply')
  sub_dir = pcd_dir.split('/')[-1]
  out_dir = f'visualization/0603/{sub_dir}_afford'
  os.makedirs(out_dir, exist_ok=True)

  # for hook_i in range(len(hook_pcd_paths)):
  #   pcd = o3d.io.read_point_cloud(hook_pcd_paths[hook_i])
  #   name = hook_pcd_paths[hook_i].split('/')[-2]
  #   path = f'visualization_{name}.png'
  #   origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
  #   o3d.visualization.draw_geometries([ pcd], point_show_normal=True)
    # render(pcd, path)
  
  # for obj_i in range(len(pcd_paths)):
  #   # afford = np.load(pcd_paths[obj_i])
  #   pcd = o3d.io.read_point_cloud(pcd_paths[obj_i])
  #   pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(afford[:,:3])

    # colors = cv2.applyColorMap((255 * afford[:,3]).astype(np.uint8), colormap=cv2.COLORMAP_JET).squeeze()
    # colors = cv2.applyColorMap((255 * afford[:,4]).astype(np.uint8), colormap=cv2.COLORMAP_JET).squeeze()
    # colors = cv2.applyColorMap((255 * (afford[:,3] + afford[:,4]) / 2).astype(np.uint8), colormap=cv2.COLORMAP_JET).squeeze()
    # colors = colors[:,::-1]
    # pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)

    # sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.002)
    # sphere.translate(afford[0,:3].reshape(3,1))
    # sphere.rotate(R.from_rotvec([0, np.pi / 3, 0]).as_matrix(), center=(0, 0, 0))
    # colors = np.zeros(np.asarray(sphere.vertices).shape)
    # colors[:] = np.array([30, 30, 30])
    # sphere.vertex_colors = o3d.utility.Vector3dVector(colors / 255.0)

    # pcd.rotate(R.from_rotvec([0, np.pi / 3, 0]).as_matrix(), center=(0, 0, 0))
    # name = pcd_paths[obj_i].split('/')[-2]
    # path = f'{out_dir}/{name}_afford.png'
    # origin = o3d.geometry.Trsegsh.create_coordinate_frame(size=0.1)
    # o3d.visualization.draw_geometries([sphere,pcd], point_show_normal=False)
    # render([pcd, sphere], path)
    # render([pcd], path)

  for obj_i in range(len(pcd_paths)):
    pcd = o3d.io.read_point_cloud(pcd_paths[obj_i])
    colors = np.zeros(np.asarray(pcd.points).shape)
    colors[:] = np.array([76 / 255, 52 / 255, 35 / 255])
    pcd.colors = o3d.utility.Vector3dVector(colors)
    pcd.rotate(R.from_rotvec([0, np.pi / 3, 0]).as_matrix(), center=(0, 0, 0))
    name = pcd_paths[obj_i].split('/')[-2]
    path = f'{out_dir}/{name}_pcd.png'
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    # o3d.visualization.draw_geometries([pcd], point_show_normal=False)
    render([pcd], path)
  # Close
  vis.destroy_window()


if __name__=="__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--input-dir', '-id', type=str, default='models/hook_all_new')
  args = parser.parse_args()
  main(args)