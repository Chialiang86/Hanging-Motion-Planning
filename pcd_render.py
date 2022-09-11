import glob, os
import open3d as o3d
import argparse
import time

def render(pcd, path):

  # Visualize Point Cloud
  vis = o3d.visualization.Visualizer()
  vis.create_window()
  vis.add_geometry(pcd)
  vis.update_geometry(pcd)

  # Updates
  vis.poll_events()
  vis.update_renderer()

  # Capture image
  time.sleep(0.5)
  vis.capture_screen_image(path)

  # Close
  vis.destroy_window()


def main(args):

  obj_dir = args.obj
  hook_dir = args.hook
  assert os.path.exists(obj_dir), f'{obj_dir} not exists'
  assert os.path.exists(hook_dir), f'{hook_dir} not exists'

  obj_pcd_paths = glob.glob(f'{obj_dir}/*/base.ply')
  hook_pcd_paths = glob.glob(f'{hook_dir}/*/base.ply')

  # for hook_i in range(len(hook_pcd_paths)):
  #   pcd = o3d.io.read_point_cloud(hook_pcd_paths[hook_i])
  #   name = hook_pcd_paths[hook_i].split('/')[-2]
  #   path = f'visualization_{name}.png'
  #   origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
  #   o3d.visualization.draw_geometries([ pcd], point_show_normal=True)
    # render(pcd, path)

  for obj_i in range(len(obj_pcd_paths)):
    pcd = o3d.io.read_point_cloud(obj_pcd_paths[obj_i])
    name = obj_pcd_paths[obj_i].split('/')[-2]
    path = f'visualization_{name}.png'
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    o3d.visualization.draw_geometries([ pcd], point_show_normal=True)
    # render(pcd, path)


if __name__=="__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--obj', type=str, default='models/geo_data/hanging_exp')
  parser.add_argument('--hook', type=str, default='models/hook')
  args = parser.parse_args()
  main(args)