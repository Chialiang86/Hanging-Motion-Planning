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

  pcd_dir = args.input_dir
  assert os.path.exists(pcd_dir), f'{pcd_dir} not exists'

  pcd_paths = glob.glob(f'{pcd_dir}/*/base.ply')

  # for hook_i in range(len(hook_pcd_paths)):
  #   pcd = o3d.io.read_point_cloud(hook_pcd_paths[hook_i])
  #   name = hook_pcd_paths[hook_i].split('/')[-2]
  #   path = f'visualization_{name}.png'
  #   origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
  #   o3d.visualization.draw_geometries([ pcd], point_show_normal=True)
    # render(pcd, path)

  for obj_i in range(len(pcd_paths)):
    pcd = o3d.io.read_point_cloud(pcd_paths[obj_i])
    name = pcd_paths[obj_i].split('/')[-2]
    path = f'visualization_{name}.png'
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    o3d.visualization.draw_geometries([pcd], point_show_normal=False)
    # render(pcd, path)


if __name__=="__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--input-dir', '-id', type=str, default='models/geo_data/hanging_exp')
  args = parser.parse_args()
  main(args)