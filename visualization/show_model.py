import argparse
import open3d as o3d
import imageio
import os
import glob
import cv2
import copy
import numpy as np

from tqdm import tqdm
from scipy.spatial.transform import Rotation as R

def capture_from_viewer(mesh):
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    vis.add_geometry(mesh)

    # Updates
    vis.update_geometry(mesh)
    vis.poll_events()
    vis.update_renderer()

    o3d_screenshot_mat = vis.capture_screen_float_buffer(do_render=True) # need to be true to capture the image
    o3d_screenshot_mat = (255.0 * np.asarray(o3d_screenshot_mat)).astype(np.uint8)
    o3d_screenshot_mat = cv2.cvtColor(o3d_screenshot_mat,cv2.COLOR_BGR2RGB)
    o3d_screenshot_mat = cv2.resize(o3d_screenshot_mat, (o3d_screenshot_mat.shape[1] // 6, o3d_screenshot_mat.shape[0] // 6))
    vis.destroy_window()

    return o3d_screenshot_mat

def main(args):

    input_dir = args.input_dir
    assert os.path.exists(input_dir), f'{input_dir} not exists'

    sub_dir = input_dir.split('/')[-1]
    out_dir = f'visualization/hanging_objects/{sub_dir}'
    os.makedirs(out_dir, exist_ok=True)

    # frames for visualization
    obj_files = glob.glob(f'{input_dir}/*/base.obj')
    obj_files.sort()
    frames = 36
    rotate_per_frame = (2 * np.pi) / frames

    for obj_file in tqdm(obj_files):
        
        mesh = o3d.io.read_triangle_mesh(obj_file)
        mesh_90 = copy.deepcopy(mesh)
        r_90 = mesh.get_rotation_matrix_from_xyz((0, 0, np.pi / 2)) # (rx, ry, rz) = (right, up, inner)
        mesh_90.rotate(r_90, center=(0, 0, 0))

        obj_dir = os.path.split(obj_file)[0].split('/')[-1]

        img_list = []
        # img_list_90 = []
        for _ in range(frames):
            r = mesh.get_rotation_matrix_from_xyz((0, rotate_per_frame, 0)) # (rx, ry, rz) = (right, up, inner)
            mesh.rotate(r, center=(0, 0, 0))
            # mesh_90.rotate(r, center=(0, 0, 0))

            img = capture_from_viewer(mesh)
            # img_90 = capture_from_viewer(mesh_90)
            img_list.append(img)
            # img_list_90.append(img_90)
        
        save_path = f'{out_dir}/{sub_dir}_{obj_dir}-0.gif'
        # save_path_90 = f'{out_dir}/{sub_dir}_{obj_dir}-90.gif'
        imageio.mimsave(save_path, img_list, fps=10)
        # imageio.mimsave(save_path_90, img_list_90, fps=20)
        # print(f'{save_path} and {save_path_90} saved')

if __name__=="__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--input-dir', '-id', type=str, default='models/geo_data/hanging_exp')
    args = parser.parse_args()

    main(args)