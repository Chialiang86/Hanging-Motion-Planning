import argparse
import open3d as o3d
import numpy as np
import time, os, json, glob

from tqdm import tqdm
from scipy.spatial.transform import Rotation as R

def assign_good(vis):

    global status 
    # You can do something here when a key is pressed
    status = "good"
    return True

def assign_bad(vis):

    global status 
    # You can do something here when a key is pressed
    status = "bad"
    return True

def assign_break(vis):

    global status 
    # You can do something here when a key is pressed
    status = "break"
    return True

def assign_x30(vis):

    global status 
    # You can do something here when a key is pressed
    status = "x30"
    return True

def assign_x30neg(vis):

    global status 
    # You can do something here when a key is pressed
    status = "x30neg"
    return True

def assign_y30(vis):

    global status 
    # You can do something here when a key is pressed
    status = "y30"
    return True

def assign_y30neg(vis):

    global status 
    # You can do something here when a key is pressed
    status = "y30neg"
    return True

def assign_z30(vis):

    global status 
    # You can do something here when a key is pressed
    status = "z30"
    return True

def assign_z30neg(vis):

    global status 
    # You can do something here when a key is pressed
    status = "z30neg"
    return True

def assign_x(vis):

    global status 
    # You can do something here when a key is pressed
    status = "x"
    return True

def assign_xneg(vis):

    global status 
    # You can do something here when a key is pressed
    status = "xneg"
    return True

def assign_y(vis):

    global status 
    # You can do something here when a key is pressed
    status = "y"
    return True

def assign_yneg(vis):

    global status 
    # You can do something here when a key is pressed
    status = "yneg"
    return True

def assign_z(vis):

    global status 
    # You can do something here when a key is pressed
    status = "z"
    return True

def assign_zneg(vis):

    global status 
    # You can do something here when a key is pressed
    status = "zneg"
    return True

def get_affine_mat(status, step_size=0.1):
    global affine_op
    assert status in affine_op, f'unknown status : {status}'
    
    retmat = np.identity(4)
    if status == "x":
        retmat[:3, 3] += np.array([step_size, 0, 0])
    if status == "xneg":
        retmat[:3, 3] -= np.array([step_size, 0, 0])
    if status == "y":
        retmat[:3, 3] += np.array([0, step_size, 0])
    if status == "yneg":
        retmat[:3, 3] -= np.array([0, step_size, 0])
    if status == "z":
        retmat[:3, 3] += np.array([0, 0, step_size])
    if status == "zneg":
        retmat[:3, 3] -= np.array([0, 0, step_size])
    if status == "x30":
        retmat[:3, :3] = R.from_rotvec([ np.pi / 6, 0, 0]).as_matrix()
    if status == "x30neg":
        retmat[:3, :3] = R.from_rotvec([-np.pi / 6, 0, 0]).as_matrix()
    if status == "y30":
        retmat[:3, :3] = R.from_rotvec([0, np.pi / 6, 0]).as_matrix()
    if status == "y30neg":
        retmat[:3, :3] = R.from_rotvec([0,-np.pi / 6, 0]).as_matrix()
    if status == "z30":
        retmat[:3, :3] = R.from_rotvec([0, 0, np.pi / 6]).as_matrix()
    if status == "z30neg":
        retmat[:3, :3] = R.from_rotvec([0, 0,-np.pi / 6]).as_matrix()
    
    return retmat

def main(args):

    input_dir = args.input_dir
    assert os.path.exists(input_dir), f'{input_dir} not exists'

    ext = args.extension
    assert ext in ['.obj', '.ply'], f'unknown file extension: {ext}'

    mesh_paths = None
    if ext == ".obj":
        mesh_paths = glob.glob(f'{input_dir}/*/*.obj')
    if ext == ".ply":
        mesh_paths = glob.glob(f'{input_dir}/*/*.ply')
    mesh_paths.sort()
    coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=args.coor_scale)

    # window for displaying point cloud
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    vis.register_key_callback(ord('Y'), assign_good) 
    vis.register_key_callback(ord('N'), assign_bad) 
    vis.register_key_callback(ord('B'), assign_break) 
    vis.register_key_callback(ord('S'), assign_x30) 
    vis.register_key_callback(ord('W'), assign_x30neg) 
    vis.register_key_callback(ord('D'), assign_y30) 
    vis.register_key_callback(ord('A'), assign_y30neg) 
    vis.register_key_callback(ord('C'), assign_z30) 
    vis.register_key_callback(ord('Z'), assign_z30neg) 
    vis.register_key_callback(ord('L'), assign_x) 
    vis.register_key_callback(ord('J'), assign_xneg) 
    vis.register_key_callback(ord('I'), assign_y) 
    vis.register_key_callback(ord('K'), assign_yneg) 
    vis.register_key_callback(ord('M'), assign_z) 
    vis.register_key_callback(ord('.'), assign_zneg) 
    vis.add_geometry(coor)

    global status 
    global affine_op

    # fout = open("res.txt", "w")

    # for mesh_path in mesh_paths:
        
    #     mesh = o3d.io.read_triangle_mesh(mesh_path)

    #     vis.add_geometry(mesh)
    #     vis.poll_events()
    #     time.sleep(1)
    #     vis.remove_geometry(mesh)

    affine_op = [
                    "x", "y", "z", "xneg", "yneg", "zneg",
                    "x30", "y30", "z30", "x30neg", "y30neg", "z30neg"
                ]

    result_path = f'{input_dir}/res.txt'

    f_res = open(result_path, 'a')
    f_res_rd = open(result_path, 'r')
    res_history = f_res_rd.readlines()

    for m_id, mesh_path in enumerate(tqdm(mesh_paths)):
        
        if len(res_history) > m_id:
            print(f'{mesh_path} exists, ignore it')
            continue

        fname = f'{os.path.splitext(mesh_path)[0]}.obj'
        if ext==".ply" and os.path.exists(fname):
            print(f'{fname} exists, ignore it')
            continue

        mesh = o3d.io.read_triangle_mesh(mesh_path)

        vis.add_geometry(mesh)
        status = ""
        while status != "good" and status != "bad":
            time.sleep(0.01)

            if status == "break":
                return

            vis.update_geometry(mesh)
            vis.poll_events()
            vis.update_renderer()

            if status in affine_op:
                affine_mat = get_affine_mat(status, args.coor_scale / 10)

                points = np.asarray(mesh.vertices)
                points_homo = np.hstack((points, np.ones((points.shape[0], 1)))).T
                points_homo = affine_mat @ points_homo
                points = points_homo.T[:, :3]
                mesh.vertices = o3d.utility.Vector3dVector(points)

                status = ""

        if status != "bad" and status != "break" and status != "":

            o3d.io.write_triangle_mesh(fname, mesh)
        
            f_res.write(f'{mesh_path} good\n')

        elif status != "break" and status != "":
            
            f_res.write(f'{mesh_path} bad\n')

        vis.remove_geometry(mesh)

        # vis.remove_geometry(mesh)
        # fout.write(f'{mesh_path} : {status}\n')
        # fout.flush()
    
    f_res.close()

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--input_dir', '-id', type=str, default='pointe_out')
    parser.add_argument('--extension', '-ext', type=str, default='.obj')
    parser.add_argument('--coor_scale', '-cs', type=float, default=1.0)
    args = parser.parse_args()
    main(args)