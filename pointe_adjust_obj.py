import argparse
import open3d as o3d
import numpy as np
import time, os, json, glob, keyboard

from tqdm import tqdm
from scipy.spatial.transform import Rotation as R


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

def assign_0(vis):

    global category
    # You can do something here when a key is pressed
    category = "0"
    return True

def assign_1(vis):

    global category
    # You can do something here when a key is pressed
    category = "1"
    return True

def assign_2(vis):

    global category
    # You can do something here when a key is pressed
    category = "2"
    return True

def assign_3(vis):

    global category
    # You can do something here when a key is pressed
    category = "3"
    return True

def assign_4(vis):

    global category
    # You can do something here when a key is pressed
    category = "4"
    return True

def assign_scaleup(vis):

    global status 
    # You can do something here when a key is pressed
    status = "scaleup"
    return True

def assign_scaledown(vis):

    global status 
    # You can do something here when a key is pressed
    status = "scaledown"
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
        retmat[:3, :3] = R.from_rotvec([ np.pi / 12, 0, 0]).as_matrix()
    if status == "x30neg":
        retmat[:3, :3] = R.from_rotvec([-np.pi / 12, 0, 0]).as_matrix()
    if status == "y30":
        retmat[:3, :3] = R.from_rotvec([0, np.pi / 12, 0]).as_matrix()
    if status == "y30neg":
        retmat[:3, :3] = R.from_rotvec([0,-np.pi / 12, 0]).as_matrix()
    if status == "z30":
        retmat[:3, :3] = R.from_rotvec([0, 0, np.pi / 12]).as_matrix()
    if status == "z30neg":
        retmat[:3, :3] = R.from_rotvec([0, 0,-np.pi / 12]).as_matrix()
    if status == "scaleup":
        retmat[0, 0] = 1.1
        retmat[1, 1] = 1.1
        retmat[2, 2] = 1.1
    if status == "scaledown":
        retmat[0, 0] = 0.9
        retmat[1, 1] = 0.9
        retmat[2, 2] = 0.9
    
    return retmat

def main(args):

    input_dir = args.input_dir
    assert os.path.exists(input_dir), f'{input_dir} not exists'

    ext = args.extension
    assert ext in ['.obj', '.ply'], f'unknown file extension: {ext}'

    mesh_paths = None
    if ext == ".obj":
        mesh_paths = glob.glob(f'{input_dir}/*/*_normalized.obj')
    if ext == ".ply":
        mesh_paths = glob.glob(f'{input_dir}/*/*.ply')
    mesh_paths.sort()
    coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=args.coor_scale)

    # window for displaying point cloud
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
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
    vis.register_key_callback(ord('0'), assign_0) 
    vis.register_key_callback(ord('1'), assign_1) 
    vis.register_key_callback(ord('2'), assign_2) 
    vis.register_key_callback(ord('3'), assign_3) 
    vis.register_key_callback(ord('4'), assign_4) 
    vis.register_key_callback(265, assign_scaleup) # up arrow
    vis.register_key_callback(264, assign_scaledown) # down arrow 
    vis.add_geometry(coor)

    global status 
    global category 
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
                    "x30", "y30", "z30", "x30neg", "y30neg", "z30neg", "scaleup", "scaledown"
                ]

    os.makedirs(args.output_dir, exist_ok=True)
    result_path = f'{args.output_dir}/res.txt'

    if os.path.exists(result_path):
        f_res = open(result_path, 'a')
    else :
        f_res = open(result_path, 'w')
    f_res_rd = open(result_path, 'r')
    res_history = f_res_rd.readlines()

    # mesh.vertices = mesh.vertices * target_length / length

    for m_id, mesh_path in enumerate(tqdm(mesh_paths)):

        # if 'cooking' in mesh_path:
        #     continue
        # if 'wrench' in mesh_path:
        #     continue

        cont = False
        for line in res_history:
            if mesh_path in line:
                print(f'{mesh_path} exists, ignore it')
                cont = True
        if cont:
            continue

        mesh = o3d.io.read_triangle_mesh(mesh_path)
        
        # normalize to a 0.1m width bounding box 
        size = np.array(np.max(mesh.vertices, axis=0)
                        - np.min(mesh.vertices, axis=0))
        length = np.max(size)
        mesh.scale( 0.1 / length, np.array([0, 0, 0]))

        vis.add_geometry(mesh)
        status = ""
        category = ""
        category_dict = {
            "0": "",
            "1": "easy",
            "2": "normal",
            "3": "hard",
            "4": "devil",
        }
        while status != "bad" and category == "":
            time.sleep(0.01)

            if status == "break":
                return

            vis.update_geometry(mesh)
            vis.poll_events()
            vis.update_renderer()

            if status in affine_op:

                if status == "scaleup" or status == "scaledown":

                    if status == "scaleup":
                        mesh.scale(1.1, np.array([0, 0, 0]))
                    if status == "scaledown":
                        mesh.scale(0.9, np.array([0, 0, 0]))
                else :

                    affine_mat = get_affine_mat(status, args.coor_scale / 10)
                    points = np.asarray(mesh.vertices)
                    points_homo = np.hstack((points, np.ones((points.shape[0], 1)))).T
                    points_homo = affine_mat @ points_homo
                    points = points_homo.T[:, :3]
                    mesh.vertices = o3d.utility.Vector3dVector(points)

                status = ""

        if category != "":
            
            if category_dict[category] != "":
                fname = '{}/{}_{}/{}.obj'.format(
                    args.output_dir, 
                    mesh_path.split('/')[-2], 
                    category_dict[category],
                    os.path.splitext(os.path.split(mesh_path)[-1])[0]
                )
            else:
                fname = '{}/{}/{}.obj'.format(
                    args.output_dir, 
                    mesh_path.split('/')[-2], 
                    os.path.splitext(os.path.split(mesh_path)[-1])[0]
                )
            print(fname)

            # if ext==".ply" and os.path.exists(fname):
            #     print(f'{fname} exists, ignore it')
            #     continue
                

            # create new object folder
            os.makedirs(os.path.split(fname)[0], exist_ok=True)

            o3d.io.write_triangle_mesh(fname, mesh)        
            f_res.write(f'{mesh_path} {category_dict[category]}\n')

        elif status != "break" and status != "":
            
            f_res.write(f'{mesh_path} bad\n')

        vis.remove_geometry(mesh)

        # vis.remove_geometry(mesh)
        # fout.write(f'{mesh_path} : {status}\n')
        # fout.flush()
    
    f_res.close()

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--input_dir', '-id', type=str, default='pointe_out/hcu')
    parser.add_argument('--output_dir', '-od', type=str, default='pointe_out/hcu_selected')
    parser.add_argument('--extension', '-ext', type=str, default='.obj')
    parser.add_argument('--coor_scale', '-cs', type=float, default=0.1)
    args = parser.parse_args()
    main(args)