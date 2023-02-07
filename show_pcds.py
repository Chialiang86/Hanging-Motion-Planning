
import open3d as o3d
import numpy as np
import time, os, json, glob


def assign_good(vis):

    global status 
    # You can do something here when a key is pressed
    status = "good"
    return False

def assign_bad(vis):

    global status 
    # You can do something here when a key is pressed
    status = "bad"
    return True

def main():
    pcd_paths = glob.glob(f'output/a hook for hanging spatula*.npz')
    pcd_paths.sort()
    mesh_paths = glob.glob(f'output/*.ply')
    mesh_paths.sort()
    coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)

    # window for displaying point cloud
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    vis.register_key_callback(ord('Y'), assign_good) # y
    vis.register_key_callback(ord('N'), assign_bad) # n
    vis.add_geometry(coor)

    global status 

    fout = open("res.txt", "w")

    for mesh_path in mesh_paths:
        
        mesh = o3d.io.read_triangle_mesh(mesh_path)

        vis.add_geometry(mesh)
        vis.poll_events()
        time.sleep(1)
        vis.remove_geometry(mesh)


    # for pcd_path in pcd_paths:
        
        # pcd_arr = np.load(pcd_path)
        # points = pcd_arr['coords']
        # colors = np.vstack((pcd_arr['R'], pcd_arr['G'], pcd_arr['B'])).T

        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(points)
        # pcd.colors = o3d.utility.Vector3dVector(colors)

        # vis.add_geometry(pcd)
        # status = ""
        # while status == "":
        #     vis.poll_events()
        # vis.remove_geometry(pcd)

        # # vis.remove_geometry(pcd)
        # print(f'{pcd_path} : {status}')
        # fout.write(f'{pcd_path} : {status}\n')
        # fout.flush()
    
    fout.close()

if __name__=="__main__":
    main()