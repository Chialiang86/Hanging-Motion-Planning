import glob

import matplotlib.pyplot as plt
import numpy as np

import pybullet as p

# def reset_pose(obj_id, x_offset=0.1, y_offset=0., z_offset=1.):
#     x = x_offset
#     y = y_offset
#     z = z_offset

#     roll = ( 0 * np.pi / 180 + np.random.uniform(-np.pi / 18, np.pi / 18))
#     pitch = (  0 * np.pi / 180 + np.random.uniform(-np.pi / 2, np.pi / 6))
#     yaw = (90 * np.pi / 180 + np.random.uniform(-np.pi / 18, np.pi / 18))
#     p.setGravity(0, 0, 0)
#     p.resetBasePositionAndOrientation(
#         obj_id,
#         [x, y, z],
#         p.getQuaternionFromEuler([0, pitch, np.pi/2]))

def main():

    paths = glob.glob(f'models/hook_all/*/*.npy')
    paths.sort()

    thresh = 2000
    print(f'thresh : {thresh}')
    num_pts = []
    for path in paths:
        pcd = np.load(path)
        num_pt = pcd.shape[0]
        num_pts.append(num_pt)

        if num_pt < thresh:
            print(f'less than {thresh} points in {path}')
    
    num_pts = np.asarray(num_pts)

    cond = np.where(num_pts < thresh)
    print(f'=================================')
    print(f'num pcd: {len(num_pts)}')
    print(f'mean: {np.mean(num_pts)}')
    print(f'max: {np.max(num_pts)}')
    print(f'min: {np.min(num_pts)}')
    print(f'pts < {thresh}: {len(cond[0])}/{len(num_pts)}')
    print(f'=================================')

    interval = 1000
    low = int(np.floor(np.min(num_pts) / interval) * interval)
    high = int(np.ceil(np.max(num_pts) / interval) * interval)
    cnts = []
    for inter in range(low, high, interval):
        cond = np.where((num_pts >= inter) & (num_pts < inter + interval))
        cnt = len(cond[0])
        cnts.append(cnt)
    
    xticks = [f'{num/1000}k ~ {(num + interval)/1000}k' for num in range(low, high, interval)]

    plt.figure(figsize=(8, 12))
    plt.ylabel('count')
    plt.xlabel('num of points')
    plt.xticks(range(len(cnts)), xticks, rotation='vertical')
    plt.bar(range(len(cnts)), cnts)
    plt.title('Point Cloud Statitics')
    
    plt.savefig(f'hook_data.png')

    # physics_client_id = p.connect(p.GUI)
    # # p.resetDebugVisualizerCamera(2.1, 90, -30, [0.0, -0.0, -0.0])
    # p.resetDebugVisualizerCamera(
    #     cameraDistance=0.15,
    #     cameraYaw=90,
    #     cameraPitch=0,
    #     cameraTargetPosition=[0.5, -0.05, 1.3]
    # )
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    # p.resetSimulation()
    # p.setPhysicsEngineParameter(numSolverIterations=150)
    # sim_timestep = 1.0 / 1000
    # gravity = -9.8
    # p.setTimeStep(sim_timestep)

    # time.sleep(1.0)

    # path = 'models/daily/daily_5/base.urdf'
    # obj_id = p.loadURDF(path)

    # while True:
    #     p.stepSimulation()
    #     keys = p.getKeyboardEvents()
    #     if ord('q') in keys and keys[ord('q')] == 3 and p.KEY_IS_DOWN: 
    #         break
        
    #     reset_pose(obj_id, 0.5, -0.05, 1.3)

    # directory = 'data/data_all'
    # json_paths = glob.glob(f'{directory}/*/*.json')
    # json_paths.sort()

    # for json_path in json_paths:

    #     json_dict = json.load(open(json_path, 'r'))
    #     print(f'processing {json_path}')
    #     assert 'hook_path' in json_dict.keys() and \
    #         'obj_path' in json_dict.keys() and \
    #         'hook_pose' in json_dict.keys() and \
    #         'contact_info' in json_dict.keys() and \
    #         'contact_point_hook' in json_dict['contact_info'][0].keys() and \
    #         'contact_point_obj' in json_dict['contact_info'][0].keys() and \
    #         'obj_pose' in json_dict['contact_info'][0].keys()

if __name__=="__main__":
    main()