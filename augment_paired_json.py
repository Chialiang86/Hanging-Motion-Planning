import glob, os, json, shutil, time
import numpy as np
import quaternion

import pybullet as p

from scipy.spatial.transform import Rotation as R
from sklearn.cluster import DBSCAN

# for motion planners
from utils.bullet_utils import get_matrix_from_pose, get_pose_from_matrix, get_matrix_from_pos_rot, draw_coordinate

hook_names = [
    'Hook_hcu_104_devil', 'Hook_hcu_134_normal', 'Hook_hcu_138_hard', 'Hook_hcu_181_hard', 'Hook_hcu_190_easy', 
    'Hook_hcu_243_normal', 'Hook_hcu_279_devil', 'Hook_hcu_293_easy', 'Hook_hcu_296_normal', 'Hook_hcu_303_normal', 
    'Hook_hcu_306_normal', 'Hook_hcu_335_normal', 'Hook_hcu_359_easy', 'Hook_hcu_362_easy', 'Hook_hcu_364_normal', 
    'Hook_hcu_376_devil', 'Hook_hcu_380_hard', 'Hook_hcu_390_easy', 'Hook_hcu_3_hard', 'Hook_hcu_75_easy', 
    'Hook_hcu_89_devil', 'Hook_hs_105_hard', 'Hook_hs_117_hard', 'Hook_hs_154_hard', 'Hook_hs_156_hard', 
    'Hook_hs_190_easy', 'Hook_hs_216_easy', 'Hook_hs_229_normal', 'Hook_hs_275_devil', 'Hook_hs_293_normal', 
    'Hook_hs_314_easy', 'Hook_hs_317_easy', 'Hook_hs_339_devil', 'Hook_hs_363_devil', 'Hook_hs_370_easy', 
    'Hook_hs_393_devil', 'Hook_hs_42_hard', 'Hook_hs_70_easy', 'Hook_hs_94_hard', 'Hook_hs_95_devil', 
    'Hook_hsr_118_hard', 'Hook_hsr_123_normal', 'Hook_hsr_125_easy', 'Hook_hsr_13_devil', 'Hook_hsr_15_normal', 
    'Hook_hsr_218_devil', 'Hook_hsr_22_normal', 'Hook_hsr_263_hard', 'Hook_hsr_298_normal', 'Hook_hsr_304_hard', 
    'Hook_hsr_312_devil', 'Hook_hsr_321_devil', 'Hook_hsr_335_hard', 'Hook_hsr_371_devil', 'Hook_hsr_381_easy', 
    'Hook_hsr_391_hard', 'Hook_hsr_56_normal', 'Hook_hsr_5_normal', 'Hook_hsr_71_easy', 'Hook_omni_124_devil'
]

def extract_contact_point(candidate_pts : np.ndarray, eps=0.002, min_samples=2):

    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(candidate_pts)
    clustering_labels = clustering.labels_

    # compute cluster height
    cluster_means_z = []
    for i in range(np.max(clustering_labels) + 1):
        cond = np.where(clustering_labels == i)
        cluster_pts = candidate_pts[cond]
        
        cluster_mean_z = np.mean(cluster_pts, axis=0)[2]
        cluster_means_z.append(cluster_mean_z)
    
    # no clustering, choose the lowest point on the object as the final contact point
    if len(cluster_means_z) == 0:
        return list(sorted(candidate_pts, key=lambda x: x[2])[0])

    cluster_means_z = np.asarray(cluster_means_z)
    
    # get the "highest" cluster
    highest_cluster_id = np.argsort(-cluster_means_z)[0]
    
    # the lowest point on the object in the highest cluster
    cond = np.where(clustering_labels == highest_cluster_id)
    highest_pts = candidate_pts[cond]
    lphc_id = np.argsort(-highest_pts[:, 1])[0]
    lphc = highest_pts[lphc_id]

    return lphc

if __name__=="__main__":

    input_kptraj_dir = 'keypoint_trajectory/everyday_objects_50'
    output_dir = 'data/data_all_new_testing_stable'

    # hook and kptraj
    hook_kptraj_jsons_all = glob.glob(f'{input_kptraj_dir}/Hook*.json')
    hook_kptraj_jsons = []

    for hook_kptraj_json in hook_kptraj_jsons_all:
        hook_name = hook_kptraj_json.split('/')[-1].split('.')[0]
        if hook_name in hook_names:
            hook_kptraj_jsons.append(hook_kptraj_json)

    # obj and kp
    obj_kp_jsons = glob.glob(f'{input_kptraj_dir}/everyday_objects*.json')

    # ------------------------ #
    # --- Setup simulation --- #
    # ------------------------ #

    # Create pybullet GUI
    # physics_client_id = p.connect(p.GUI)
    physics_client_id = p.connect(p.DIRECT)
    # p.resetDebugVisualizerCamera(2.1, 90, -30, [0.0, -0.0, -0.0])
    p.resetDebugVisualizerCamera(
        cameraDistance=0.2,
        cameraYaw=90,
        cameraPitch=-30,
        cameraTargetPosition=[0.5, 0.0, 1.3]
    )
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=150)
    sim_timestep = 1.0 / 240
    p.setTimeStep(sim_timestep)
    p.setGravity(0, 0, -9.8)
    height_thresh = 0.8

    for hook_kptraj_json in hook_kptraj_jsons[50:]:
        hook_kptraj_dict = json.load(open(hook_kptraj_json, 'r'))

        hook_name = hook_kptraj_json.split('/')[-1].split('.')[0]
        hook_urdf_path = hook_kptraj_dict['file']
        hook_trajs = hook_kptraj_dict['trajectory']
        hook_pose = hook_kptraj_dict['hook_pose']
        hook_trans = get_matrix_from_pose(hook_pose)

        last_wpt_hook = hook_trajs[0][-1]
        front_dir = np.asarray(hook_trajs[0][-1])[:3] - np.asarray(hook_trajs[0][-5])[:3]
        front_dir /= np.linalg.norm(front_dir)
        last_wpt_trans = hook_trans @ get_matrix_from_pose(last_wpt_hook)

        hook_id = p.loadURDF(hook_urdf_path, hook_pose[:3], hook_pose[3:])

        for o_i in range(len(obj_kp_jsons)):
            
            obj_kp_json = obj_kp_jsons[o_i]
            obj_name = obj_kp_json.split('/')[-1].split('.')[0]
            obj_kp_dict = json.load(open(obj_kp_json, 'r'))
            obj_urdf_path = obj_kp_dict['file']
            obj_init_poses = obj_kp_dict['initial_pose']
            obj_contact_pose = obj_kp_dict['contact_pose']
            
            dot_prod = np.dot(front_dir, last_wpt_trans[:3, 0])
            if dot_prod < 0:
                rot_180 = np.identity(4)
                rot_180[:3, :3] = R.from_rotvec([0, 0, np.pi]).as_matrix()
                last_wpt_trans_redine = last_wpt_trans @ rot_180
            else :
                last_wpt_trans_redine = np.copy(last_wpt_trans)

            obj_trans = last_wpt_trans_redine @ np.linalg.inv(get_matrix_from_pose(obj_contact_pose))
            obj_pose = list(get_pose_from_matrix(obj_trans))

            obj_id = p.loadURDF(obj_urdf_path)

            p.resetBasePositionAndOrientation(obj_id, obj_pose[:3], obj_pose[3:])

            success = True
            for _ in range(int(0.5 / sim_timestep)):
                pos, rot = p.getBasePositionAndOrientation(obj_id)
                if pos[2] < height_thresh:
                    success = False
                time.sleep(sim_timestep)
                p.stepSimulation()
            
            if p.getContactPoints(obj_id, hook_id) == ():
                success = False
            
            if not success:
                print(f'ignore {hook_name}-{obj_name}')
                p.removeBody(obj_id)
                continue
            
            output_whole_dir = f'{output_dir}/{hook_name}-everyday_objects'
            os.makedirs(output_whole_dir, exist_ok=True)

            # output_path = f'{output_whole_dir}/{hook_name}-{obj_name}.json'
            # output_dict = {
            #     'hook_path': hook_urdf_path,
            #     'obj_path': obj_urdf_path,
            #     'hook_pose': hook_pose,
            #     'contact_info': [
            #         {
            #             'contact_point_hook': [],
            #             'obj_pose': obj_pose,
            #             'contact_point_obj': [],
            #         }
            #     ],
            #     'initial_pose': obj_init_poses
            # }

            # json.dump(output_dict, open(output_path, 'w'), indent=4)
            # print(f'{output_path} has been written')

            # get_contact_point
            contact_points = p.getContactPoints(obj_id, hook_id)
            if len(contact_points) < 3:
                p.removeBody(obj_id)
                continue
            
            # add candidate contact points
            candidate_pts = []
            for contact_point in contact_points:
                candidate_pts.append(contact_point[5]) # on the object
            candidate_pts = np.asarray(candidate_pts)
            
            # relative homogeneous contact point
            contact_point = extract_contact_point(candidate_pts, eps=0.01, min_samples=3)
            contact_point_homo = np.concatenate((contact_point, [1]))

            # relative transform (hook, object)
            hook_transform = get_matrix_from_pos_rot(hook_pose[:3], hook_pose[3:])
            obj_transform = get_matrix_from_pos_rot(pos, rot)
            contact_point_hook = np.linalg.inv(hook_transform) @ contact_point_homo
            contact_point_obj = np.linalg.inv(obj_transform) @ contact_point_homo

            # draw contact point
            contact_point_homo_hook = hook_transform @ contact_point_hook
            contact_point_homo_obj = obj_transform @ contact_point_obj
            contact_point_pose_hook = list(contact_point_homo_hook[:3]) + [0, 0, 0, 1]
            contact_point_pose_obj = list(contact_point_homo_obj[:3]) + [0, 0, 0, 1]
            p.removeAllUserDebugItems()

            for i in range(int(10 / sim_timestep)):
                p.stepSimulation()
                time.sleep(sim_timestep)

            output_path = f'{output_whole_dir}/{hook_name}-{obj_name}.json'
            output_dict = {
                'hook_path': hook_urdf_path,
                'obj_path': obj_urdf_path,
                'hook_pose': hook_pose,
                'contact_info': [
                    {
                        'contact_point_hook': contact_point_hook.tolist(),
                        'obj_pose': list(pos + rot),
                        'contact_point_obj': contact_point_obj.tolist(),
                    }
                ],
                'initial_pose': obj_init_poses
            }

            json.dump(output_dict, open(output_path, 'w'), indent=4)
            print(f'{output_path} has been written')

            p.removeBody(obj_id)

        p.removeBody(hook_id)
