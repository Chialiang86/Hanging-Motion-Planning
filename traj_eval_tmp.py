import json, os, glob
import time
import numpy as np
import quaternion
from scipy.spatial.transform import Rotation as R
import pybullet as p
import pybullet_data

from utils.bullet_utils import get_matrix_from_pose, get_pose_from_matrix

PENETRATION_THRESHOLD = 0.001
def waypoint_score(hook_id : int, obj_id : int):

    # thresh = 0.001
    p.performCollisionDetection()

    contact_points = p.getContactPoints(bodyA=hook_id, bodyB=obj_id)
    # closest_points = p.getClosestPoints(bodyA=hook_id, bodyB=obj_id, distance=thresh)
    # within_thresh = 1 if len(closest_points) > 0 else 0

    penetration = 0.0
    for contact_point in contact_points:
        # contact distance, positive for separation, negative for penetration
        contact_distance = contact_point[8] 
        penetration = min(contact_distance, penetration) if contact_distance < 0 else 0.0

    return penetration

def main():

    # Create pybullet GUI
    # physics_client_id = p.connect(p.DIRECT)
    physics_client_id = p.connect(p.GUI)
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

    # ------------------- #
    # --- Setup robot --- #
    # ------------------- #

    # Load plane contained in pybullet_data
    p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
    
    traj_paths = glob.glob(f'0308_tmp/*.json')
    traj_paths.sort()
    test_steps = 40
    full_mean_penetrations = []

    max_pene = 0.0
    max_pene_traj = []
    max_pene_obj_path = ''
    max_pene_hook_path = ''
    max_pene_hook_pose = []
    for traj_path in traj_paths:
        tmp_dict = json.load(open(traj_path, 'r'))
        obj_path = tmp_dict['obj_file']
        obj_pose = tmp_dict['obj_pose']
        hook_path = tmp_dict['hook_file']
        hook_pose = tmp_dict['hook_pose']
        obj_trajs = tmp_dict['obj_trajs']

        obj_id = p.loadURDF(obj_path)
        hook_id = p.loadURDF(hook_path, hook_pose[:3], hook_pose[3:])

        for obj_traj in obj_trajs:
            
            test_traj = obj_traj[-test_steps:]
            penetration_mean = 0
            for wpt in test_traj:
                p.resetBasePositionAndOrientation(obj_id, wpt[:3], wpt[3:])

                penetration = waypoint_score(hook_id=hook_id, obj_id=obj_id)
                penetration_mean += penetration

            penetration_mean /= test_steps
            if max_pene > penetration_mean:
                max_pene = penetration_mean
                max_pene_traj = obj_traj
                max_pene_obj_path = obj_path
                max_pene_hook_path = hook_path
                max_pene_hook_pose = hook_pose

            full_mean_penetrations.append(penetration_mean)

        p.removeBody(obj_id)
        p.removeBody(hook_id)

    obj_id = p.loadURDF(max_pene_obj_path)
    hook_id = p.loadURDF(max_pene_hook_path, max_pene_hook_pose[:3], max_pene_hook_pose[3:])
    for wpt in max_pene_traj:
        p.resetBasePositionAndOrientation(obj_id, wpt[:3], wpt[3:]) 
        time.sleep(0.03)
    p.removeBody(obj_id)
    p.removeBody(hook_id)

    full_mean_penetrations = np.asarray(full_mean_penetrations)
    ind = np.argsort(full_mean_penetrations)[:int(0.2 * full_mean_penetrations.shape[0])]

    full_mean_penetrations = np.asarray(full_mean_penetrations)
    print('==========================================================')
    print('max penetration dist = {:.08f}'.format(-np.mean(full_mean_penetrations[ind])))
    print('==========================================================')

if __name__=="__main__":
    main()