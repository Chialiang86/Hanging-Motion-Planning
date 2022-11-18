import argparse, os, glob, json
import numpy as np

from utils.motion_planning_utils import get_sample7d_fn, get_distance7d_fn, get_extend7d_fn, get_collision7d_fn
from pybullet_planning.motion_planners.meta import smooth_path

def main(args):

    assert os.path.exists(args.input_dir), f'{args.input_dir} not exists'

    input_dir = args.input_dir
    trajectory_files = glob.glob(f'{input_dir}/Hook*.json')

    for trajectory_file in trajectory_files:
        a = 0

    return

if __name__=="__main__":
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-dir', '-id', type=str, default='keypoint_trajectory_1104')
    parser.add_argument('--aug-num', '-an', type=int, default=100)
    args = parser.parse_args()
    
    main(args)