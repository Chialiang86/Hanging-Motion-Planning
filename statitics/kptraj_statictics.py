import os, glob, argparse, json
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm


def main(args):

    kptraj_root = args.kptraj_root
    kptraj_dir = f'{args.kptraj_root}/{args.kptraj_dir}'

    assert os.path.exists(kptraj_root), f'{kptraj_root} not exists'
    assert os.path.exists(kptraj_dir), f'{kptraj_dir} not exists'

    kptraj_jsons = glob.glob(f'{kptraj_dir}/Hook*.json')
    kptraj_jsons.sort()

    kptraj_nums = []
    thresh = 80
    for kptraj_json in tqdm(kptraj_jsons):
        kptraj_dict = json.load(open(kptraj_json, 'r'))
        kptraj = kptraj_dict['trajectory']
        kptraj_num = len(kptraj)
        kptraj_nums.append(kptraj_num)

        if kptraj_num < thresh:
            print(f'{kptraj_json} contain only less-than-{thresh} trajectories: {kptraj_num}')
    
    kptraj_nums = np.asarray(kptraj_nums)
    interval = 20
    low = int(np.floor(np.min(kptraj_nums) / interval) * interval)
    high = int(np.ceil(np.max(kptraj_nums) / interval) * interval)
    cnts = []
    for inter in range(low, high, interval):
        cond = np.where((kptraj_nums >= inter) & (kptraj_nums < inter + interval))
        cnt = len(cond[0])
        cnts.append(cnt)
    
    xticks = [f'{num}' for num in range(low, high, interval)]

    plt.figure(figsize=(8, 12))
    plt.ylabel('count')
    plt.xlabel('num of points')
    plt.xticks(range(len(cnts)), xticks, rotation='vertical')
    plt.bar(range(len(cnts)), cnts)
    plt.title('Point Cloud Statitics')
    
    plt.savefig(f'hook_kptraj.png')

    print(f'====================================================')
    print(f'keypoint trajectory directory: {kptraj_dir}')
    print(f'mean_traj_num: {np.mean(kptraj_nums)}')
    print(f'min_traj_num: {np.min(kptraj_nums)}')
    print(f'max_traj_num: {np.max(kptraj_nums)}')
    print(f'====================================================')

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--kptraj_root', '-kr', type=str, default='keypoint_trajectory')
    parser.add_argument('--kptraj_dir', '-kd', type=str, default='kptraj_all_new')
    args = parser.parse_args()
    main(args)