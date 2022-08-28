import argparse
import os
import glob
from tqdm import tqdm
import pybullet as p


def main(args):
    input_dir = args.input_dir
    assert os.path.exists(input_dir), f'{input_dir} not exists'

    pathes = glob.glob(f'{input_dir}/*/*_normalized.obj')
    for path in tqdm(pathes):
        p.vhacd(path, path, f'{path}.txt')

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-dir', '-i', type=str)
    args = parser.parse_args()
    main(args)