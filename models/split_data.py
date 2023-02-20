import os, glob, shutil, argparse
import numpy as np

def main(args):

    assert os.path.exists(args.input_dir), f'{args.input_dir} not exists'

    obj_ids = glob.glob(f'{args.input_dir}/*')
    obj_ids.sort()

    easy_obj_ids = []
    normal_obj_ids = []
    hard_obj_ids = []
    devil_obj_ids = []
    for obj_id in obj_ids:
        if 'easy' in obj_id:
            easy_obj_ids.append(obj_id)
        if 'normal' in obj_id:
            normal_obj_ids.append(obj_id)
        if 'hard' in obj_id:
            hard_obj_ids.append(obj_id)
        if 'devil' in obj_id:
            devil_obj_ids.append(obj_id)
    
    max_iter = np.max(np.asarray([len(easy_obj_ids), len(normal_obj_ids), len(hard_obj_ids), len(devil_obj_ids)]))
    for iter, idx in enumerate(range(0, max_iter, 25)):
        out_dir = f'{args.input_dir}_{iter}'
        os.makedirs(out_dir, exist_ok=True)

        for obj_dir in easy_obj_ids[idx:idx+25]:
            obj_id = obj_dir.split('/')[-1]
            output_dir = f'{out_dir}/{obj_id}'
            print(f'{obj_id} => {output_dir}')
            shutil.copytree(obj_dir, output_dir)
        
        for obj_dir in normal_obj_ids[idx:idx+25]:
            obj_id = obj_dir.split('/')[-1]
            output_dir = f'{out_dir}/{obj_id}'
            print(f'{obj_id} => {output_dir}')
            shutil.copytree(obj_dir, output_dir)
        
        for obj_dir in hard_obj_ids[idx:idx+25]:
            obj_id = obj_dir.split('/')[-1]
            output_dir = f'{out_dir}/{obj_id}'
            print(f'{obj_id} => {output_dir}')
            shutil.copytree(obj_dir, output_dir)
        
        for obj_dir in devil_obj_ids[idx:idx+25]:
            obj_id = obj_dir.split('/')[-1]
            output_dir = f'{out_dir}/{obj_id}'
            print(f'{obj_id} => {output_dir}')
            shutil.copytree(obj_dir, output_dir)


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-input_dir', '-id', type=str, default='hook_all_new')
    args = parser.parse_args()

    main(args)
