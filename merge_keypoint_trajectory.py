import glob, os, json, shutil, argparse

from tqdm import tqdm

def main(args):

    input_subdirs = [
        "kptraj_all_new_1-0", 
        "kptraj_all_new_1-1", 
    ]

    input_root = args.input_root
    output_dir = args.output_dir

    assert os.path.exists(input_root), f'{input_root} not exists'

    for input_subdir in input_subdirs:
        input_dir = f'{input_root}/{input_subdir}'
        assert os.path.exists(input_root), f'{input_root} not exists'

        hook_jsons = glob.glob(f'{input_dir}/Hook*.json')
        hook_jsons.sort()

        for hook_json in tqdm(hook_jsons):

            sub_path = os.path.split(hook_json)[-1]
            out_hook_json = f'{input_root}/{output_dir}/{sub_path}'
            if not os.path.exists(out_hook_json):
                
                shutil.copy2(hook_json, out_hook_json)
            
            else :
                
                f_exists = open(out_hook_json, 'r')
                hook_dict_exists = json.load(f_exists)
                f_exists.close()

                trajectories_exists = hook_dict_exists['trajectory']
                
                f_src = open(hook_json, 'r')
                hook_dict_src = json.load(f_src)
                f_src.close()

                trajectories_src = hook_dict_src['trajectory']
                trajectories_exists.extend(trajectories_src)

                hook_dict_exists['trajectory'] = trajectories_exists
                f_out = open(out_hook_json, 'w')
                json.dump(hook_dict_exists, f_out, indent=4)
                f_out.close()


if __name__=="__main__":
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-root', '-ir', type=str, default='keypoint_trajectory')
    parser.add_argument('--output-dir', '-od', type=str, default='kptraj_all_new_1')
    args = parser.parse_args()

    main(args)