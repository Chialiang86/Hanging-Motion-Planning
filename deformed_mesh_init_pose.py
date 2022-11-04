import os, json, argparse, glob

def main(args):
    input_dir = args.input_dir
    assert os.path.exists(input_dir), f'{input_dir} not exists'

    all_json_dirs = os.listdir(f'{input_dir}/')
    pivot_json_dirs = []
    for json_file in all_json_dirs:
        # pivot obj id will not contain '-' in folder name
        if '#' not in json_file:
            pivot_json_dirs.append(f'{input_dir}/{json_file}')
    pivot_json_dirs.sort()

    object_list = [ 
        "daily_5",
        # "bag_5",  
        # "scissor_4", "mug_59", "wrench_1", 
        # "bag_6", "bag_70",
        # "daily_11", "daily_114", "daily_115", "daily_2", "daily_23",  
        # "daily_42", "daily_57", "daily_63", "daily_84", "daily_7", "daily_71", "daily_72",
        # "daily_85", "daily_97", "daily_8", "daily_106", "daily_41",
        # "mug_118",
        # "mug_100", "mug_11", "mug_112", "mug_113", "mug_115",  "mug_123", "mug_126", "mug_128", 
        # "mug_132", "mug_67", "mug_70", "mug_80", "mug_82", "mug_90", "mug_135", "mug_199", "mug_73", "mug_129",
        # "mug_142", "mug_146", "mug_147", "mug_149", "mug_150", "mug_159", "mug_166", "mug_184",
        # "mug_173", "mug_181", "mug_19", "mug_193", "mug_204", "mug_205", "mug_43", "mug_145", "mug_64",
        # "scissor_101", "scissor_12", "scissor_14", "scissor_19", "scissor_22", "scissor_27", "scissor_39", 
        # "scissor_48", "scissor_58", "scissor_62", "scissor_74", "scissor_79", "scissor_8", "scissor_92", 
        # "scissor_95", "scissor_98", "scissor_31",
        # "wrench_10", "wrench_12",  "wrench_17", "wrench_35", "wrench_25", 
        # "wrench_27", "wrench_31", "wrench_32", "wrench_36", "wrench_6"
    ]

    for pivot_json_dir in pivot_json_dirs:
        hook_name = pivot_json_dir.split('/')[-1].split('-')[0]
        object_dir = pivot_json_dir.split('/')[-1].split('-')[1]

        cnt = 0
        generated_dir = f'{input_dir}/{hook_name}#{cnt}-{object_dir}'
        while os.path.exists(generated_dir):
            print(f'processing {generated_dir}')

            for object_name in object_list:
                
                full_object_name = f'{object_dir}_{object_name}'
                # reading pivot path
                pivot_path = f'{hook_name}-{full_object_name}.json'
                pivot_full_path = f'{pivot_json_dir}/{pivot_path}'
                # reading generated path
                generated_path = f'{hook_name}#{cnt}-{full_object_name}.json'
                generated_full_path = f'{generated_dir}/{generated_path}'

                # read pivot/generated json path
                pivot_json_dict = None
                generated_json_dict = None
                f_pivot_json = open(pivot_full_path, 'r') 
                pivot_json_dict = json.load(f_pivot_json)
                f_pivot_json.close()
                fread_generated_json = open(generated_full_path, 'r')
                generated_json_dict = json.load(fread_generated_json)
                fread_generated_json.close()

                if "initial_pose" in generated_json_dict.keys():
                    print(f'ignore {generated_full_path}')
                    continue

                # writing initial poses
                generated_json_dict["initial_pose"] = pivot_json_dict["initial_pose"]
                fwrite_generated_json = open(generated_full_path, 'w')
                generated_json_dict = json.dump(generated_json_dict, fwrite_generated_json, indent=4)
                fwrite_generated_json.close()
                print(f'{generated_full_path} has been written.')
                    
            cnt += 1
            generated_dir = f'{input_dir}/{hook_name}#{cnt}-{object_dir}'


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-dir', '-id', type=str, default='data')
    args = parser.parse_args()
    main(args)