import glob, json, os

if __name__=="__main__":
    dst_dir = 'keypoint_pose/everyday_objects_50'
    dst_jsons = glob.glob(f'{dst_dir}/*.json')
    for dst_json in dst_jsons:

        if 'Hook_my_bar_easy.json' in dst_json:
            print(f'ignore: {dst_json}')
            continue
        object_id = dst_json.split('/')[-1].split('.')[0]
        src_path = f'data/everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-{object_id}.json'

        src_dict = json.load(open(src_path, 'r'))

        dst_dict = json.load(open(dst_json, 'r'))
        dst_dict['initial_pose'] = src_dict['initial_pose']

        json.dump(dst_dict, open(dst_json, 'w'), indent=4)