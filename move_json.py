import shutil, glob, json, os

if __name__=="__main__":
    src_paths = glob.glob('keypoint_pose/everyday_objects_50/*.json')
    # src_paths = glob.glob('inference_objs_50_initpose/*.json')
    dst_dir = 'models/everyday_objects_50'
    # dst_dir = 'inference_objs_50'
    for src_path in src_paths:
        src_subpath = src_path.split('/')[-1]
        obj_id = src_subpath.split('.')[0]

        prefix = 'everyday_objects_50_'
        # prefix = 'everyday_objects_50_'
        out_dir = f'{dst_dir}/{obj_id[len(prefix):]}'
        dst_path = f'{out_dir}/{src_subpath}'
        print(f'{src_path} => {dst_path}')
        shutil.copy(src_path, dst_path)