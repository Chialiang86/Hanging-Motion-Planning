import argparse, os, glob, shutil
import open3d as o3d

from tqdm import tqdm

def main(args):
    src_dir = args.src
    dst_dir = args.dst

    assert os.path.exists(src_dir), f'{src_dir} not exists'

    obj_paths = glob.glob(f'{src_dir}/*.ply')

    # config output directory

    for obj_path in tqdm(obj_paths):

        mesh = o3d.io.read_triangle_mesh(obj_path)
        serial_num = int(os.path.splitext(obj_path)[0].split('/')[-1].split('-')[-1])

        output_dir = f'{dst_dir}/Hook_{dst_dir}_{serial_num}'
        os.makedirs(output_dir, exist_ok=True)
        
        output_path = f'{output_dir}/{dst_dir}_{serial_num}_normalized.obj'
        o3d.io.write_triangle_mesh(output_path, mesh)

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--src', '-src', type=str, default='a hook for hanging scissors')
    parser.add_argument('--dst', '-dst', type=str, default='hsr')
    args = parser.parse_args()

    main(args)