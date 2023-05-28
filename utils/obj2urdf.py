#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os
import glob
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import open3d as o3d
import trimesh
import pybullet as p

from tqdm import tqdm

def open3d_to_trimesh(src):
    """Convert mesh from open3d to trimesh

    https://github.com/wkentaro/morefusion/blob/b8b892b3fbc384982a4929b1418ee29393069b11/morefusion/utils/open3d_to_trimesh.py

    Parameters
    ----------
    src : open3d.open3d.geometry.TriangleMesh

    Returns
    -------
    dst : trimesh.base.Trimesh

    Raises
    ------
    ValueError
        when type of src is not open3d.open3d.geometry.TriangleMesh
    """
    if isinstance(src, o3d.geometry.TriangleMesh):
        vertex_colors = None
        if src.has_vertex_colors:
            vertex_colors = np.asarray(src.vertex_colors)
        dst = trimesh.Trimesh(
            vertices=np.asarray(src.vertices),
            faces=np.asarray(src.triangles),
            vertex_normals=np.asarray(src.vertex_normals),
            vertex_colors=vertex_colors,
        )
    else:
        raise ValueError("Unsupported type of src: {}".format(type(src)))

    return dst

def create_urdf(mesh, file):
    """Create urdf from mesh

    Parameters
    ----------
    mesh : trimesh.base.Trimesh or open3d.open3d.geometry.TriangleMesh
        input mesh
    output_path : str
        Ouput file where output mesh saved
    init_texture : bool
        If true, make the mesh texture the same as the base one.
        This is necessary if you want to change the texture when rendering with
        https://github.com/kosuke55/hanging_points_cnn/blob/master/hanging_points_cnn/create_dataset/renderer.py

    Returns
    -------
    output_file : str
        output file path
    """
    if isinstance(mesh, o3d.geometry.TriangleMesh):
        mesh = open3d_to_trimesh(mesh)
    center = np.mean(np.asarray(mesh.vertices), axis=0)

    dirname, filename = os.path.split(file)

    # TODO: in the future I need to decide wether the center should be [0, 0, 0]
    base_urdf_path = 'models/base/base.urdf'
    tree = ET.parse(base_urdf_path)
    root = tree.getroot()
    center = ''.join(str(i) + ' ' for i in mesh.centroid.tolist()).strip()
    root[0].find('inertial').find('origin').attrib['xyz'] = center

    mesh.export(os.path.join(dirname, 'base.obj'), 'obj')

    p.vhacd(os.path.join(dirname, 'base.obj'), os.path.join(dirname, 'base.obj'), os.path.join(dirname, 'base.txt'))

    tree.write(os.path.join(dirname, 'base.urdf'), encoding='utf-8', xml_declaration=True)

def main(args):

    print('success in.')

    input_dir = args.input_dir
    # base_save_dir = args.save_dir
    # os.makedirs(base_save_dir, exist_ok=True)

    target_length = 0.06

    files = glob.glob(f'{input_dir}/*/*_normalized.obj')
    files.sort()
    keep_list = [
        # 'Hook_htl_40_easy',
        # 'Hook_htl_70_normal',
        # 'Hook_htl_97_hard',
        # 'Hook_htl_100_easy',
        # 'Hook_htl_105_normal',
        # 'Hook_htl_106_hard',
        # 'Hook_htl_133_easy',
        # 'Hook_htl_147_devil',
        # 'Hook_htl_152_normal',
        # 'Hook_htl_175_devil',
        # 'Hook_htl_175_hard',
        # 'Hook_htl_186_hard',
        # 'Hook_htl_194_devil',
        # 'Hook_htl_194_hard',
        # 'Hook_htl_209_hard',
        # 'Hook_htl_222_hard',
        # 'Hook_htl_232_hard',
        # 'Hook_htl_284_normal',
        # 'Hook_htl_337_hard',
        # 'Hook_htl_337_hard',
        'Hook_hky_120_devil',
    ]

    for file in tqdm(files):
        
        cont = True
        for ignore_item in keep_list:
            if ignore_item in file:
                print(f'ignore : {file}')
                cont = False
                break
        if cont:
            continue


        dirname, filename = os.path.split(file)
        filename_without_ext, ext = os.path.splitext(filename)

        out_path = f'{dirname}/base.urdf'
        # if os.path.exists(out_path):
        #     print(f'{out_path} exists, continue...')
        #     continue

        try:
            mesh = o3d.io.read_triangle_mesh(file)
            mesh = mesh.simplify_vertex_clustering(
                voxel_size=0.001,
                contraction=o3d.geometry.SimplificationContraction.Average)
            mesh = open3d_to_trimesh(mesh)
            mesh_invert = mesh.copy()
            mesh_invert.invert()
            mesh += mesh_invert
            mesh.merge_vertices()
            # size = np.array(np.max(mesh.vertices, axis=0)
            #                 - np.min(mesh.vertices, axis=0))
            # length = np.max(size)
            # mesh.vertices = mesh.vertices * target_length / length

        except Exception as e:
            print('skip {}'.format(file))
            continue
        
        if mesh.vertices.shape[0] > 1 and mesh.faces.shape[0] > 1:
            print(f'processing {file}')
            create_urdf(mesh, file)
            print(f'{out_path} saved')
        else:
            print('skip {}'.format(file))

if __name__=='__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument(
        '--input_dir',
        '-i',
        type=str,
        help='input directory',
        default='')
    # parser.add_argument(
    #     '--save-dir',
    #     '-s',
    #     type=str,
    #     help='save directory',
    #     default='')

    args = parser.parse_args()

    main(args)
