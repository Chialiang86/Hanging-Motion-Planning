import os, glob
import argparse
import copy
import torch
import pytorch3d
import numpy as np
from skimage import measure
from PIL import Image
from tqdm.notebook import tqdm
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
import scipy.spatial.distance as dist
from pytorch3d.io import load_obj, save_obj
from pytorch3d.structures import Meshes
# from pytorch3d.utils import ico_sphere
from pytorch3d.ops import sample_points_from_meshes
from pytorch3d.ops.marching_cubes import marching_cubes_naive
from pytorch3d.loss import (
    chamfer_distance, 
    mesh_edge_loss, 
    mesh_laplacian_smoothing, 
    mesh_normal_consistency,
)

def my_chamfer_distance(pts_src : torch.Tensor, pts_tgt : torch.Tensor):

    assert pts_src.shape[2] == pts_tgt.shape[2] and pts_src.shape[2] == 3

    cdist = torch.cdist(pts_src, pts_tgt) # B x M x N
    first_term = cdist.min(dim=2)[0].mean() # B x M
    second_term = cdist.min(dim=1)[0].mean() # B x N
    loss_chamfer = (0.3 * first_term + 0.7 * second_term) / 4
    return loss_chamfer

def plot_pointcloud(mesh, title="", save=False):
    # Sample points uniformly from the surface of the mesh.
    points = sample_points_from_meshes(mesh, 5000)
    x, y, z = points.clone().detach().cpu().squeeze().unbind(1)    
    fig = plt.figure(figsize=(5, 5))
    ax = Axes3D(fig)
    ax.scatter3D(x, z, -y)
    ax.set_xlabel('x')
    ax.set_ylabel('z')
    ax.set_zlabel('y')
    ax.set_title(title)
    ax.view_init(190, 30)
    if save:
        plt.savefig(f'{title}.png')
    plt.show()


def plot_pcds(points, title="", save=False):
    # Sample points uniformly from the surface of the mesh.
    x, y, z = points.clone().detach().cpu().squeeze().unbind(1)    
    fig = plt.figure(figsize=(5, 5))
    ax = Axes3D(fig)
    ax.scatter3D(x, z, -y)
    ax.set_xlabel('x')
    ax.set_ylabel('z')
    ax.set_zlabel('y')
    ax.set_title(title)
    ax.view_init(190, 30)
    if save:
        plt.savefig(f'{title}.png')
    plt.show()


def deform_mesh_random(tgt_mesh : Meshes, device=torch.device("cuda:0"), max_scale=0.15):

    # We will learn to deform the source mesh by offsetting its vertices
    # The shape of the deform parameters is equal to the total number of vertices in tgt_mesh
    deform_verts = torch.full(tgt_mesh.verts_packed().shape, 0.0, device=device, requires_grad=False)
    deform_verts = (2 * torch.rand(tgt_mesh.verts_packed().shape) - 1) * max_scale 
    deform_verts= deform_verts.to(device)

    # Deform the mesh (original mesh will be the same)
    new_tgt_mesh = tgt_mesh.offset_verts(deform_verts)

    return new_tgt_mesh

def deform_mesh_to_tgt(src_mesh : Meshes, tgt_mesh : Meshes, device=torch.device("cuda:0")):

    # We initialize the source shape to be a sphere of radius 1
    print(f'there are {src_mesh.verts_list()[0].shape} vertices \
            and {src_mesh.faces_list()[0].shape} faces')

    # The optimizer
    deform_verts = torch.full(src_mesh.verts_packed().shape, 0.0, device=device, requires_grad=True)
    optimizer = torch.optim.SGD([deform_verts], lr=1.0, momentum=0.9)

    # Number of optimization steps
    Niter = 200
    # Weight for the chamfer loss
    w_chamfer = 10.0 
    # Weight for mesh edge loss
    w_edge = 1.0 
    # Weight for mesh normal consistency
    w_normal = 0.01 
    # Weight for mesh laplacian smoothing
    w_laplacian = 0.1 
    # Plot period for the losses
    loop = tqdm(range(Niter))

    chamfer_losses = []
    laplacian_losses = []
    edge_losses = []
    normal_losses = []

    new_src_mesh = None
    for i in loop:
        # Initialize optimizer
        optimizer.zero_grad()
        
        # Deform the mesh
        new_src_mesh = src_mesh.offset_verts(deform_verts)
        
        # We sample 5k points from the surface of each mesh 
        sample_tgt = sample_points_from_meshes(tgt_mesh, 5000)
        sample_src = sample_points_from_meshes(new_src_mesh, 5000)
        
        # We compare the two sets of pointclouds by computing (a) the chamfer loss
        loss_chamfer, _ = chamfer_distance(sample_tgt, sample_src)
        # and (b) the edge length of the predicted mesh
        loss_edge = mesh_edge_loss(new_src_mesh)
        # mesh normal consistency
        loss_normal = mesh_normal_consistency(new_src_mesh)
        # mesh laplacian smoothing
        loss_laplacian = mesh_laplacian_smoothing(new_src_mesh, method="uniform")
        # Weighted sum of the losses
        loss = loss_chamfer * w_chamfer + loss_edge * w_edge + loss_normal * w_normal + loss_laplacian * w_laplacian
        # Print the losses
        loop.set_description('total_loss = %.6f' % loss)
        
        # Save the losses for plotting
        chamfer_losses.append(float(loss_chamfer.detach().cpu()))
        edge_losses.append(float(loss_edge.detach().cpu()))
        normal_losses.append(float(loss_normal.detach().cpu()))
        laplacian_losses.append(float(loss_laplacian.detach().cpu()))
            
        # Optimization step
        loss.backward()
        optimizer.step()


    return new_src_mesh

def main(args):

    input_dir = args.input_dir
    output_dir = args.output_dir
    gen_num = args.gen_num
    
    assert os.path.exists(input_dir), f'{input_dir} not exists'
    assert os.path.exists(output_dir), f'{output_dir} not exists'

    # Set the device
    if torch.cuda.is_available():
        device = torch.device("cuda:0")
    else:
        device = torch.device("cpu")
        print("WARNING: CPU only, this will be slow!")

    # list mesh files
    obj_files = glob.glob(f'{input_dir}/*/concave.obj')
    print(obj_files)

    for obj_file in obj_files:

        print(f'processing {obj_file} ...')
        
        # We read the target 3D model using load_obj
        verts_template, faces_template, aux = load_obj(obj_file)
        
        obj_id = obj_file.split('/')[-2]
        gen_cnt = 0
        while gen_cnt < gen_num:

            print(f'generating {obj_file}-{gen_cnt} ...')
            verts, faces = copy.deepcopy(verts_template), copy.deepcopy(faces_template)

            # verts is a FloatTensor of shape (V, 3) where V is the number of vertices in the mesh
            # faces is an object which contains the following LongTensors: verts_idx, normals_idx and textures_idx
            # For this tutorial, normals and textures are ignored.
            faces_idx = faces.verts_idx.to(device)
            verts = verts.to(device)

            # We scale normalize and center the target mesh to fit in a sphere of radius 1 centered at (0,0,0). 
            # (scale, center) will be used to bring the predicted mesh to its original center and scale
            # Note that normalizing the target mesh, speeds up the optimization but is not necessary!
            center = verts.mean(0)
            verts = verts - center
            scale = max(verts.abs().max(0)[0])
            verts = verts / scale

            # We construct a Meshes structure for the target mesh
            tgt_mesh = Meshes(verts=[verts], faces=[faces_idx])
            deformed_tgt_mesh = deform_mesh_random(tgt_mesh, device=device)
            deformed_tgt_mesh = deform_mesh_to_tgt(deformed_tgt_mesh, tgt_mesh, device=device)

            # # run marching cube
            # volume_resolution = 32
            # voxel_size = 1 / volume_resolution
            # points = sample_points_from_meshes(deformed_tgt_mesh, 5000).squeeze()
            # points_min, points_max = torch.min(points, 0).values, torch.max(points, 0).values
            # scale = torch.max(points_max - points_min)
            # points = ((points - points_min) / scale)
            # points_occupancy = (points // voxel_size).type(torch.long)
            # volumes = torch.zeros([volume_resolution + 1, volume_resolution + 1, volume_resolution + 1], dtype=torch.float32, device=device)
            # volumes[points_occupancy] = 1.0
            # print('marching ...')
            # verts, faces = marching_cubes_naive(volumes.unsqueeze(0))
            # print('marched ...')

            # # save target mesh
            tgt_obj_path = f'{output_dir}/{obj_id}/concave-{gen_cnt}.obj'
            verts = deformed_tgt_mesh.verts_packed()
            verts = (verts * scale) + center
            faces=deformed_tgt_mesh.faces_packed()
            
            # save_obj(tgt_obj_path, verts=deformed_tgt_mesh.verts_packed(), faces=deformed_tgt_mesh.faces_packed())
            save_obj(tgt_obj_path, verts=verts, faces=faces)

            gen_cnt += 1
        

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-dir', '-id', type=str, default='models/hook')
    parser.add_argument('--gen-num', '-gn', type=int, default=1)
    parser.add_argument('--output-dir', '-od', type=str, default='models/hook')
    args = parser.parse_args()
    main(args)
