import os, glob, torch
import matplotlib.pyplot as plt
from tqdm import tqdm

from point_e.models.download import load_checkpoint
from point_e.models.configs import MODEL_CONFIGS, model_from_config
from point_e.util.pc_to_mesh import marching_cubes_mesh
from point_e.util.plotting import plot_point_cloud
from point_e.util.point_cloud import PointCloud

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

print('creating SDF model...')
name = 'sdf'
model = model_from_config(MODEL_CONFIGS[name], device)
model.eval()

print('loading SDF model...')
model.load_state_dict(load_checkpoint(name, device))

# Load a point cloud we want to convert into a mesh.
pc_paths = glob.glob(f'pointe_out/*/*.npz')
pc_paths.sort()

for pc_path in tqdm(pc_paths):
    
    out_path = f'{pc_path[:-4]}.ply'
    if os.path.exists(out_path):
        continue

    pc = PointCloud.load(pc_path)

    # Plot the point cloud as a sanity check.
    fig = plot_point_cloud(pc, grid_size=2)

    # Produce a mesh (with vertex colors)
    mesh = marching_cubes_mesh(
        pc=pc,
        model=model,
        batch_size=4096,
        grid_size=32, # increase to 128 for resolution used in evals
        progress=True,
    )

    # Write the mesh to a PLY file to import into some other program.
    with open(out_path, 'wb') as f:
        mesh.write_ply(f)