import os
import numpy as np
import torch

from tqdm import tqdm
from point_e.diffusion.configs import DIFFUSION_CONFIGS, diffusion_from_config
from point_e.diffusion.sampler import PointCloudSampler
from point_e.models.download import load_checkpoint
from point_e.models.configs import MODEL_CONFIGS, model_from_config
from point_e.util.plotting import plot_point_cloud

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

print('creating base model...')
base_name = 'base40M-textvec'
base_model = model_from_config(MODEL_CONFIGS[base_name], device)
base_model.eval()
base_diffusion = diffusion_from_config(DIFFUSION_CONFIGS[base_name])

print('creating upsample model...')
upsampler_model = model_from_config(MODEL_CONFIGS['upsample'], device)
upsampler_model.eval()
upsampler_diffusion = diffusion_from_config(DIFFUSION_CONFIGS['upsample'])

print('downloading base checkpoint...')
base_model.load_state_dict(load_checkpoint(base_name, device))

print('downloading upsampler checkpoint...')
upsampler_model.load_state_dict(load_checkpoint('upsample', device))

sampler = PointCloudSampler(
    device=device,
    models=[base_model, upsampler_model],
    diffusions=[base_diffusion, upsampler_diffusion],
    num_points=[1024, 4096 - 1024],
    aux_channels=['R', 'G', 'B'],
    guidance_scale=[3.0, 0.0],
    model_kwargs_key_filter=('texts', ''), # Do not condition the upsampler at all
)

# Set a prompt to condition on.
# Set a prompt to condition on.
prompts_list = [
            [
                'a hook for hanging wrench', 
                'a hook for hanging wrench', 
                'a hook for hanging wrench', 
                'a hook for hanging wrench'
            ],
            [
                'a hook for hanging cooking utensils', 
                'a hook for hanging cooking utensils', 
                'a hook for hanging cooking utensils', 
                'a hook for hanging cooking utensils' 
            ],
            [
                'a hook for hanging keys', 
                'a hook for hanging keys', 
                'a hook for hanging keys', 
                'a hook for hanging keys' 
            ],
            [
                'a hook for hanging scissors', 
                'a hook for hanging scissors', 
                'a hook for hanging scissors', 
                'a hook for hanging scissors'
            ]
        ]

# Produce a sample from the model.
for prompts in prompts_list:
    print(f'now is time for {prompts[0]}')
    output_dir = f'pointe_out/{prompts[0]}'
    os.makedirs(output_dir, exist_ok=True)
    batch_size = len(prompts)
    counter = 0
    for i in tqdm(range(100)):
        samples = None
        for x in tqdm(sampler.sample_batch_progressive(batch_size=batch_size, model_kwargs=dict(texts=prompts))):
            samples = x

        pcds = sampler.output_to_point_clouds(samples)
        for i, pcd in enumerate(pcds):
            pcds[i].save(f'{output_dir}/{prompts[i]}-{counter}.npz')
            counter += 1
