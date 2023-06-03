import os, glob, shutil

hook_names = [
    'Hook_hcu_104_devil', 'Hook_hcu_134_normal', 'Hook_hcu_138_hard', 'Hook_hcu_181_hard', 'Hook_hcu_190_easy', 
    'Hook_hcu_243_normal', 'Hook_hcu_279_devil', 'Hook_hcu_293_easy', 'Hook_hcu_296_normal', 'Hook_hcu_303_normal', 
    'Hook_hcu_306_normal', 'Hook_hcu_335_normal', 'Hook_hcu_359_easy', 'Hook_hcu_362_easy', 'Hook_hcu_364_normal', 
    'Hook_hcu_376_devil', 'Hook_hcu_380_hard', 'Hook_hcu_390_easy', 'Hook_hcu_3_hard', 'Hook_hcu_75_easy', 
    'Hook_hcu_89_devil', 'Hook_hs_105_hard', 'Hook_hs_117_hard', 'Hook_hs_154_hard', 'Hook_hs_156_hard', 
    'Hook_hs_190_easy', 'Hook_hs_216_easy', 'Hook_hs_229_normal', 'Hook_hs_275_devil', 'Hook_hs_293_normal', 
    'Hook_hs_314_easy', 'Hook_hs_317_easy', 'Hook_hs_339_devil', 'Hook_hs_363_devil', 'Hook_hs_370_easy', 
    'Hook_hs_393_devil', 'Hook_hs_42_hard', 'Hook_hs_70_easy', 'Hook_hs_94_hard', 'Hook_hs_95_devil', 
    'Hook_hsr_118_hard', 'Hook_hsr_123_normal', 'Hook_hsr_125_easy', 'Hook_hsr_13_devil', 'Hook_hsr_15_normal', 
    'Hook_hsr_218_devil', 'Hook_hsr_22_normal', 'Hook_hsr_263_hard', 'Hook_hsr_298_normal', 'Hook_hsr_304_hard', 
    'Hook_hsr_312_devil', 'Hook_hsr_321_devil', 'Hook_hsr_335_hard', 'Hook_hsr_371_devil', 'Hook_hsr_381_easy', 
    'Hook_hsr_391_hard', 'Hook_hsr_56_normal', 'Hook_hsr_5_normal', 'Hook_hsr_71_easy', 'Hook_omni_124_devil'
]

input_dir = 'data_all_new'
output_dir = 'data_all_new_testing'

input_paths = glob.glob(f'{input_dir}/*')

for input_path in input_paths:

    hook_name = input_path.split('/')[-1].split('-')[0]

    if hook_name in hook_names:
    
        output_path = '{}/{}'.format(output_dir, input_path.split('/')[-1])

        shutil.copytree(input_path, output_path)
