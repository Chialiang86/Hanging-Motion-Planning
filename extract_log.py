import os
import argparse
import pandas as pd

def main(args):
    
    in_file = args.input
    out_file = args.output

    assert os.path.exists(in_file), f'{in_file} not exists'
    hook_name = os.path.splitext(in_file)[0].split('/')[-1].split('-')[0]
    obj_name = os.path.splitext(in_file)[0].split('/')[-1].split('-')[1]

    print(f'processing : {out_file} hook name : {hook_name}, obj name : {obj_name}')
    
    before_length = 0.0
    after_length = 0.0
    iteration = 0
    success = 0
    nodes = 0
    time = 0.0
    with open(in_file, 'r') as f:
        lines = []
        lines = f.readlines()
        # print('[Before Smoothness]|Length {}'.format(len(waypoints)))
        before_token = '[Before Smoothness]' 
        # print('[After Smoothness]|Length {}'.format(len(path)))
        after_token = '[After Smoothness]' 
        # print('[Results]|Iterations {}|Nodes {}|Time {}'.format(iteration, len(nodes1) + len(nodes2), elapsed_time(start_time)))
        result_token = '[Results]' 
        success_token = '[Success]' 
        for l in lines:
            line = l.split('\n')[0]
            if before_token in line:
                before_length = int(line.split('|')[1].split(' ')[1])
            if after_token in line:
                after_length = int(line.split('|')[1].split(' ')[1])
            if success_token in line:
                success = int(line.split('|')[1])
            if result_token in line:
                iteration = int(line.split('|')[1].split(' ')[1])
                nodes = int(line.split('|')[2].split(' ')[1])
                time = float(line.split('|')[3].split(' ')[1])

        # column_names = ['object', 'hook', 'success', 'time', 'iteration', 'nodes', 'length_before', 'length_after']
        data_dict = {
            'object': [obj_name],
            'hook': [hook_name],
            'success': [success],
            'time': [time],
            'iteration': [iteration],
            'nodes': [nodes],
            'length_before': [before_length],
            'length_after': [after_length]
        }
        data_df = pd.DataFrame(data_dict)
        data_df.to_csv(out_file, mode='a', index=False, header=False)
        
        print('process completed')

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', '-i', type=str, default='')
    parser.add_argument('--output', '-o', type=str, default='')
    args = parser.parse_args()
    main(args)