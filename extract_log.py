import os
import argparse
import pandas as pd

def main(args):
    
    in_file = args.input
    out_files = [f'{args.output}_easy.csv', f'{args.output}_medium.csv', f'{args.output}_hard.csv']

    assert os.path.exists(in_file), f'{in_file} not exists'
    hook_name = os.path.splitext(in_file)[0].split('/')[-1].split('-')[0]
    obj_name = os.path.splitext(in_file)[0].split('/')[-1].split('-')[1]

    print(f'hook name : {hook_name}, obj name : {obj_name}')
    
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

        before_length = []
        after_length = []
        success = []
        iteration = []
        nodes = []
        time = []
        for l in lines:
            line = l.split('\n')[0]
            if before_token in line:
                before_length.append(int(line.split('|')[1].split(' ')[1]))
            if after_token in line:
                after_length.append(int(line.split('|')[1].split(' ')[1]))
            if success_token in line:
                success.append(int(line.split('|')[1]))
            if result_token in line:
                if int(line.split('|')[1].split(' ')[1]) < 4999:
                    iteration.append(int(line.split('|')[1].split(' ')[1]))
                    nodes.append(int(line.split('|')[2].split(' ')[1]))
                    time.append(float(line.split('|')[3].split(' ')[1]))
        
        if len(before_length) == 0:
            before_length = [0, 0, 0]
        if len(after_length) == 0:
            after_length = [0, 0, 0]
        if len(success) == 0:
            success = [0, 0, 0]
        if len(iteration) == 0:
            iteration = [0, 0, 0]
        if len(nodes) == 0:
            nodes = [0, 0, 0]
        if len(time) == 0:
            time = [0, 0, 0]

        # print(f'before_length {len(before_length)}')
        # print(f'after_length {len(after_length)}')
        # print(f'success {len(success)}')
        # print(f'iteration {len(iteration)}')
        # print(f'nodes {len(nodes)}')
        # print(f'time {len(time)}')

        # assert len(before_length) == 3
        # assert len(after_length) == 3
        # assert len(success) == 3
        # assert len(iteration) == 3
        # assert len(nodes) == 3
        # assert len(time) == 3

        # column_names = ['object', 'hook', 'success', 'time', 'iteration', 'nodes', 'length_before', 'length_after']
        for i in range(3):
            data_dict = {
                'object': [obj_name],
                'hook': [hook_name],
                'success': [success[i]],
                'time': [time[i]],
                'iteration': [iteration[i]],
                'nodes': [nodes[i]],
                'length_before': [before_length[i]],
                'length_after': [after_length[i]]
            }
            data_df = pd.DataFrame(data_dict)
            data_df.to_csv(out_files[i], mode='a', index=False, header=False)
        
        print('process completed')

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', '-i', type=str, default='')
    parser.add_argument('--output', '-o', type=str, default='')
    args = parser.parse_args()
    main(args)