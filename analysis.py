import os
import glob
import argparse
import time
import numpy as np

def extract_lines(lines : list, data : dict, keys : list=['object', 'hook', 'success', 'time', 'iteration']):

    for key in keys:
        if key not in data.keys():
            data[key] = []
    for line in lines:
        line_data = line.split(',')
        for i in range(len(keys)):
            try:
                data[keys[i]].append(float(line_data[i]))
            except ValueError:
                data[keys[i]].append(line_data[i])

def write_analysis(data : dict, pivots : dict, columns : list, difficulty : str):
    data_column = 'target,difficulty,'
    for column in columns:
        data_column += (column + ',')

    ret = []

    data_pivots = {}
    for key in pivots.keys():
        data_pivots[key] = {}
        for item in pivots[key]:
            cond = np.where(data[key] == item)

            output_lines = []
            output_lines.append(data_column + '\n')

            data_line = f'{item},{difficulty},'
            for column in columns:
                print(column, item, len(data[column][cond]), np.mean(data[column][cond]))
                data_line += (str(np.mean(data[column][cond])) + ',')
            output_lines.append(data_line + '\n')
            ret.extend(output_lines)
    
    return ret

def main(args):

    root = 'data'
    if args.hce :
        easy_files   = glob.glob(f'{root}/*/*hce_easy.csv')
        medium_files = glob.glob(f'{root}/*/*hce_medium.csv')
        hard_files   = glob.glob(f'{root}/*/*hce_hard.csv')
    else :
        easy_files   = glob.glob(f'{root}/*/*easy.csv')
        medium_files = glob.glob(f'{root}/*/*medium.csv')
        hard_files   = glob.glob(f'{root}/*/*hard.csv')

    easy_files.sort()
    medium_files.sort()
    hard_files.sort()

    easy_datas = {}
    medium_datas = {}
    hard_datas = {}

    keys = ['object', 'hook', 'success', 'time', 'iteration', 'nodes']

    for easy_file in easy_files:
        f_easy_file = open(easy_file, 'r')
        lines = f_easy_file.readlines()
        extract_lines(lines, data=easy_datas, keys=keys)
    for medium_file in medium_files:
        f_medium_file = open(medium_file, 'r')
        lines = f_medium_file.readlines()
        extract_lines(lines, data=medium_datas, keys=keys)
    for hard_file in hard_files:
        f_hard_file = open(hard_file, 'r')
        lines = f_hard_file.readlines()
        extract_lines(lines, data=hard_datas, keys=keys)

    for key in easy_datas.keys():
        easy_datas[key] = np.asarray(easy_datas[key])
    for key in medium_datas.keys():
        medium_datas[key] = np.asarray(medium_datas[key])
    for key in hard_datas.keys():
        hard_datas[key] = np.asarray(hard_datas[key])
    
    # object	hook	success     time	iterations	nodes	length_before	length_after
    objects = ['bag', 'daily', 'mug', 'scissor', 'wrench']
    hook = ['Hook_bar', 'Hook_60', 'Hook_skew', 'Hook_90', 'Hook_180']
    if args.hce :
        pivots = {
            'object': ['hanging_exp_bag_5_hce', 'hanging_exp_daily_5_hce', 'hanging_exp_mug_59_hce', 'hanging_exp_scissor_4_hce', 'hanging_exp_wrench_1_hce'],
            'hook': ['Hook_bar', 'Hook_60', 'Hook_skew', 'Hook_90', 'Hook_180']
        }
    else :
        pivots = {
            'object': ['hanging_exp_bag_5', 'hanging_exp_daily_5', 'hanging_exp_mug_59', 'hanging_exp_scissor_4', 'hanging_exp_wrench_1'],
            'hook': ['Hook_bar', 'Hook_60', 'Hook_skew', 'Hook_90', 'Hook_180']
        }

    keys = ['success', 'time', 'iteration', 'nodes']
    
    prefix = 'hce' if args.hce else 'complete'
    out_path = f'analysis/{prefix}_{time.time()}.csv'
    out_data = []
    easy_out_data = write_analysis(easy_datas, pivots, columns=keys, difficulty='easy')
    medium_out_data = write_analysis(medium_datas, pivots, columns=keys, difficulty='medium')
    hard_out_data = write_analysis(hard_datas, pivots, columns=keys, difficulty='hard')
    out_data.extend(easy_out_data)
    out_data.extend(medium_out_data)
    out_data.extend(hard_out_data)
    with open(out_path, 'w') as f:
        f.writelines(out_data)
    print('process completed')

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--hce', action='store_true')
    args = parser.parse_args()
    main(args)