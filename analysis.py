import enum
import os
import copy
import glob
import argparse
import time
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

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
                # print(column, item, len(data[column][cond]), np.mean(data[column][cond]))
                data_line += (str(np.mean(data[column][cond])) + ',')
            output_lines.append(data_line + '\n')
            ret.extend(output_lines)
    
    return ret

def draw_heatmap(data : dict, objects : list, hooks : list):
    time_heat_map = np.zeros((len(hooks), len(objects)))
    success_heat_map = np.zeros((len(hooks), len(objects)))
    for i, hook in enumerate(hooks):
        for j, object in enumerate(objects):
            cond = np.where((data['object'] == object) & (data['hook'] == hook))
            time_mean = np.mean(data['time'][cond])
            success_mean = np.mean(data['success'][cond])
            time_heat_map[i, j] = time_mean
            success_heat_map[i, j] = success_mean
    
    return time_heat_map, success_heat_map

def plot_time(data_time, title='', label='easy', xlabel='time (sec)', ylabel='trials'):
    upper = np.ceil(np.max(data_time)).astype(int)
    bins = range(0, upper, 20)
    plt.hist(data_time, bins=bins, label=label)
    plt.legend(loc='upper right')
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.show()

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

    all_datas = {'object':[], 'hook':[], 'success':[], 'time':[], 'iteration':[], 'nodes':[]}
    easy_datas = {}
    medium_datas = {}
    hard_datas = {}

    keys = ['object', 'hook', 'success', 'time', 'iteration', 'nodes']

    prefix = 'hce' if args.hce else 'complete'
    time_stamp = time.time()

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
        all_datas[key].extend(easy_datas[key])
    for key in medium_datas.keys():
        medium_datas[key] = np.asarray(medium_datas[key])
        all_datas[key].extend(medium_datas[key])
    for key in hard_datas.keys():
        hard_datas[key] = np.asarray(hard_datas[key])
        all_datas[key].extend(hard_datas[key])
    for key in keys:
        all_datas[key] = np.asarray(hard_datas[key])
    

    # object	hook	success     time	iterations	nodes	length_before	length_after
    hooks = ['Hook_bar', 'Hook_60', 'Hook_skew', 'Hook_90', 'Hook_180']
    objects_short = ['bag', 'lock', 'mug', 'scissor', 'wrench']
    if args.hce :
        objects = ['hanging_exp_bag_5_hce', 'hanging_exp_daily_5_hce', 'hanging_exp_mug_59_hce', 'hanging_exp_scissor_4_hce', 'hanging_exp_wrench_1_hce']
        pivots = {
            'object': objects,
            'hook': hooks
        }
    else :
        objects = ['hanging_exp_bag_5', 'hanging_exp_daily_5', 'hanging_exp_mug_59', 'hanging_exp_scissor_4', 'hanging_exp_wrench_1']
        pivots = {
            'object': ['hanging_exp_bag_5', 'hanging_exp_daily_5', 'hanging_exp_mug_59', 'hanging_exp_scissor_4', 'hanging_exp_wrench_1'],
            'hook': ['Hook_bar', 'Hook_60', 'Hook_skew', 'Hook_90', 'Hook_180']
        }

    keys = ['success', 'time', 'iteration', 'nodes']
    out_path = f'analysis/{prefix}.csv'
    out_data = []
    easy_out_data = write_analysis(easy_datas, pivots, columns=keys, difficulty='easy')
    medium_out_data = write_analysis(medium_datas, pivots, columns=keys, difficulty='medium')
    hard_out_data = write_analysis(hard_datas, pivots, columns=keys, difficulty='hard')
    out_data.extend(easy_out_data)
    out_data.extend(medium_out_data)
    out_data.extend(hard_out_data)
    with open(out_path, 'w') as f:
        f.writelines(out_data)

    # time distribution
    out_path = f'analysis/{prefix}_distribution.png'
    bins = range(0, 1750, 25)
    plt.hist([easy_datas['time'], medium_datas['time'], hard_datas['time']], bins=bins, label=['easy', 'medium', 'hard'])
    plt.legend(loc='upper right')
    plt.title('Planning Time Distribution')
    plt.xlabel('time (sec)')
    plt.ylabel('count')
    plt.savefig(out_path)

    # heatmap
    out_path = f'analysis/{prefix}_heatmap.png'
    time_heat_map, success_heat_map = draw_heatmap(all_datas, objects=objects, hooks=hooks)
    fig, (ax1, ax2) = plt.subplots(ncols=2, figsize=(12, 5))
    fig.subplots_adjust(wspace=0.2)
    sns.heatmap(time_heat_map, xticklabels=objects_short, ax=ax1, yticklabels=hooks, fmt='.3f', annot=True)
    sns.heatmap(success_heat_map, xticklabels=objects_short, ax=ax2, yticklabels=hooks, fmt='.3f', annot=True)
    ax1.set_title('Planning Time (sec)')
    ax1.set_xticklabels(objects_short, rotation=0)
    ax1.set_yticklabels(hooks, rotation=45)
    ax2.set_title('Planning Success Rate')
    ax2.set_xticklabels(objects_short, rotation=0)
    ax2.set_yticklabels(hooks, rotation=45)
    plt.savefig(out_path)
    # plt.show()
    print('process completed')
    

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--hce', action='store_true')
    args = parser.parse_args()
    main(args)