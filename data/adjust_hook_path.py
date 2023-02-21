import os, glob, json
from tqdm import tqdm

def main():
    folder = 'data_all_new_4'
    fix_term = 'hook_all_new_4'
    json_files = glob.glob(f'{folder}/*/*.json')
    json_files.sort()
    for json_file in tqdm(json_files):
        f = open(json_file, 'r')
        json_dict = json.load(f)
        f.close()

        if 'hook_path' in json_dict.keys():
            path_split_by_hook = json_dict['hook_path'].split('/')[1]
            if path_split_by_hook != fix_term:
                
                original_path = json_dict['hook_path']
                true_path = ''
                file_tokens = original_path.split('/')
                for i in range(len(file_tokens) - 1):
                    if i != 1:
                        true_path += f'{file_tokens[i]}/'
                    else :
                        true_path += f'{fix_term}/'
                true_path += file_tokens[-1]

                json_dict['hook_path'] = true_path
                print(f'{original_path} => {true_path}')

                f = open(json_file, 'w')
                json_dict = json.dump(json_dict, f, indent=4)
                f.close()

if __name__=="__main__":
    main()