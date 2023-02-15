import os, glob, json

json_paths = glob.glob('*/*.json')

for json_path in json_paths:

    json_dict = json.load(open(json_path))
    hook_path = json_dict["hook_path"]

    if 'patch' in hook_path:
        print(hook_path)
    hook_path_split = hook_path.split('/')
    modified_hook_path = f'{hook_path_split[0]}/hook_all/{hook_path_split[2]}/{hook_path_split[3]}'
    print(f"modified_hook_path {modified_hook_path}")
    json_dict["hook_path"] = modified_hook_path

    json.dump(json_dict, open(json_path, 'w'), indent=4)