import glob, os
import json
from tqdm import tqdm

json_files = glob.glob("data/*/*.json")

for json_file in tqdm(json_files):
    print(json_file)
    f_json = open(json_file, "r")
    json_dict = json.load(f_json)
    f_json.close()

    new_json_dict = {
        "hook_path": json_dict["hook_path"],
        "obj_path": json_dict["obj_path"],
        "hook_pose": json_dict["hook_pose"],
        "contact_info": [
            {
                "contact_point_hook": [] \
                    if "contact_point_hook" not in json_dict["contact_info"][0].keys() \
                    else json_dict["contact_info"][0]["contact_point_hook"],
                "contact_point_obj": [] \
                    if "contact_point_obj" not in json_dict["contact_info"][0].keys() \
                    else json_dict["contact_info"][0]["contact_point_obj"],
                "obj_pose": json_dict["contact_info"][0]["obj_pose"],
            }
        ],
    }

    if "initial_pose" in json_dict.keys():
        new_json_dict["initial_pose"] = []
        for initial_pose_ele in json_dict["initial_pose"]:
            new_initial_pose_ele = {
                "robot_pose": initial_pose_ele["robot_pose"],
                "obj_pose": initial_pose_ele["obj_pose"],
            }
            new_json_dict["initial_pose"].append(new_initial_pose_ele)
    assert len(new_json_dict["initial_pose"]) == len(json_dict["initial_pose"])

    f_json = open(json_file, "w")
    json_dict = json.dump(new_json_dict, f_json, indent=4)
    f_json.close()

