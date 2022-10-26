import numpy as np
import glob
import json
import xml.etree.ElementTree as ET

def get_center_scale_from_urdf(urdf_path):

    tree = ET.parse(urdf_path)
    root = tree.getroot()
    center = np.array(
      [
        float(i) for i in root[0].find(
          "inertial"
        ).find(
          "origin"
        ).attrib['xyz'].split(' ')
      ]
    )
    scale = np.array(
      [
        float(i) for i in root[0].find(
          "visual"
        ).find(
          "geometry"
        ).find(
          "mesh"
        ).attrib["scale"].split(" ")
      ]
    )
    assert scale[0] == scale[1] and scale[0] == scale[2] and scale[1] == scale[2], f"scale is not uniformed : {scale}"
    return center, scale[0]

def main():
    urdf_files = glob.glob(f'*/base.urdf')
    urdf_files.sort()

    f_out = open('geo_info.json', 'w')
    json_dict = {
        'geo_info': []
    }
    
    for urdf_file in urdf_files:
        # print(f'processing {urdf_file} ...')
        center, scale = get_center_scale_from_urdf(urdf_file)
        json_element = {
            'path': f'models/geo_data/hanging_exp/{urdf_file}',
            'center': center.tolist(),
            'scale': scale
        }
        print(f'{urdf_file[:-10]} {scale}')
        json_dict['geo_info'].append(json_element)
    
    json.dump(json_dict, f_out, indent=4)
    print('geo_info.json saved')

if __name__=="__main__":
    main()