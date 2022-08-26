from pathlib import Path
from hanging_points_generator.generator_utils \
    import check_contact_points

texture = True
urdf_dir = '/media/kosuke55/SANDISK/meshdata/ycb_hanging_object/urdf2'

paths = list(sorted(Path('./') .glob('*.json')))
skip=True
image_dir = '/home/kosuke55/icra2021/pic/big_axis/groud_truth'
for path in paths:

    # if path.stem == '048_hammer':
    if path.stem == '033_spatula':
        skip = False
    if skip:
        continue

    pose = str(path)
    if texture:
        urdf = str(Path(urdf_dir) / path.stem / 'textured.urdf')
    else:
        urdf = str(path.with_suffix('.urdf'))
    image_name = str(image_dir / path.with_suffix('.png'))
    print(image_name)
    try:
        check_contact_points(
            pose,
            urdf,
            cluster_min_points=1,
            use_filter_penetration=False,
            inf_penetration_check=False,
            align=False,
            average=False,
            average_pos=False,
            image_name=image_name)
    except KeyboardInterrupt:
        pass
