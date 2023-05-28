import os, glob, argparse

def main(args):

    model_dir = args.model_dir
    data_dir = args.data_dir
    kptraj_dir = args.kptraj_dir

    assert os.path.exists(model_dir), f'{model_dir} not exists'
    assert os.path.exists(data_dir), f'{data_dir} not exists'
    assert os.path.exists(kptraj_dir), f'{kptraj_dir} not exists'
    
    model_ids = os.listdir(model_dir)
    model_ids.sort()
    data_ids = [path.split('-')[0] for path in os.listdir(data_dir)]
    data_ids.sort()
    kptraj_ids = [path.split('.')[0] for path in os.listdir(kptraj_dir)]

    print('================================')
    for model_id in model_ids:
        if model_id not in data_ids:
            print(f'{model_id} not in data_ids')

    print('================================')    
    for data_id in data_ids:
        if data_id not in model_ids:
            print(f'{data_id} not in model_ids')

    # print('================================')    
    # for data_id in data_ids:
    #     if data_id not in kptraj_ids:
    #         print(f'{data_id} not in kptraj_ids')

    # print('================================')    
    # for kptraj_id in kptraj_ids:
    #     if kptraj_id not in data_ids:
    #         print(f'{kptraj_id} not in data_ids')

    # print('================================')    
    # for model_id in model_ids:
    #     if model_id not in kptraj_ids:
    #         print(f'{model_id} not in kptraj_ids')

    # print('================================')    
    # for kptraj_id in kptraj_ids:
    #     if kptraj_id not in model_ids:
    #         print(f'{kptraj_id} not in model_ids')
    return

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--model_dir', '-md', type=str, default='models/inference_hooks')
    parser.add_argument('--data_dir', '-dd', type=str, default='data/inference_hooks')
    parser.add_argument('--kptraj_dir', '-kd', type=str, default='keypoint_trajectory/kptraj_all_new')
    args = parser.parse_args()
    main(args)