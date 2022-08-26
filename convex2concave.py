from unicodedata import name
import pybullet as p
import pybullet_data as pd
import os

def main():
    input_path = "models/hook/Hook_60/base.obj"
    output_path = "models/hook/Hook_60/base_vhacd.obj"
    log_path = "models/hook/Hook_60/log_vhacd.txt.obj"
    p.connect(p.DIRECT)
    p.vhacd(input_path, output_path, log_path)

if __name__=="__main__":
    main()