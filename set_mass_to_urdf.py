#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os
import glob
import shutil
import xml.etree.ElementTree as ET

directory = 'models/inference_hooks'

urdfs = glob.glob(f'{directory}/*/*.urdf')

for urdf in urdfs:

    print(urdf)
    tree = ET.parse(urdf)
    root = tree.getroot()
    root[0].find('inertial').find('mass').attrib['value'] = "0.0"

    tree.write(urdf, encoding='utf-8', xml_declaration=True)