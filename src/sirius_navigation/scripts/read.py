#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import yaml

file_path = "/home/sirius/sirius_ws/src/sirius_navigation/config/map.yaml"

with open(file_path, 'r', encoding='utf-8') as f:
    positions_list = yaml.safe_load(f)

print(positions_list['points'][0][0])