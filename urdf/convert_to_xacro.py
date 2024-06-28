#! /usr/bin/env python

import sys
import os

urdf_file = sys.argv[1]
xacro_template = sys.argv[2]
xacro_file = sys.argv[3]

urdf_text = open(urdf_file, 'r').read()
template_text = open(xacro_template, 'r').read()

urdf_text = urdf_text.replace('<robot name="q_gripper">', '')
urdf_text = urdf_text.replace('</robot>', '')

xacro_text = template_text.replace('URDF_CONTENT', urdf_text)
prefix_words = ['base', 'finger']
for w in prefix_words:
    xacro_text = xacro_text.replace(f'="{w}"', f'="${{prefix}}_{w}"')

with open(xacro_file, 'w') as f:
    f.write(xacro_text)