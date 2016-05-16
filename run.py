# coding: utf-8

import subprocess
import os


while True:
    graspit_simulator = os.path.expanduser("~/graspit/build/graspit_simulator") 
    cmd = graspit_simulator + " -p libgraspGenerationPlugin  -c mug.iv"
    subprocess.call(cmd.split())
