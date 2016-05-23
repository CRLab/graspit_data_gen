# coding: utf-8

import subprocess
import os


count = 0
while True:
    if count == 100000:
        assert False
    graspit_simulator = os.path.expanduser("~/graspit/build/graspit_simulator") 
    cmd = graspit_simulator + " -p libgraspGenerationPlugin  -c mug.iv"
    subprocess.call(cmd.split())
    count += 1
