# coding: utf-8

import subprocess



while True:
    cmd = "/home/timchunght/graspit/build/graspit_simulator -p libgraspGenerationPlugin  -c mug.iv"
    subprocess.call(cmd.split())
