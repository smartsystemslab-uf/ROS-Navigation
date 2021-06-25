import os
import sys
import time
import shutil

directory = 'Camera1'
if os.path.isdir(directory):
    shutil.rmtree(directory)

os.mkdir(directory)
    