import glob
import os
import sys
import cv2
import random
import matplotlib.pyplot as plt
import time
import numpy as np
import argparse

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


def main():


    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.load_world(str(input("Insert prefered town...")))



if __name__ == '__main__':
    main()

