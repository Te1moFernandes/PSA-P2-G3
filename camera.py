import glob
import os
import sys
import cv2
import random
import matplotlib.pyplot as plt
import time
import numpy as np
import argparse
import readvideoroad

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

def process_image(image):
    image = np.array(image.raw_data)
    img = image.reshape((600,800,4))
    img = img[:,:,:3]

    cv2.imshow('img', img)
    cv2.waitKey(1)


def main():
    actorList = []

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.load_world('Town01')
    print(client.get_available_maps())
    blueprintLibrary = world.get_blueprint_library()
    vehicle_bp = blueprintLibrary.filter('cybertruck')[0]
    transform = carla.Transform(carla.Location(x=230, y=195, z=5), carla.Rotation(yaw=180))
    vehicle = world.spawn_actor(vehicle_bp, transform)
    actorList.append(vehicle)
    print('Ta spawnado')
    print(actorList)

    camera_bp = blueprintLibrary.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')
    #camera_bp.set_attribute('sensor_tick', '1.0')
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    camera.listen(lambda image: process_image(image))
    time.sleep(10)


if __name__ == '__main__':
    main()

