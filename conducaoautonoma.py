#!/usr/bin/python3

# importing some useful packages
import glob
import os
import sys
import numpy as np
import cv2
import matplotlib.pyplot as plt
import random
import time
import argparse

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

#--------------------------------------CAMERA DO CARLA-----------------------------------#

def process_image(image):
    image = np.array(image.raw_data)
    img = image.reshape((704,1279,4))
    img = img[:,:,:3]


    cv2.imshow('img', )
    cv2.waitKey(1)



def main():
    actorList = []

    client = carla.Client('localhost',2000)
    client.set_timeout(10.0)
    world = client.load_world('Town07')
    #print(client.get_available_map())

    blueprintLibrary = world.get_blueprint_library()
    vehicle_bp = blueprintLibrary.filter('model3')[0]
    transform = carla.Transform(carla.Location(x=50,y=60,z=2),carla.Rotation(yaw=180))
    vehicle = world.spawn_actor(vehicle_bp, transform)
    vehicle.apply_control(carla.VehicleControl(throttle=0.15, brake=0)) #andar para a frente
    actorList.append(vehicle)
    print('Ta spawnado')
    print(actorList)
#dormir
    camera_bp = blueprintLibrary.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '1279')
    camera_bp.set_attribute('image_size_y', '704')
    camera_bp.set_attribute('fov', '90')
    #camera_bp.set_attribute('sensor_tick', '1.0')
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    camera.listen(lambda image: process_image(image))
    time.sleep(60)

    while True:

        pt1_sum_ri = (0, 0) # pontos da direita ( somatório 1, 2, average e contagem)
        pt2_sum_ri = (0, 0)
        pt1_avg_ri = (0, 0)
        count_posi_num_ri = 0

        pt1_sum_le = (0, 0) # pontos da esquerda (1, 2, average e contagem)
        pt2_sum_le = (0, 0)
        pt1_avg_le = (0, 0)
        count_posi_num_le = 0

        
        global Camera_image
        RGB_Camera_im = cv2.cvtColor(Camera_image, cv2.COLOR_BGR2GRAY)
        ############################################
        # Resolução da imagem
        size_im = cv2.resize(RGB_Camera_im, dsize=(640, 480))
        ########################


        ########################
        roi = size_im[240:480, 108:532]
        roi_im = cv2.resize(roi, (424, 240))
        cv2.imshow("roi_im", roi_im)

if __name__ == '__main__':
    main()
