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
    img = image.reshape((600,800,4))
    img = img[:,:,:3]

    cv2.imshow('img', img)
    cv2.waitKey(1)



def main():
    actorList = []
    try:
        client = carla.Client('localhost',2000)
        client.set_timeout(10.0)
        world = client.load_world('Town07')
        print(client.get_available_map())

        blueprintLibrary = world.get_blueprint_library()
        vehicle_bp = blueprintLibrary.filter('cybertruck')[0]
        transform = carla.Transform(carla.Location(x=130,y=195,z=40),carla.Rotation(yaw=180))
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

#------------------------------------------------- FIM DO CODIGO DA CAMARA DO CARLA----------------------------#

#------------------------------------------------- CODIGO DA DETECAO DE ESTRADA--------------------------------#
    def make_coordinates(image, line_parameters):
        slope, intercept = line_parameters
        y1 = image.shape[0]
        y2 = int(y1 * (3 / 5))
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        return np.array([x1, y1, x2, y2])

    # Agrupar as linhas de acordo com o seu declive, caso o declive seja positivo são linhas da esquerda, caso contrário da direita (de acordo com a imagem)------------------#
    def average_slope_intercept(image, lines):
        left_fit = []
        right_fit = []
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))
        left_fit_average = np.average(left_fit, axis=0)
        right_fit_average = np.average(right_fit, axis=0)
        left_line = make_coordinates(image, left_fit_average)
        right_line = make_coordinates(image, right_fit_average)
        return np.array([left_line, right_line])

    def canny(image):
        # Obter imagem cinza-------------------#
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        # Diminuir "noise" e obter uma imagem mais suave--------------------#
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        # Descobrir a gama que é de interesse, ou seja, a mudança drástica de intensidade------------------#
        canny = cv2.Canny(blur, 50, 150)
        return canny

    def display_lines(image, lines):
        line_image = np.zeros_like(image)
        if lines is not None:
            for x1, y1, x2, y2 in lines:
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
        return line_image

    # Criação da mascara e da area que interessa para o caso----------------#
    def region_of_interest(image):
        altura = image.shape[0]
        # largura = image.shape[1]
        poligonos = np.array([
            [(200, altura), (1100, altura), (550, 250)]
        ])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, poligonos, 255)
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    # ---------------------------------------VIDEO-------------------------------------#
    cap = cv2.VideoCapture("img")
    while (cap.isOpened()):
        ret, frame = cap.read()

        # ------------------------TRATAMENTO DE DADOS---------------------#
        # Uso da função canny/Tratamento da imagem---------------#
        canny_image = canny(frame)
        # Imagem cortada----------------------#
        cropped_image = region_of_interest(canny_image)
        # interseção dos pontos para criar as linhas melhores candidatas para o que interessa----------------------#
        lines = cv2.HoughLinesP(cropped_image, 2, np.pi / 180, 100, np.array([]), minLineLength=40, maxLineGap=100)
        averaged_lines = average_slope_intercept(frame, lines)
        line_image = display_lines(frame, averaged_lines)

        # Juntar a imagem das linhas com a imagem principal (a cópia neste caso)-------------------------#
        combo_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        # ------------------------FIM DO TRATAMENTO DE DADOS---------------------#

        # Show Video------------------#
        cv2.imshow("video", combo_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

#--------------------------------------------FIM DO CODIGO DE DETECAO DE ESTRADA----------------------------------------#

if __name__ == '__main__':
    main()
