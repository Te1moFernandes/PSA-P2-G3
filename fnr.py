#! /usr/bin/python3

# rospy for the subscriber
import numpy as np
import rospy

# OpenCV2 for saving an image
import cv2
from geometry_msgs.msg import Twist

# ROS Image message
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

# Global variables
# Instantiate CvBridge
bridge = CvBridge()
publisher = None

def det_estrada(image):

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

        print(image.shape)
        altura = image.shape[0]
        largura = image.shape[1]
        P1 = (round(largura/5), round(altura-1))
        P2 = (round(largura/2), round(1/3*altura))
        P3 = (round(4*largura/5), round(altura-1))
        print('P1=' + str(P1))
        print('P2=' + str(P2))
        print('P3=' + str(P3))
        poligonos = np.array([
            [P1, P2, P3]
        ])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, poligonos, 255)
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    # ---------------------------------------VIDEO-------------------------------------#
    # read image----------------------------#
    # Cópia da imagem-------------------------#
    lane_image = np.copy(image)
    # Uso da função canny/Tratamento da imagem---------------#
    canny_image = canny(lane_image)
    # Imagem cortada----------------------#
    cropped_image = region_of_interest(canny_image)
    # interseção dos pontos para criar as linhas melhores candidatas para o que interessa----------------------#
    lines = cv2.HoughLinesP(cropped_image, 2, np.pi / 180, 100, np.array([]), minLineLength=40, maxLineGap=100)
    averaged_lines = average_slope_intercept(lane_image, lines)
    line_image = display_lines(lane_image, averaged_lines)

    # Juntar a imagem das linhas com a imagem principal (a cópia neste caso)-------------------------#
    combo_image = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1)

    # Show image------------------#
    return combo_image


def imageCallback(msg):
    print("Received an image!")

    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except:
        print('Could not convert image')
        return

    # Save your OpenCV2 image as a jpeg
    detecao_ativa = det_estrada(cv2_img)
    cv2.imshow('front_camera', detecao_ativa)
    cv2.waitKey(20)


    # How to process the image to define the best angle and speed?

    # make a driving decision
    #NÂO SEI COMO METER A LINHA DO MEIO DE ACORDO COM ESTE CODIGO, o slope teria de ser infinito e o b infinito! (x= tal)

    #
    # #turn left
    # if 0 < lane_center_x <= 318:
    #     speed = 0.3
    #     angle = 0.15
    #
    #
    # #move on and floor it
    # elif 318 < lane_center_x < 322:
    #     speed = 0.5
    #     angle = 0
    #
    # #turn right
    # elif lane_center_x >= 322:
    #     speed = 0.3
    #     angle = -0.15
    #
    # #additional info needed, STOP
    # else:
    #     speed = 0
    #     angle = 0

    # build a twist msg to publish
    # twist = Twist()
    # twist.linear.x = speed
    # twist.angular.z = angle
    # global publisher
    # publisher.publish(twist)
    #

def main():
    rospy.init_node('driver')
    # Define your image topic
    image_topic = "/front_camera/rgb/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, imageCallback)
    global publisher
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # Spin until ctrl + c
    rospy.spin()


if __name__ == '__main__':
    main()
