#! /usr/bin/python3

# rospy for the subscriber
import time
from copy import deepcopy

import rospy

import numpy as np

# OpenCV2 for saving an image
import cv2
from vertical_stacking import verticalStacking
from geometry_msgs.msg import Twist

# ROS Image message
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

# Global variables
# Instantiate CvBridge
bridge = CvBridge()
publisher = None


def imageCallback(msg):
    print("Received an image!")

    try:
        # Convert your ROS Image message to OpenCV2
        image_rgb = bridge.imgmsg_to_cv2(msg, "bgr8")
    except:
        print('Could not convert image')
        return

    # Preprocessing
    image_gui = deepcopy(image_rgb)
    image_gray = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2GRAY)  # convert to grayscale
    hsv = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2HSV) #converter imagens azuis
    lower_blue = np.array([80, 50, 50], np.uint8)  # example value
    upper_blue = np.array([130, 255, 255], np.uint8)
    lower_red = np.array([0, 50, 50], np.uint8)
    upper_red = np.array([10, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    kernal = np.ones((5, 5), "uint8")
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(image_gui, image_gui,
                              mask=red_mask)
    blue_mask = cv2.dilate(blue_mask, kernal)
    res_blue = cv2.bitwise_and(image_gui, image_gui,
                               mask=blue_mask)


    #Cor vermelha
    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            image_gui = cv2.rectangle(image_gui, (x, y),
                                       (x + w, y + h),
                                       (0, 0, 255), 2)

            cv2.putText(image_gui, "Sinal de Perigo/Proibicao", (x, y+50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 255))


    #Cor azul
    contours, hierarchy = cv2.findContours(blue_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            image_gui = cv2.rectangle(image_gui, (x, y),
                                       (x + w, y + h),
                                       (255, 0, 0), 2)

            cv2.putText(image_gui, "Sinal de Obrigacao/Informacao", (x, y+50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 0, 0))

    _, image_thresh = cv2.threshold(image_gray, 127, 255, cv2.THRESH_BINARY)  # thresholding

    # Save your OpenCV2 image as a jpeg

    # How to process the image to define the best angle and speed?

    reference_y = 260
    reference_y_delta = 20
    height, width = image_thresh.shape
    middle_x = int(width* 3 / 4)
    minimum_number_white_pixels = 5

    image_stacked = verticalStacking(image=image_thresh,
                                     y_limits=[reference_y-reference_y_delta, reference_y + reference_y_delta])

    # search for white pixels in stacked image from lert to right
    white_xs = []
    for x in range(0, width):
        if image_stacked[x] > minimum_number_white_pixels:
            white_xs.append(x)

    groups = []
    # iterate through white_xs and create groups
    first = True
    group_idx = 0
    for x in white_xs:
        if first:
            group = {'idx': group_idx, 'xs': [x]}
            groups.append(group)
            group_idx += 1
            first = False
            continue

        # decide if a new group should be create
        last_x = groups[-1]['xs'][-1]
        if abs(x - last_x) > 1:  # create new group
            group = {'idx': group_idx, 'xs': [x]}
            groups.append(group)
            group_idx += 1
        else:
            groups[-1]['xs'].append(x)

    # Compute the average x for each group
    for group in groups:
        group['xavg'] = sum(group['xs']) / len(group['xs'])

    # Compute the distance between the average and the middle x
    for group in groups:
        group['dist_to_middle'] = abs(middle_x - group['xavg'])

    #print(groups)

    # select middle line as the group which is closed to the middle of the image
    smallest_distance = 9999
    for group in groups:
        if group['dist_to_middle'] < smallest_distance:
            smallest_distance = group['dist_to_middle']
            right_line = group

    #print(middle_line)

    # for group in groups:
    #     group['dist_to_middle_line'] = middle_line['xavg'] - group['xavg']
    #
    # smallest_distance = 9999
    # for group in groups:
    #     if group['dist_to_middle_line'] >= 0:
    #         continue
    #     elif group['dist_to_middle_line'] < smallest_distance:
    #         smallest_distance = group['dist_to_middle_line']
    #         right_line = group
    #
    # smallest_distance = 9999
    # for group in groups:
    #     if group['dist_to_middle_line'] <= 0:
    #         continue
    #     elif group['dist_to_middle_line'] < smallest_distance:
    #         smallest_distance = group['dist_to_middle_line']
    #         left_line = group



    #print(right_line)



    # -------------------------------------------
    # Drawing
    # -------------------------------------------
    for x in white_xs:
        cv2.line(image_gui, (x, reference_y), (x, reference_y), (0, 0, 255), 4)

    if not right_line == None:
        point = (int(right_line['xavg']), reference_y)
        color = (0, 255, 255)
        cv2.line(image_gui, point, point, color, 4)
        cv2.putText(image_gui, 'Linha Guia', point, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1, cv2.LINE_AA)

    #if not right_line == None:
        #point = (int(right_line['xavg']), reference_y)
        #cv2.line(image_gui, point, point, color, 4)
        #cv2.putText(image_gui, 'RL', point, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1, cv2.LINE_AA)

    #if not left_line == None:
        #point = (int(left_line['xavg']), reference_y)
        #cv2.line(image_gui, point, point, color, 4)
        #cv2.putText(image_gui, 'LL', point, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1, cv2.LINE_AA)

    # make a driving decision
    #print(right_line['dist_to_middle'])
    kp = 0.005
    #ks = 0.005

    angle = kp * (middle_x - right_line['xavg'])

    #speed = ks * (1/abs(middle_x - right_line['xavg']))
    speed = 0.7







    print(len(groups))
    print(height)
    print(width)
    print(round(height))

    if len(groups)>5:
        cv2.putText(image_gui, "ALERTA PASSADEIRA", (150, 150) , cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 5)
    # for group in groups:
    #     element_size=(len(group['xs']))
    #     if element_size>200.:
    #         print('ALERTA PASSADEIRA')




    # cv2.imshow('image_rgb', image_rgb)
    # cv2.imshow('image_gray', image_gray)
    # cv2.imshow('image_thresh', image_thresh)
    cv2.imshow('image_gui', image_gui)
    #cv2.imshow('mask', mask)
    #cv2.imshow('Sinais de Obrigação/Informação', res_blue)
    #cv2.imshow('Sinais de Perigo/Proibição', res_red)
    cv2.waitKey(20)

    # build a twist msg to publish
    twist = Twist()
    twist.linear.x = speed
    twist.angular.z = angle
    global publisher
    publisher.publish(twist)


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