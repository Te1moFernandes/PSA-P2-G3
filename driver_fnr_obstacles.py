#! /usr/bin/python3

# rospy for the subscriber
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
    _, image_thresh = cv2.threshold(image_gray, 127, 255, cv2.THRESH_BINARY)  # thresholding

    ### DETECT OBSTACLES

    ## convert to hsv
    hsv = cv2.cvtColor(image_gui, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (36, 25, 25), (70, 255,255))

    ## slice the green

    imask = mask > 0
    green = np.zeros_like(image_gui, np.uint8)
    green[imask] = image_gui[imask]
    cv2.imshow('mask', mask)


    # Avoid obstacles


    # Save your OpenCV2 image as a jpeg

    # How to process the image to define the best angle and speed?

    reference_y = 260
    reference_y_2 = 100
    reference_y_delta = 20
    reference_y_delta_2 = 50
    height, width = image_thresh.shape
    height2, width2 = mask.shape
    middle_x = int(width* 3 / 4)
    minimum_number_white_pixels = 5
    minimum_number_green_pixels = 100

    image_stacked = verticalStacking(image=image_thresh,
                                     y_limits=[reference_y-reference_y_delta, reference_y + reference_y_delta])

    image_stacked_2 = verticalStacking(mask, y_limits=[reference_y_2-reference_y_delta_2, reference_y_2+reference_y_delta_2])

    # search for white pixels in stacked image from left to right
    white_xs = []
    for x in range(0, width):
        if image_stacked[x] > minimum_number_white_pixels:
            white_xs.append(x)

    # search for green pixels in stacked image from left to right
    green_xs = []
    for x in range(0, width2):
        if image_stacked_2[x] > minimum_number_green_pixels:
            green_xs.append(x)

    #print(green_xs)

    groups_whites = []
    # iterate through white_xs and create groups_whites
    first = True
    group_idx = 0
    for x in white_xs:
        if first:
            group = {'idx': group_idx, 'xs': [x]}
            groups_whites.append(group)
            group_idx += 1
            first = False
            continue

        # decide if a new group should be create
        last_x = groups_whites[-1]['xs'][-1]
        if abs(x - last_x) > 1:  # create new group
            group = {'idx': group_idx, 'xs': [x]}
            groups_whites.append(group)
            group_idx += 1
        else:
            groups_whites[-1]['xs'].append(x)


    # Compute the average x for each group
    for group in groups_whites:
        group['xavg'] = sum(group['xs']) / len(group['xs'])

    # Compute the distance between the average and the middle x
    for group in groups_whites:
        group['dist_to_middle'] = abs(middle_x - group['xavg'])


    # select middle line as the group which is closed to the middle of the image
    smallest_distance = 9999
    for group in groups_whites:
        if group['dist_to_middle'] < smallest_distance:
            smallest_distance = group['dist_to_middle']
            right_line = group

    groups_greens = []
    # iterate through greens_xs and create groups_greens
    first2 = True
    group_idx2 = 0
    for x in green_xs:
        if first2:
            group = {'idx': group_idx, 'xs': [x]}
            groups_greens.append(group)
            group_idx2 += 1
            first2 = False
            continue

        # decide if a new group should be create
        last_x = groups_greens[-1]['xs'][-1]
        if abs(x - last_x) > 1:  # create new group
            group = {'idx': group_idx2, 'xs': [x]}
            groups_greens.append(group)
            group_idx2 += 1
        else:
            groups_greens[-1]['xs'].append(x)

    for group in groups_greens:
        group['xavg'] = sum(group['xs']) / len(group['xs'])

    # for group in groups_whites:
    #     group['dist_to_middle_line'] = middle_line['xavg'] - group['xavg']
    #
    # smallest_distance = 9999
    # for group in groups_whites:
    #     if group['dist_to_middle_line'] >= 0:
    #         continue
    #     elif group['dist_to_middle_line'] < smallest_distance:
    #         smallest_distance = group['dist_to_middle_line']
    #         right_line = group
    #
    # smallest_distance = 9999
    # for group in groups_whites:
    #     if group['dist_to_middle_line'] <= 0:
    #         continue
    #     elif group['dist_to_middle_line'] < smallest_distance:
    #         smallest_distance = group['dist_to_middle_line']
    #         left_line = group





    # -------------------------------------------
    # Drawing
    # -------------------------------------------
    for x in white_xs:
        cv2.line(image_gui, (x, reference_y), (x, reference_y), (0, 0, 255), 4)

    for x in green_xs:
        cv2.line(image_gui, (x,reference_y_2), (x, reference_y_2), (255,0,0), 4)

    if not green_xs == None:
        point = (int(green_xs['xavg']), reference_y_2)
        color = (255,255,0)
        cv2.line(image_gui, point, point, color, 4)
        cv2.putText(image_gui, 'MidPoint', point, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1, cv2.LINE_AA)

    if not right_line == None:
        point = (int(right_line['xavg']), reference_y)
        color = (0, 255, 255)
        cv2.line(image_gui, point, point, color, 4)
        cv2.putText(image_gui, 'RL', point, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1, cv2.LINE_AA)

    # if not right_line == None:
    #     point = (int(right_line['xavg']), reference_y)
    #     cv2.line(image_gui, point, point, color, 4)
    #     cv2.putText(image_gui, 'RL', point, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1, cv2.LINE_AA)
    #
    # if not left_line == None:
    #     point = (int(left_line['xavg']), reference_y)
    #     cv2.line(image_gui, point, point, color, 4)
    #     cv2.putText(image_gui, 'LL', point, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1, cv2.LINE_AA)

    # make a driving decision
    #print(right_line['dist_to_middle'])

    kp = 0.005

    angle = kp * (middle_x - right_line['xavg'])

    speed = 0.7


    # cv2.imshow('image_rgb', image_rgb)
    # cv2.imshow('image_gray', image_gray)
    # cv2.imshow('image_thresh', image_thresh)
    cv2.imshow('image_gui', image_gui)
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
