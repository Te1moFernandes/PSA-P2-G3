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

# def det_estrada(image):
#
#     def make_coordinates(image, line_parameters):
#         slope, intercept = line_parameters
#         y1 = image.shape[0]
#         y2 = int(y1 * (3 / 5))
#         x1 = int((y1 - intercept) / slope)
#         x2 = int((y2 - intercept) / slope)
#         return np.array([x1, y1, x2, y2])
#
#     # Agrupar as linhas de acordo com o seu declive, caso o declive seja positivo são linhas da esquerda, caso contrário da direita (de acordo com a imagem)------------------#
#     def average_slope_intercept(image, lines):
#         left_fit = []
#         right_fit = []
#         for line in lines:
#             x1, y1, x2, y2 = line.reshape(4)
#             parameters = np.polyfit((x1, x2), (y1, y2), 1)
#             slope = parameters[0]
#             intercept = parameters[1]
#             if slope < 0:
#                 left_fit.append((slope, intercept))
#             else:
#                 right_fit.append((slope, intercept))
#         left_fit_average = np.average(left_fit, axis=0)
#         right_fit_average = np.average(right_fit, axis=0)
#         left_line = make_coordinates(image, left_fit_average)
#         right_line = make_coordinates(image, right_fit_average)
#         return np.array([left_line, right_line])
#
#     def canny(image):
#         # Obter imagem cinza-------------------#
#         gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
#         # Diminuir "noise" e obter uma imagem mais suave--------------------#
#         blur = cv2.GaussianBlur(gray, (5, 5), 0)
#         # Descobrir a gama que é de interesse, ou seja, a mudança drástica de intensidade------------------#
#         canny = cv2.Canny(blur, 50, 150)
#         return canny
#
#     def display_lines(image, lines):
#
#         line_image = np.zeros_like(image)
#
#         if lines is not None:
#             for x1, y1, x2, y2 in lines:
#                 cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
#         return line_image
#
#     # Criação da mascara e da area que interessa para o caso----------------#
#     def region_of_interest(image):
#
#         print(image.shape)
#         altura = image.shape[0]
#         largura = image.shape[1]
#         P1 = (round(largura/5), round(altura-1))
#         P2 = (round(largura/2), round(1/3*altura))
#         P3 = (round(4*largura/5), round(altura-1))
#         print('P1=' + str(P1))
#         print('P2=' + str(P2))
#         print('P3=' + str(P3))
#         poligonos = np.array([
#             [P1, P2, P3]
#         ])
#         mask = np.zeros_like(image)
#         cv2.fillPoly(mask, poligonos, 255)
#         masked_image = cv2.bitwise_and(image, mask)
#         return masked_image
#
#     # ---------------------------------------VIDEO-------------------------------------#
#     # read image----------------------------#
#     # Cópia da imagem-------------------------#
#     lane_image = np.copy(image)
#     # Uso da função canny/Tratamento da imagem---------------#
#     canny_image = canny(lane_image)
#     # Imagem cortada----------------------#
#     cropped_image = region_of_interest(canny_image)
#     # interseção dos pontos para criar as linhas melhores candidatas para o que interessa----------------------#
#     lines = cv2.HoughLinesP(cropped_image, 2, np.pi / 180, 100, np.array([]), minLineLength=40, maxLineGap=100)
#     averaged_lines = average_slope_intercept(lane_image, lines)
#     line_image = display_lines(lane_image, averaged_lines)
#
#     # Juntar a imagem das linhas com a imagem principal (a cópia neste caso)-------------------------#
#     combo_image = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1)
#
#     # Show image------------------#
#     return combo_image


def imageCallback(msg):
    print("Received an image!")

    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except:
        print('Could not convert image')
        return

    # Save your OpenCV2 image as a jpeg
    #detecao_ativa = det_estrada(cv2_img)
    cv2.imshow('front_camera', cv2_img)
    cv2.waitKey(20)


    # How to process the image to define the best angle and speed?

    # make a driving decision
    pt1_sum_ri = (0, 0)
    pt2_sum_ri = (0, 0)
    pt1_avg_ri = (0, 0)
    count_posi_num_ri = 0

    pt1_sum_le = (0, 0)
    pt2_sum_le = (0, 0)
    pt1_avg_le = (0, 0)

    count_posi_num_le = 0


    RGB_Camera_im = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
    #################################################
    # Now image resolution is 720x1280x3
    size_im = cv2.resize(RGB_Camera_im, dsize=(640, 480))  # VGA resolution
    # size_im = cv2.resize(test_im, dsize=(800, 600))  # SVGA resolution
    # size_im = cv2.resize(test_im, dsize=(1028, 720))  # HD resolution
    # size_im = cv2.resize(test_im, dsize=(1920, 1080))  # Full-HD resolution
    # cv2.imshow("size_im", size_im)
    #################################################

    #################################################
    # ROI Coordinates Set-up
    # roi = size_im[320:480, 213:426]  # [380:430, 330:670]   [y:y+b, x:x+a]
    # roi_im = cv2.resize(roi, (213, 160))  # x,y
    # cv2.imshow("roi_im", roi_im)
    roi = size_im[240:480, 108:532]  # [380:430, 330:670]   [y:y+b, x:x+a]
    roi_im = cv2.resize(roi, (424, 240))  # (a of x, b of y)
    # cv2.imshow("roi_im", roi_im)
    #################################################

    #################################################
    # Gaussian Blur Filter
    Blur_im = cv2.bilateralFilter(roi_im, d=-1, sigmaColor=5, sigmaSpace=5)
    #################################################

    #################################################
    # Canny edge detector
    edges = cv2.Canny(Blur_im, 50, 100)
    # cv2.imshow("edges", edges)
    #################################################

    #################################################
    # Hough Transformation
    # lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180.0, threshold=80, minLineLength=30, maxLineGap=50)
    # rho, theta는 1씩 변경하면서 검출하겠다는 의미, np.pi/180 라디안 = 1'
    # threshold 숫자가 작으면 정밀도↓ 직선검출↑, 크면 정밀도↑ 직선검출↓
    # min_line_len 선분의 최소길이
    # max_line,gap 선분 사이의 최대 거리
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180.0, threshold=25, minLineLength=10, maxLineGap=20)


    #N = lines.shape[0]

    if lines is None: #in case HoughLinesP fails to return a set of lines
            #make sure that this is the right shape [[ ]] and ***not*** []
            lines = [[0,0,0,0]]
    else:

        #for line in range(N):
        for line in lines:

            x1, y1, x2, y2 = line[0]

            #x1 = lines[line][0][0]
            #y1 = lines[line][0][1]
            #x2 = lines[line][0][2]
            #y2 = lines[line][0][3]

            if x2 == x1:
                a = 1
            else:
                a = x2 - x1

            b = y2 - y1

            radi = b / a  # 라디안 계산
            # print('radi=', radi)

            theta_atan = math.atan(radi) * 180.0 / math.pi
            # print('theta_atan=', theta_atan)

            pt1_ri = (x1 + 108, y1 + 240)
            pt2_ri = (x2 + 108, y2 + 240)
            pt1_le = (x1 + 108, y1 + 240)
            pt2_le = (x2 + 108, y2 + 240)

            if theta_atan > 20.0 and theta_atan < 90.0:
                # cv2.line(size_im, (x1+108, y1+240), (x2+108, y2+240), (0, 255, 0), 2)
                # print('live_atan=', theta_atan)

                count_posi_num_ri += 1

                pt1_sum_ri = sumMatrix(pt1_ri, pt1_sum_ri)
                # pt1_sum = pt1 + pt1_sum
                # print('pt1_sum=', pt1_sum)

                pt2_sum_ri = sumMatrix(pt2_ri, pt2_sum_ri)
                # pt2_sum = pt2 + pt2_sum
                # print('pt2_sum=', pt2_sum)

            if theta_atan < -20.0 and theta_atan > -90.0:
                # cv2.line(size_im, (x1+108, y1+240), (x2+108, y2+240), (0, 0, 255), 2)
                # print('live_atan=', theta_atan)

                count_posi_num_le += 1

                pt1_sum_le = sumMatrix(pt1_le, pt1_sum_le)
                # pt1_sum = pt1 + pt1_sum
                # print('pt1_sum=', pt1_sum)

                pt2_sum_le = sumMatrix(pt2_le, pt2_sum_le)
                # pt2_sum = pt2 + pt2_sum
                # print('pt2_sum=', pt2_sum)

        # print('pt1_sum=', pt1_sum_ri)
        # print('pt2_sum=', pt2_sum_ri)
        # print('count_posi_num_ri=', count_posi_num_ri)
        # print('count_posi_num_le=', count_posi_num_le)

        # testartu = pt1_sum / np.array(count_posi_num)
        # print(tuple(testartu))

        pt1_avg_ri = pt1_sum_ri // np.array(count_posi_num_ri)
        pt2_avg_ri = pt2_sum_ri // np.array(count_posi_num_ri)
        pt1_avg_le = pt1_sum_le // np.array(count_posi_num_le)
        pt2_avg_le = pt2_sum_le // np.array(count_posi_num_le)

        # print('pt1_avg_ri=', pt1_avg_ri)
        # print('pt2_avg_ri=', pt2_avg_ri)
        # print('pt1_avg_le=', pt1_avg_le)
        # print('pt2_avg_le=', pt2_avg_le)

        # print('pt1_avg=', pt1_avg_ri)
        # print('pt2_avg=', pt2_avg_ri)
        # print('np_count_posi_num=', np.array(count_posi_num))

        # line1_ri = tuple(pt1_avg_ri)
        # line2_ri = tuple(pt2_avg_ri)
        # line1_le = tuple(pt1_avg_le)
        # line2_le = tuple(pt2_avg_le)
        # print('line1=', line1_ri)
        # print('int2=', int2)

        #################################################
        # 차석인식의 흔들림 보정
        # right-----------------------------------------------------------
        x1_avg_ri, y1_avg_ri = pt1_avg_ri
        # print('x1_avg_ri=', x1_avg_ri)
        # print('y1_avg_ri=', y1_avg_ri)
        x2_avg_ri, y2_avg_ri = pt2_avg_ri
        # print('x2_avg_ri=', x2_avg_ri)
        # print('y2_avg_ri=', y2_avg_ri)

        a_avg_ri = ((y2_avg_ri - y1_avg_ri) / (x2_avg_ri - x1_avg_ri))
        b_avg_ri = (y2_avg_ri - (a_avg_ri * x2_avg_ri))
        # print('a_avg_ri=', a_avg_ri)
        # print('b_avg_ri=', b_avg_ri)

        pt2_y2_fi_ri = 480

        # pt2_x2_fi_ri = ((pt2_y2_fi_ri - b_avg_ri) // a_avg_ri)

        if a_avg_ri > 0:
            pt2_x2_fi_ri = int((pt2_y2_fi_ri - b_avg_ri) // a_avg_ri)
        else:
            pt2_x2_fi_ri = 0

        # print('pt2_x2_fi_ri=', pt2_x2_fi_ri)
        pt2_fi_ri = (pt2_x2_fi_ri, pt2_y2_fi_ri)
        # pt2_fi_ri = (int(pt2_x2_fi_ri), pt2_y2_fi_ri)
        # print('pt2_fi_ri=', pt2_fi_ri)

        # left------------------------------------------------------------
        x1_avg_le, y1_avg_le = pt1_avg_le
        x2_avg_le, y2_avg_le = pt2_avg_le
        # print('x1_avg_le=', x1_avg_le)
        # print('y1_avg_le=', y1_avg_le)
        # print('x2_avg_le=', x2_avg_le)
        # print('y2_avg_le=', y2_avg_le)

        a_avg_le = ((y2_avg_le - y1_avg_le) / (x2_avg_le - x1_avg_le))
        b_avg_le = (y2_avg_le - (a_avg_le * x2_avg_le))
        # print('a_avg_le=', a_avg_le)
        # print('b_avg_le=', b_avg_le)

        pt1_y1_fi_le = 480
        if a_avg_le < 0:
            pt1_x1_fi_le = int((pt1_y1_fi_le - b_avg_le) // a_avg_le)
        else:
            pt1_x1_fi_le = 0
        # pt1_x1_fi_le = ((pt1_y1_fi_le - b_avg_le) // a_avg_le)
        # print('pt1_x1_fi_le=', pt1_x1_fi_le)

        pt1_fi_le = (pt1_x1_fi_le, pt1_y1_fi_le)
        # print('pt1_fi_le=', pt1_fi_le)

        # print('pt1_avg_ri=', pt1_sum_ri)
        # print('pt2_fi_ri=', pt2_fi_ri)
        # print('pt1_fi_le=', pt1_fi_le)
        # print('pt2_avg_le=', pt2_sum_le)
        #################################################

        #################################################
        # lane painting
        # right-----------------------------------------------------------
        # cv2.line(size_im, tuple(pt1_avg_ri), tuple(pt2_avg_ri), (0, 255, 0), 2) # right lane
        cv2.line(size_im, tuple(pt1_avg_ri), tuple(pt2_fi_ri), (0, 255, 0), 2)  # right lane
        # left-----------------------------------------------------------
        # cv2.line(size_im, tuple(pt1_avg_le), tuple(pt2_avg_le), (0, 255, 0), 2) # left lane
        cv2.line(size_im, tuple(pt1_fi_le), tuple(pt2_avg_le), (0, 255, 0), 2)  # left lane
        # center-----------------------------------------------------------
        cv2.line(size_im, (320, 480), (320, 360), (0, 228, 255), 1)  # middle lane
        #################################################

        #################################################
        # possible lane
        # FCP = np.array([pt1_avg_ri, pt2_avg_ri, pt1_avg_le, pt2_avg_le])
        # cv2.fillConvexPoly(size_im, FCP, color=(255, 242, 213)) # BGR
        #################################################
        FCP_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8) + 0
        # FCP = np.array([pt1_avg_ri, pt2_avg_ri, pt1_avg_le, pt2_avg_le])
        # FCP = np.array([(100,100), (100,200), (200,200), (200,100)])
        FCP = np.array([pt2_avg_le, pt1_fi_le, pt2_fi_ri, pt1_avg_ri])
        cv2.fillConvexPoly(FCP_img, FCP, color=(255, 242, 213))  # BGR
        alpha = 0.9
        size_im = cv2.addWeighted(size_im, alpha, FCP_img, 1 - alpha, 0)

        # alpha = 0.4
        # size_im = cv2.addWeighted(size_im, alpha, FCP, 1 - alpha, 0)
        #################################################

        #################################################
        # lane center 및 steering 계산 (320, 360)
        lane_center_y_ri = 360
        if a_avg_ri > 0:
            lane_center_x_ri = int((lane_center_y_ri - b_avg_ri) // a_avg_ri)
        else:
            lane_center_x_ri = 0

        lane_center_y_le = 360
        if a_avg_le < 0:
            lane_center_x_le = int((lane_center_y_le - b_avg_le) // a_avg_le)
        else:
            lane_center_x_le = 0

        # caenter left lane (255, 90, 185)
        cv2.line(size_im, (lane_center_x_le, lane_center_y_le - 10), (lane_center_x_le, lane_center_y_le + 10),
                 (0, 228, 255), 1)
        # caenter right lane
        cv2.line(size_im, (lane_center_x_ri, lane_center_y_ri - 10), (lane_center_x_ri, lane_center_y_ri + 10),
                 (0, 228, 255), 1)
        # caenter middle lane
        lane_center_x = ((lane_center_x_ri - lane_center_x_le) // 2) + lane_center_x_le
        cv2.line(size_im, (lane_center_x, lane_center_y_ri - 10), (lane_center_x, lane_center_y_le + 10),
                 (0, 228, 255), 1)

    #turn left
    if 0 < lane_center_x <= 318:
        speed = 0.3
        angle = 0.15


    #move on and floor it
    elif 318 < lane_center_x < 322:
        speed = 0.5
        angle = 0

    #turn right
    elif lane_center_x >= 322:
        speed = 0.3
        angle = -0.15

    #additional info needed, STOP
    else:
        speed = 0
        angle = 0

    #build a twist msg to publish
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
