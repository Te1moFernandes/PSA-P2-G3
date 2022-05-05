# importing some useful packages

import numpy as np
import cv2
import matplotlib.pyplot as plt


def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = int(y1*(3/5))
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
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


# read image----------------------------#
image = cv2.imread("test_image.png")
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
cv2.imshow("result", combo_image)
cv2.waitKey(0)
