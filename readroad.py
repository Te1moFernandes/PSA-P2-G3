# importing some useful packages

import numpy as np
import cv2
import matplotlib.pyplot as plt


def canny(image):
# Obter imagem cinza-------------------#
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
# Diminuir "noise" e obter uma imagem mais suave--------------------#
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
# Descobrir a gama que é de interesse, ou seja, a mudança drástica de intensidade------------------#
    canny = cv2.Canny(blur, 50, 150)
    return canny

# Criação da mascara e da area que interessa para o caso----------------#
def region_of_interest(image):
    altura = image.shape[0]
    #largura = image.shape[1]
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
canny = canny(lane_image)
# Imagem cortada----------------------#
cropped_image = region_of_interest(canny)
# Show image------------------#
cv2.imshow("result", cropped_image)
cv2.waitKey(0)
