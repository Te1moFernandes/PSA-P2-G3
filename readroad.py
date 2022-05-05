# importing some useful packages

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2



def main():

# read image
    image = cv2.imread("road.png")
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# print image
    print('This image is:', type(image), 'with dimensions:', image.shape)

# celulas da função shape, primeira altura, segunda largura, terceira, profundidade/canais de cor
    height = image.shape[0]
    width = image.shape[1]

    region_of_interest_vertices = [
        (0, height),
        (width/2, height/2),
        (width, height)
    ]

    def region_of_interest(img, vertices):
        mask = np.zeros_like(img)
        channel_count = img.shape[2]
        match_mask_color = (255,) * channel_count
        cv2.fillPoly(mask, vertices, match_mask_color)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    cropped_image = region_of_interest(image, np.array([region_of_interest_vertices, np.int32]))

# show the image
# if you wanted to show a single color channel image called 'gray', for example, call as plt.imshow(gray, cmap='gray')
    plt.imshow(cropped_image)
    plt.show()

if __name__ == "__main__":
    main()
