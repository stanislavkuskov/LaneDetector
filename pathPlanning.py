import cv2
import numpy as np

def calc_central_line(left_fitx, right_fitx, img_size, central_line_img, imshow=False):

    '''
    1. Вычисление центральной линии по точкам левой и правой линии
    2. Определение смещения от центра в каждой точке центральной линии

    '''
    # центральная линия
    centr_fitx = ((right_fitx - left_fitx) / 2) + left_fitx
    # Смещение центральной линии по точкам
    centr_offset = centr_fitx-(img_size[1]/2)



    if imshow == True:
        # Отрисовка центральной линии на изображении
        for i in range(len(centr_fitx)):
            cv2.circle(central_line_img, (int(centr_fitx[i]), i), 1, (255, 50, 255))
        cv2.imshow("res", central_line_img)

    # evklid_distance = np.linalg.norm(centr_fitx - prew_center_fitx)
    # print(evklid_distance)
    # cv2.waitKey(0)
    # prew_center_fitx = centr_fitx

    return centr_fitx, centr_offset


def calculate_offset(img_size, bottom_centr_fitx):
    """Вычисляем смещение шасси от центральной линии на изображении
    Смещение в пикселях
    """
    img_center = img_size[1]/2
    print(img_center,bottom_centr_fitx)

    vehicle_offset = (img_center - bottom_centr_fitx)
    print(vehicle_offset)
    # Fit new polynomials to x,y in world space


