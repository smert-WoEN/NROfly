import sys

import cv2
import numpy as np
import requests

if __name__ == '__main__':
    def nothing(*arg):
        pass

cv2.namedWindow("result")  # создаем главное окно
cv2.namedWindow("settings")  # создаем окно настроек

url = "https://stepik.org/media/attachments/course/128568/color2.png"
resp = requests.get(url, stream=True).raw
image = np.asarray(bytearray(resp.read()), dtype=np.uint8)
image = cv2.imdecode(image, cv2.IMREAD_COLOR)

# создаем 6 бегунков для настройки начального и конечного цвета фильтра
cv2.createTrackbar('h1', 'settings', 0, 255, nothing)
cv2.createTrackbar('s1', 'settings', 0, 255, nothing)
cv2.createTrackbar('v1', 'settings', 0, 255, nothing)
cv2.createTrackbar('h2', 'settings', 255, 255, nothing)
cv2.createTrackbar('s2', 'settings', 255, 255, nothing)
cv2.createTrackbar('v2', 'settings', 255, 255, nothing)
crange = [0, 0, 0, 0, 0, 0]

while True:
    img = cv2.GaussianBlur(image, (3, 3), cv2.BORDER_DEFAULT)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # считываем значения бегунков
    h1 = cv2.getTrackbarPos('h1', 'settings')
    s1 = cv2.getTrackbarPos('s1', 'settings')
    v1 = cv2.getTrackbarPos('v1', 'settings')
    h2 = cv2.getTrackbarPos('h2', 'settings')
    s2 = cv2.getTrackbarPos('s2', 'settings')
    v2 = cv2.getTrackbarPos('v2', 'settings')

    # формируем начальный и конечный цвет фильтра
    h_min = np.array((h1, s1, v1), np.uint8)
    h_max = np.array((h2, s2, v2), np.uint8)
    hsvMin = np.array((0, 9, 0), np.uint8)
    hsvMax = np.array((50, 255, 255), np.uint8)
    # накладываем фильтр на кадр в модели HSV
    thresh = cv2.inRange(hsv, h_min, h_max)
    _, counter, _ = cv2.findContours(thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    print(len(counter))
    cv2.imshow('result', thresh)
    cv2.imshow('image', image)
    cv2.drawContours(img, counter, contourIdx=-1, color=(0, 255, 0), thickness=2,
                 lineType=cv2.LINE_AA)
    cv2.imshow('img', img)
    ch = cv2.waitKey(5)
    if ch == 27:
        break

cv2.destroyAllWindows()
