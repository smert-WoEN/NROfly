import cv2
import numpy as np
import requests

if __name__ == '__main__':
    def nothing(*arg):
        pass

hsvMinRed = np.array((0, 248, 196), np.uint8)
hsvMaxRed = np.array((255, 255, 249), np.uint8)
hsvMinBlue = np.array((0, 255, 0), np.uint8)
hsvMaxBlue = np.array((140, 255, 255), np.uint8)
hsvMinYellow = np.array((20, 10, 0), np.uint8)
hsvMaxYellow = np.array((45, 255, 255), np.uint8)
hsvMinPurple = np.array((145, 10, 0), np.uint8)
hsvMaxPurple = np.array((150, 255, 255), np.uint8)
hsvMinGreen = np.array((50, 10, 0), np.uint8)
hsvMaxGreen = np.array((80, 255, 255), np.uint8)

url = "https://stepik.org/media/attachments/course/128568/color2.png"
resp = requests.get(url, stream=True).raw
image = np.asarray(bytearray(resp.read()), dtype=np.uint8)
image = cv2.imdecode(image, cv2.IMREAD_COLOR)

img = cv2.GaussianBlur(image, (3, 3), cv2.BORDER_DEFAULT)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

hsvFilterRed = cv2.inRange(hsv, hsvMinRed, hsvMaxRed)
_, contourRed, _ = cv2.findContours(hsvFilterRed, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
# redContourDraw = img.copy()
# cv2.drawContours(image=redContourDraw, contours=contourRed, contourIdx=-1, color=(0, 255, 0), thickness=2,
#                  lineType=cv2.LINE_AA)

hsvBlueFilter = cv2.inRange(hsv, hsvMinBlue, hsvMaxBlue)
_, contourBlue, _ = cv2.findContours(hsvBlueFilter, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
#
# blueContourDraw = img.copy()
# cv2.drawContours(image=blueContourDraw, contours=contourBlue, contourIdx=-1, color=(0, 255, 0), thickness=2,
#                  lineType=cv2.LINE_AA)

hsvYellowFilter = cv2.inRange(hsv, hsvMinYellow, hsvMaxYellow)
_, contourYellow, _ = cv2.findContours(hsvYellowFilter, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
hsvPurpleFilter = cv2.inRange(hsv, hsvMinPurple, hsvMaxPurple)
_, contourPurple, _ = cv2.findContours(hsvPurpleFilter, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
hsvGreenFilter = cv2.inRange(hsv, hsvMinGreen, hsvMaxGreen)
_, contourGreen, _ = cv2.findContours(hsvGreenFilter, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
print("red:", len(contourRed))
print("yellow:", len(contourYellow))
print("green:", len(contourGreen))
print("blue:", len(contourBlue))
print("purple:", len(contourPurple))
# while True:
#     cv2.imshow("HSV", hsv)
#     #cv2.imshow("dst", dst)
#     cv2.imshow("img", img)
#     cv2.imshow("hsvRedFilter", hsvFilterRed)
#     cv2.imshow("hsvBlueFilter", hsvBlueFilter)
#     cv2.imshow("redContourDraw", redContourDraw)
#     cv2.imshow("blueContourDraw", blueContourDraw)
#     ch = cv2.waitKey(5)
#     if ch == 27:
#         break