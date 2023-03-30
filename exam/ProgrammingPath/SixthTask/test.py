import cv2
import numpy as np


for i in range(10):
    img = cv2.imread(str(i) + ".png", cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.resize(gray, (28, 28))
    print(gray2.tolist())

zero = [[255, 255, 255, 255, 255, 255, 255, 255, 254, 157, 11, 0, 0, 0, 0, 0, 0, 1, 121, 249, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 250, 72, 0, 0, 0, 6, 103, 216, 216, 150, 25, 0, 0, 0, 28, 237, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 176, 0, 0, 0, 0, 177, 255, 255, 255, 255, 255, 255, 255, 9, 0, 0, 0, 123, 255, 255, 255, 255, 255], [255, 255, 255, 255, 81, 0, 0, 0, 0, 204, 255, 255, 255, 255, 255, 255, 255, 255, 253, 7, 0, 0, 0, 47, 255, 255, 255, 255], [255, 255, 255, 27, 0, 0, 0, 0, 114, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 232, 0, 0, 0, 0, 5, 255, 255, 255], [255, 255, 128, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 38, 0, 0, 0, 0, 76, 255, 255], [255, 235, 0, 0, 0, 0, 0, 117, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 196, 0, 0, 0, 0, 0, 224, 255], [255, 26, 0, 0, 0, 0, 0, 210, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 251, 2, 0, 0, 0, 0, 2, 255], [246, 0, 0, 0, 0, 0, 0, 247, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 44, 0, 0, 0, 0, 0, 212], [97, 0, 0, 0, 0, 0, 23, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 77, 0, 0, 0, 0, 0, 74], [37, 0, 0, 0, 0, 0, 55, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 138, 0, 0, 0, 0, 0, 14], [3, 0, 0, 0, 0, 0, 96, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 169, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 122, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 188, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 149, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 188, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 175, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 174, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 164, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 161, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 141, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 123, 0, 0, 0, 0, 0, 0], [28, 0, 0, 0, 0, 0, 107, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 77, 0, 0, 0, 0, 0, 27], [83, 0, 0, 0, 0, 0, 57, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 53, 0, 0, 0, 0, 0, 72], [214, 0, 0, 0, 0, 0, 26, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 18, 0, 0, 0, 0, 0, 219], [255, 12, 0, 0, 0, 0, 0, 231, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 236, 0, 0, 0, 0, 0, 5, 255], [255, 224, 0, 0, 0, 0, 0, 163, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 175, 0, 0, 0, 0, 0, 223, 255], [255, 255, 103, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 27, 0, 0, 0, 0, 86, 255, 255], [255, 255, 255, 0, 0, 0, 0, 0, 169, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 239, 0, 0, 0, 0, 0, 255, 255, 255], [255, 255, 255, 255, 24, 0, 0, 0, 2, 239, 255, 255, 255, 255, 255, 255, 255, 255, 254, 9, 0, 0, 0, 22, 255, 255, 255, 255], [255, 255, 255, 255, 255, 57, 0, 0, 0, 6, 253, 255, 255, 255, 255, 255, 255, 249, 7, 0, 0, 0, 87, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 214, 8, 0, 0, 0, 36, 200, 255, 255, 158, 19, 0, 0, 0, 31, 241, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 230, 64, 0, 0, 0, 0, 0, 0, 0, 6, 148, 253, 255, 255, 255, 255, 255, 255, 255, 255]]
one = [[255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 253, 215, 134, 35, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 248, 195, 93, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [254, 237, 172, 34, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [8, 0, 31, 170, 255, 255, 255, 222, 37, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 157, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 161, 4, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 214, 19, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 235, 26, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 243, 28, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 29, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 29, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 29, 0, 0, 0, 0, 0, 0, 0, 0, 157, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 29, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 29, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 29, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 29, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 29, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 29, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 29, 0, 0, 0, 0, 0, 0, 0, 0, 157, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 29, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 29, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 29, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 29, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 29, 0, 0, 0, 0, 0, 0, 0, 0, 156, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 244, 29, 0, 0, 0, 0, 0, 0, 0, 0, 151, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 234, 25, 0, 0, 0, 0, 0, 0, 0, 0, 126, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 200, 15, 0, 0, 0, 0, 0, 0, 0, 0, 48, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 240, 110, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 187, 255, 255, 255, 255, 255, 255], [215, 25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
two = [[255, 255, 255, 255, 255, 255, 255, 215, 61, 0, 0, 0, 0, 0, 0, 0, 0, 2, 105, 240, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 165, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 198, 255, 255, 255, 255, 255, 255], [255, 255, 255, 249, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 251, 255, 255, 255, 255], [255, 255, 255, 0, 0, 0, 0, 0, 198, 255, 255, 255, 255, 255, 179, 1, 0, 0, 0, 0, 0, 0, 0, 0, 247, 255, 255, 255], [255, 255, 29, 0, 0, 97, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 242, 17, 0, 0, 0, 0, 0, 0, 3, 255, 255, 255], [255, 187, 0, 4, 230, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 87, 0, 0, 0, 0, 0, 0, 173, 255, 255], [255, 11, 0, 247, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 42, 0, 0, 0, 0, 0, 92, 255, 255], [255, 22, 161, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 189, 0, 0, 0, 0, 0, 78, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 128, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 250, 0, 0, 0, 0, 0, 249, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 194, 0, 0, 0, 0, 53, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 61, 0, 0, 0, 4, 250, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 240, 0, 0, 0, 0, 228, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 218, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 65, 0, 0, 0, 197, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 67, 0, 0, 3, 251, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 65, 0, 0, 12, 253, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 12, 0, 0, 105, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 234, 4, 0, 0, 225, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 138, 0, 0, 41, 251, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 37, 0, 0, 181, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 181, 1, 0, 26, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 249, 20, 0, 0, 149, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 243, 183], [255, 255, 255, 255, 255, 178, 0, 0, 21, 251, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 195, 0, 30], [255, 255, 255, 248, 6, 0, 0, 196, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 232, 54, 0, 0, 0, 227], [255, 255, 76, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 54, 255], [164, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 238, 255], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 101, 255, 255]]
three = [[255, 255, 255, 255, 255, 255, 255, 201, 76, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2, 85, 227, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 252, 101, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 174, 255, 255, 255, 255, 255], [255, 255, 255, 194, 0, 0, 0, 0, 0, 7, 22, 24, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 255, 255, 255, 255], [255, 255, 61, 0, 0, 11, 180, 255, 255, 255, 255, 255, 255, 255, 255, 203, 10, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255], [255, 57, 0, 1, 253, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 209, 0, 0, 0, 0, 0, 0, 0, 151, 255, 255], [106, 0, 129, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 208, 0, 0, 0, 0, 0, 0, 88, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 253, 0, 0, 0, 0, 0, 0, 190, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 10, 0, 0, 0, 0, 9, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 240, 0, 0, 0, 0, 28, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 60, 0, 0, 0, 121, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 92, 0, 0, 33, 233, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 198, 6, 0, 0, 190, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 219, 58, 0, 0, 0, 0, 0, 0, 0, 2, 180, 254, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 43, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 140, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 249, 248, 245, 239, 130, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 19, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 253, 203, 2, 0, 0, 0, 0, 0, 0, 0, 0, 29, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 249, 53, 0, 0, 0, 0, 0, 0, 0, 116], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 133, 0, 0, 0, 0, 0, 0, 30], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 79, 0, 0, 0, 0, 0, 2], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 6, 0, 0, 0, 0, 25], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 42, 0, 0, 0, 0, 67], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 49, 0, 0, 0, 0, 200], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 245, 2, 0, 0, 0, 87, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 41, 0, 0, 0, 43, 255, 255], [255, 179, 87, 131, 207, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 230, 7, 0, 0, 0, 183, 255, 255, 255], [1, 0, 0, 0, 0, 0, 0, 44, 208, 255, 255, 255, 255, 255, 255, 255, 255, 215, 23, 0, 0, 0, 60, 250, 255, 255, 255, 255], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 30, 33, 16, 0, 0, 0, 0, 4, 142, 251, 255, 255, 255, 255, 255, 255], [248, 102, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 107, 198, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]]
four = [[255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 75, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 35, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 29, 0, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 19, 0, 0, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 7, 0, 0, 8, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 246, 10, 0, 0, 210, 41, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 245, 0, 0, 1, 227, 255, 41, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 239, 2, 0, 0, 242, 255, 255, 41, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 222, 0, 0, 7, 243, 255, 255, 255, 41, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 228, 0, 0, 7, 255, 255, 255, 255, 255, 41, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 184, 0, 0, 14, 255, 255, 255, 255, 255, 255, 42, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 182, 0, 0, 34, 255, 255, 255, 255, 255, 255, 255, 41, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 145, 0, 0, 34, 255, 255, 255, 255, 255, 255, 255, 255, 41, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 112, 0, 0, 81, 255, 255, 255, 255, 255, 255, 255, 255, 255, 41, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 102, 0, 0, 95, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 41, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 61, 0, 0, 127, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 41, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 54, 0, 0, 173, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 41, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [33, 0, 0, 179, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 42, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 41, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 41, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 41, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 42, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 41, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 41, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 41, 0, 0, 0, 0, 66, 255, 255, 255, 255, 255]]
fife = [[255, 255, 255, 255, 255, 255, 255, 255, 255, 248, 17, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 17], [255, 255, 255, 255, 255, 255, 255, 255, 255, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 225], [255, 255, 255, 255, 255, 255, 255, 255, 54, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 176, 255], [255, 255, 255, 255, 255, 255, 255, 109, 0, 0, 21, 146, 146, 146, 146, 146, 146, 146, 146, 146, 146, 146, 146, 146, 146, 172, 255, 255], [255, 255, 255, 255, 255, 255, 159, 0, 0, 9, 243, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 199, 0, 0, 0, 227, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 228, 0, 0, 0, 200, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 247, 1, 0, 0, 0, 0, 0, 55, 156, 225, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 165, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12, 139, 255, 255, 255, 255, 255, 255, 255, 255], [255, 93, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 105, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 244, 175, 90, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 163, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 248, 74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 29, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 212, 8, 0, 0, 0, 0, 0, 0, 0, 14, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 198, 0, 0, 0, 0, 0, 0, 0, 82, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 10, 0, 0, 0, 0, 0, 3, 242], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 252, 27, 0, 0, 0, 0, 0, 160], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 231, 0, 0, 0, 0, 0, 107], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 88, 0, 0, 0, 0, 120], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 143, 0, 0, 0, 0, 191], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 125, 0, 0, 0, 28, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 34, 0, 0, 0, 220, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 135, 0, 0, 0, 201, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 106, 0, 0, 0, 210, 255, 255, 255], [73, 0, 0, 0, 0, 53, 240, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 198, 1, 0, 0, 42, 253, 255, 255, 255, 255], [0, 0, 0, 0, 0, 0, 0, 1, 56, 247, 255, 255, 255, 255, 255, 216, 34, 0, 0, 0, 13, 241, 255, 255, 255, 255, 255, 255], [56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 117, 246, 255, 255, 255, 255, 255, 255, 255, 255], [255, 217, 65, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 61, 176, 251, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]]
six = [[255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 223, 132, 30, 0, 0, 0, 0, 0, 0, 4, 252], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 176, 9, 0, 0, 0, 0, 42, 161, 254, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 249, 64, 0, 0, 0, 0, 22, 223, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 250, 44, 0, 0, 0, 0, 18, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 86, 0, 0, 0, 0, 0, 204, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 247, 4, 0, 0, 0, 0, 25, 251, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 140, 0, 0, 0, 0, 0, 37, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 109, 0, 0, 0, 0, 0, 82, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 82, 0, 0, 0, 0, 0, 28, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 157, 0, 0, 0, 0, 0, 0, 250, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 243, 0, 0, 0, 0, 0, 0, 154, 255, 255, 255, 252, 203, 152, 120, 88, 109, 149, 201, 254, 255, 255, 255, 255, 255, 255, 255], [255, 17, 0, 0, 0, 0, 0, 6, 251, 166, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 133, 252, 255, 255, 255, 255], [234, 0, 0, 0, 0, 0, 0, 0, 0, 0, 11, 68, 201, 252, 202, 107, 11, 0, 0, 0, 0, 0, 0, 0, 154, 255, 255, 255], [70, 0, 0, 0, 0, 0, 0, 5, 196, 255, 255, 255, 255, 255, 255, 255, 255, 253, 27, 0, 0, 0, 0, 0, 0, 55, 255, 255], [23, 0, 0, 0, 0, 0, 11, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 100, 0, 0, 0, 0, 0, 0, 45, 255], [0, 0, 0, 0, 0, 0, 77, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 59, 0, 0, 0, 0, 0, 0, 183], [0, 0, 0, 0, 0, 0, 166, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 249, 1, 0, 0, 0, 0, 0, 48], [0, 0, 0, 0, 0, 0, 227, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 66, 0, 0, 0, 0, 0, 11], [0, 0, 0, 0, 0, 0, 228, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 229, 0, 0, 0, 0, 0, 0], [27, 0, 0, 0, 0, 0, 145, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 11], [104, 0, 0, 0, 0, 0, 33, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 47], [255, 0, 0, 0, 0, 0, 2, 252, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 175], [255, 169, 0, 0, 0, 0, 0, 127, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 235, 0, 0, 0, 0, 11, 255], [255, 255, 86, 0, 0, 0, 0, 0, 239, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 22, 0, 0, 0, 0, 248, 255], [255, 255, 255, 69, 0, 0, 0, 0, 3, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 158, 0, 0, 0, 7, 248, 255, 255], [255, 255, 255, 255, 184, 0, 0, 0, 0, 6, 221, 255, 255, 255, 255, 255, 255, 255, 255, 100, 0, 0, 0, 42, 255, 255, 255, 255], [255, 255, 255, 255, 255, 253, 73, 0, 0, 0, 0, 31, 202, 255, 255, 255, 253, 106, 1, 0, 0, 2, 216, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 254, 162, 12, 0, 0, 0, 0, 0, 0, 0, 0, 5, 130, 246, 255, 255, 255, 255, 255, 255, 255]]
seven = [[255, 255, 255, 53, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [255, 255, 176, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 61], [255, 252, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 248], [255, 29, 0, 0, 0, 0, 68, 190, 249, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 86, 0, 0, 0, 81, 255], [176, 0, 0, 109, 253, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 230, 0, 0, 0, 0, 251, 255], [6, 3, 241, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 37, 0, 0, 0, 140, 255, 255], [27, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 224, 0, 0, 0, 14, 253, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 28, 0, 0, 0, 161, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 176, 0, 0, 0, 5, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 13, 0, 0, 0, 227, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 178, 0, 0, 0, 54, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 218, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 106, 0, 0, 0, 56, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 243, 4, 0, 0, 0, 254, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 102, 0, 0, 0, 120, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 249, 0, 0, 0, 4, 248, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 31, 0, 0, 0, 141, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 225, 0, 0, 0, 13, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 44, 0, 0, 0, 198, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 195, 0, 0, 0, 20, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 13, 0, 0, 0, 213, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 183, 0, 0, 0, 64, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 4, 0, 0, 0, 237, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 125, 0, 0, 0, 64, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 114, 0, 0, 0, 141, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 12, 252, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 48, 0, 0, 0, 152, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]]
eight = [[255, 255, 255, 255, 255, 255, 232, 116, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 41, 172, 254, 255, 255, 255, 255, 255, 255], [255, 255, 255, 252, 125, 0, 0, 0, 0, 22, 129, 235, 255, 255, 255, 255, 205, 80, 3, 0, 0, 0, 19, 209, 255, 255, 255, 255], [255, 255, 193, 0, 0, 0, 0, 55, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 210, 0, 0, 0, 0, 17, 255, 255, 255], [255, 114, 0, 0, 0, 0, 162, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 231, 0, 0, 0, 0, 4, 253, 255], [205, 0, 0, 0, 0, 33, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 105, 0, 0, 0, 0, 50, 255], [90, 0, 0, 0, 0, 28, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 166, 0, 0, 0, 0, 20, 255], [61, 0, 0, 0, 0, 0, 187, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 104, 0, 0, 0, 0, 30, 255], [100, 0, 0, 0, 0, 0, 0, 219, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 242, 1, 0, 0, 0, 0, 167, 255], [239, 4, 0, 0, 0, 0, 0, 0, 98, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 40, 0, 0, 0, 0, 126, 255, 255], [255, 182, 0, 0, 0, 0, 0, 0, 0, 2, 138, 255, 255, 255, 255, 255, 255, 255, 236, 2, 0, 0, 0, 1, 216, 255, 255, 255], [255, 255, 225, 0, 0, 0, 0, 0, 0, 0, 0, 0, 108, 252, 255, 255, 252, 75, 0, 0, 0, 5, 160, 255, 255, 255, 255, 255], [255, 255, 255, 255, 91, 0, 0, 0, 0, 0, 0, 0, 0, 0, 56, 120, 0, 0, 0, 23, 222, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 253, 51, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 124, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 31, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 233, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 238, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 186, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 185, 7, 0, 0, 18, 239, 255, 151, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 204, 255, 255, 255, 255], [255, 255, 255, 226, 9, 0, 0, 0, 160, 255, 255, 255, 255, 255, 193, 0, 0, 0, 0, 0, 0, 0, 0, 0, 26, 255, 255, 255], [255, 255, 62, 0, 0, 0, 0, 207, 255, 255, 255, 255, 255, 255, 255, 255, 177, 3, 0, 0, 0, 0, 0, 0, 0, 3, 238, 255], [255, 30, 0, 0, 0, 0, 135, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 132, 0, 0, 0, 0, 0, 0, 0, 10, 246], [77, 0, 0, 0, 0, 19, 253, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 31, 0, 0, 0, 0, 0, 0, 84], [0, 0, 0, 0, 0, 82, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 123, 0, 0, 0, 0, 0, 13], [0, 0, 0, 0, 0, 122, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 67, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 86, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 152, 0, 0, 0, 0, 46], [80, 0, 0, 0, 0, 0, 237, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 149, 0, 0, 0, 0, 178], [255, 33, 0, 0, 0, 0, 24, 253, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 246, 9, 0, 0, 0, 117, 255], [255, 255, 88, 0, 0, 0, 0, 3, 228, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 215, 4, 0, 0, 0, 212, 255, 255], [255, 255, 255, 248, 65, 0, 0, 0, 0, 5, 105, 239, 255, 255, 255, 255, 255, 237, 94, 1, 0, 0, 1, 157, 254, 255, 255, 255], [255, 255, 255, 255, 255, 255, 207, 68, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 141, 245, 255, 255, 255, 255, 255, 255]]
nine = [[255, 255, 255, 255, 255, 255, 255, 249, 136, 9, 0, 0, 0, 0, 0, 0, 0, 0, 61, 196, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 218, 5, 0, 0, 0, 31, 140, 225, 246, 195, 81, 0, 0, 0, 0, 0, 101, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 52, 0, 0, 0, 36, 255, 255, 255, 255, 255, 255, 255, 255, 170, 1, 0, 0, 0, 0, 192, 255, 255, 255, 255], [255, 255, 247, 9, 0, 0, 0, 104, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 251, 0, 0, 0, 0, 0, 124, 255, 255, 255], [255, 249, 3, 0, 0, 0, 10, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 229, 0, 0, 0, 0, 0, 98, 255, 255], [255, 23, 0, 0, 0, 0, 176, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 101, 0, 0, 0, 0, 0, 212, 255], [199, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 247, 0, 0, 0, 0, 0, 0, 255], [59, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 12, 0, 0, 0, 0, 0, 133], [24, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 130, 0, 0, 0, 0, 0, 27], [3, 0, 0, 0, 0, 0, 242, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 208, 0, 0, 0, 0, 0, 1], [21, 0, 0, 0, 0, 0, 68, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 236, 0, 0, 0, 0, 0, 0], [52, 0, 0, 0, 0, 0, 3, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 179, 0, 0, 0, 0, 0, 0], [188, 0, 0, 0, 0, 0, 0, 95, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 88, 0, 0, 0, 0, 0, 0], [255, 38, 0, 0, 0, 0, 0, 0, 138, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 12, 0, 0, 0, 0, 0, 20], [255, 252, 35, 0, 0, 0, 0, 0, 0, 55, 253, 255, 255, 255, 255, 255, 255, 255, 255, 228, 36, 0, 0, 0, 0, 0, 0, 84], [255, 255, 255, 96, 0, 0, 0, 0, 0, 0, 0, 25, 158, 205, 210, 181, 102, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 222], [255, 255, 255, 255, 251, 80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 126, 255, 25, 0, 0, 0, 0, 0, 35, 255], [255, 255, 255, 255, 255, 255, 255, 255, 177, 71, 51, 38, 51, 71, 178, 255, 255, 255, 255, 195, 0, 0, 0, 0, 0, 0, 243, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 0, 0, 0, 0, 0, 0, 176, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 33, 0, 0, 0, 0, 0, 99, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 77, 0, 0, 0, 0, 0, 153, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 19, 0, 0, 0, 0, 0, 174, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 251, 18, 0, 0, 0, 0, 8, 250, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 198, 0, 0, 0, 0, 0, 162, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 34, 0, 0, 0, 0, 87, 253, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 255, 255, 255, 255, 225, 35, 0, 0, 0, 0, 145, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 255, 255, 255, 255, 248, 147, 37, 0, 0, 0, 0, 69, 232, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255], [255, 0, 0, 0, 0, 0, 0, 16, 86, 180, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]]

cv2.imshow("0", np.array(zero, dtype=np.uint8))
cv2.imshow("1", np.array(one, dtype=np.uint8))
cv2.imshow("2", np.array(two, dtype=np.uint8))
cv2.imshow("3", np.array(three, dtype=np.uint8))
cv2.imshow("4", np.array(four, dtype=np.uint8))
cv2.imshow("5", np.array(fife, dtype=np.uint8))
cv2.imshow("6", np.array(six, dtype=np.uint8))
cv2.imshow("7", np.array(seven, dtype=np.uint8))
cv2.imshow("8", np.array(eight, dtype=np.uint8))
cv2.imshow("9", np.array(nine, dtype=np.uint8))

while True:
    key = cv2.waitKey(0)
    if key == ord('q'):
        break