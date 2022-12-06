import numpy as np

import cv2
import requests


def read_image(url):
    resp = requests.get(url, stream=True).raw
    image = np.asarray(bytearray(resp.read()), dtype=np.uint8)
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)
    return image

def show(image): 
    cv2.imshow("Image", image)

def gray_blur(image):
    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    img_gray_blur = cv2.GaussianBlur(img_gray, (5, 5), 0)
    return img_gray_blur

def set_threshold(image):
    ret, img_threshold = cv2.threshold(image, 90, 255, cv2.THRESH_BINARY_INV)
    return img_threshold

def detect_countours(img_threshold):
    contours, hier = cv2.findContours(img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    rects = [cv2.boundingRect(contour) for contour in contours]

    return rects


def main():
    image = read_image("https://stepik.org/media/attachments/course/128568/1234567890.png")
    show(image)

    img_gray_blur = gray_blur(image)
    #show(img_gray_blur)

    img_threshold = set_threshold(img_gray_blur)
    #show(img_threshold)

    rects = detect_countours(img_threshold)
    

    for rect in rects:
        cv2.rectangle(image, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 3)
        show(image) 
    
    cv2.waitKey(0)


if __name__ == "__main__":
    main()