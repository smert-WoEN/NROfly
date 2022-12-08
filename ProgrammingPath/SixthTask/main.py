import numpy as np

import cv2
import requests


def read_image(url):
    resp = requests.get(url, stream=True).raw
    image = np.asarray(bytearray(resp.read()), dtype=np.uint8)
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)
    return image


def show(image, name="Image"):
    cv2.imshow(name, image)


def gray_blur(image):
    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    img_gray_blur = cv2.GaussianBlur(img_gray, (5, 5), 0)
    return img_gray_blur


def set_threshold(image):
    ret, img_threshold = cv2.threshold(image, 90, 255, cv2.THRESH_BINARY_INV)
    return img_threshold


def detect_countours(img_threshold):
    _, contours, hier = cv2.findContours(img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    rects = [cv2.boundingRect(contour) for contour in contours]

    return rects


def main():
    image = read_image("https://stepik.org/media/attachments/course/128568/1234567890.png")
    show(image)

    img_gray_blur = gray_blur(image)
    show(img_gray_blur, name="1")

    img_threshold = set_threshold(img_gray_blur)
    show(img_threshold, name="2")

    rects = detect_countours(img_threshold)
    
    digits = []
    for rect in rects:
        x, y, w, h = rect[0], rect[1], rect[2], rect[3]
        digit = image[y: y + h, x: x + w]
        digits.append([x, digit])

        image_copy = image.copy()
        cv2.rectangle(image_copy, (x, y), (x + w, y + h), (0, 255, 0), 3)
        show(image_copy, name=str(rect[0]))
    
    # Так как opencv определяет контуры одним известным местом, цифры он вырезает не по порядку. Нужно сортировать по x
    digits.sort()
    # После сортировки кооридната x уже не нужна
    digits = [digits[i][1]/255.0 for i in range(len(digits))] 
    show(digits[0], name="15")


    cv2.waitKey(0)


if __name__ == "__main__":
    main()
