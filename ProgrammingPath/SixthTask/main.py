import numpy as np

import cv2
import requests

import sklearn
from sklearn.linear_model import LogisticRegression


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


features = [[] for i in range(10)]
features[0] = [5.1, 22.1, 5.1, 22.1, 5.13, 5.51, 5.13, 5.51, 13.74, 13.64, 13.74, 13.64, 0, 0, 0, 0] # 14 + 2  f
features[1] = [7.57, 6.14, 7.66, 23.78, 21.48, 23.79, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] # 10 + 6 f
features[2] = [10.41, 23.58, 14.15, 7.91, 15.14, 7.71, 22.18, 7.04, 22.18, 7.04, 24.94, 24.91, 0, 0, 0, 0] # 14 + 2 f
features[3] = [14.16, 13.25, 14.68, 6.79, 14.68, 6.79, 16.95, 13.04, 17.71, 20.01, 21.96, 5.38, 0, 0, 0, 0] # 12 + 4  f
features[4] = [5.9, 16.37, 13.63, 13.38, 15.36, 15.78, 15.36, 15.78, 15.45, 22.17, 15.45, 22.17, 15.86, 7.12, 19.73, 19.1] # 10 + 6 f
features[5] = [6.2, 8.29, 6.2, 8.29, 8.55, 3.62, 12.14, 5.41, 12.14, 5.41, 15.61, 18.8, 17.92, 18.6, 0, 0] # 16 f
features[6] = [5.15, 22.98, 5.93, 9.59, 5.93, 9.59, 10.96, 8.93, 21.21, 13.21, 22.99, 23.39, 0, 0, 0, 0] # 12 + 4 f
features[7] = [2.99, 2.95, 19.29, 4.69, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] # 0 + 16 f
features[8] = [4.09, 3.16, 4.26, 24.33, 5.01, 17.21, 9.18, 5.27, 10.41, 12.26, 11.12, 16.67, 13.94, 6.03, 14.63, 9.4] # 16 f
features[9] = [4.56, 4.15, 5.95, 14.13, 16.66, 18.51, 17.17, 18.15, 22.29, 4.54, 0, 0, 0, 0, 0, 0] # 8 f


def get_features(i):
    sift = cv2.SIFT_create()

    img = cv2.imread(str(i) + ".png")
    img = gray_blur(img)

    img_resized = cv2.resize(img, (28, 28))

    keypoints, descriptors = sift.detectAndCompute(img_resized, None)

    features_9 =  []
    for keypoint in keypoints:
        features_9.append(round(keypoint.pt[0], 2))
        features_9.append(round(keypoint.pt[1], 2))

    #print(features_9)
    #print(len(features_9))


def train_classifier():
    X = features
    labels = [i for i in range(10)]

    classifier = LogisticRegression()
    classifier.fit(X, labels)
    return classifier

MAX_VERT_LEN = 60

def main():
    #url = input()
    for i in range(10):
        get_features(i)
    url = "https://stepik.org/media/attachments/course/128568/1234567890.png"
    image = read_image(url)
    #show(image)

    img_gray_blur = gray_blur(image)
    #show(img_gray_blur)

    img_threshold = set_threshold(img_gray_blur)
    #show(img_threshold)

    rects = detect_countours(img_threshold)
    
    digits = []
    for rect in rects:
        x, y, w, h = rect[0], rect[1], rect[2], rect[3]
        digit = image[y: y + h, x: x + w]
        digits.append([x, digit])

        #image_copy = image.copy()
        #cv2.rectangle(image_copy, (x, y), (x + w, y + h), (0, 255, 0), 3)
        #show(image_copy) 
    
    # Так как opencv определяет контуры одним известным местом, цифры он вырезает не по порядку. Нужно сортировать по x
    digits.sort()
    # После сортировки координата x уже не нужна
    digits = [digits[i][1] for i in range(len(digits))] 
    #show(digits[0])
    
    classifier = train_classifier()

    answ = ""
    i = -1
    for digit in digits:
        i += 1
        if len(digit) <= MAX_VERT_LEN:
            answ += ","
            continue;

        digit_resized = cv2.resize(digit, (28, 28), interpolation=cv2.INTER_AREA)
        digit_resized = cv2.dilate(digit_resized, (3, 3))

        sift = cv2.SIFT_create()
        keypoints, descriptors = sift.detectAndCompute(digit_resized, None)
        feature = []
        for keypoint in keypoints:
            feature.append(round(keypoint.pt[0], 2))
            feature.append(round(keypoint.pt[1], 2))

        if len(feature) > 16:
            feature = feature[:16]
        else:
            while len(feature) != 16:
                feature.append(0)
        
        #print([feature[j] - features[i][j] for j in range(10)])
        #print(len(features))
        predict = classifier.predict([feature])
        answ += str(predict[0])


    #cv2.waitKey(0)
    print(answ)

if __name__ == "__main__":
    main()