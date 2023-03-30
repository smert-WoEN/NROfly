import cv2
import numpy as np
import requests


url =  "https://stepik.org/media/attachments/course/128568/shelfQR0.png"
resp = requests.get(url, stream=True).raw
image = np.asarray(bytearray(resp.read()), dtype=np.uint8)
image = cv2.imdecode(image, cv2.IMREAD_COLOR)

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blur = cv2.medianBlur(gray, 5)
adapt_type = cv2.ADAPTIVE_THRESH_GAUSSIAN_C
thresh_type = cv2.THRESH_BINARY_INV
bin_img = cv2.adaptiveThreshold(blur, 255, adapt_type, thresh_type, 11, 2)

rho, theta, thresh = 2, np.pi/180, 600
lines = cv2.HoughLines(bin_img, rho, theta, thresh)

vert_lines = []
hor_lines = []
if lines is not None:
    for i in range(len(lines)):
        if abs(lines[i][0][1]) < 0.01:
            vert_lines.append(lines[i])
        elif abs(lines[i][0][1] - np.pi/2) < 0.01:
            hor_lines.append(lines[i])

intersection_points = []
for h_line in hor_lines:
    for v_line in vert_lines:
        app = True
        for i in intersection_points:
            if np.sqrt((i[0]-int(v_line[0][0]))**2 + (i[1]-int(h_line[0][0]))**2) < 5:
                app = False
        if app:
            intersection_points.append((int(v_line[0][0]),int(h_line[0][0])))
            # cv2.circle(image, intersection_points[-1],3, (0,0,255), 3)

intersection_points = np.array(intersection_points)
x_s = np.sort(intersection_points[:,1])
y_s = np.sort(intersection_points[:,0])

num_cols = 1
for x_i in range(x_s.shape[0]):
    if abs(x_s[x_i] - x_s[x_i+1]) < 5:
        num_cols += 1
    else:
        break

num_rows = 1
for y_i in range(y_s.shape[0]):
    if abs(y_s[y_i] - y_s[y_i+1]) < 5:
        num_rows += 1
    else:
        break

# print(x_s,y_s)

qcd = cv2.QRCodeDetector()
for shelf in range(1,num_rows):
    for ser in range(1,num_cols):
        
        qr_frame = image[x_s[(num_rows-shelf-1)*num_cols]+5:x_s[(num_rows-shelf)*num_cols],y_s[(ser-1)*num_rows]+6:y_s[(ser)*num_rows]]
        qr_frame_base = qr_frame.copy()
        qcd.setEpsY(0.1)
        qcd.setEpsX(0.2)
        decoded_info, points, _ = qcd.detectAndDecode(qr_frame)
        if points is None:
            array = np.full((len(qr_frame) * 2, len(qr_frame[0]) * 2, 3), 255, dtype=np.uint8)
            for i in range(len(qr_frame)):
                for j in range(len(qr_frame[0])):
                    array[i + len(qr_frame) // 2][j + len(qr_frame[0]) // 2] = qr_frame[i][j]
            qr_frame = array
            decoded_info, points, _ = qcd.detectAndDecode(qr_frame)
        if points is None:
            array = np.full((len(qr_frame) * 3, len(qr_frame[0]) * 3, 3), 255, dtype=np.uint8)
            for i in range(len(qr_frame)):
                for j in range(len(qr_frame[0])):
                    array[i + len(qr_frame)][j + len(qr_frame[0])] = qr_frame[i][j]
            qr_frame = array
            decoded_info, points, _ = qcd.detectAndDecode(qr_frame)
        if points is None:
            qr_frame = qr_frame_base.copy()
            blur_qr = cv2.GaussianBlur(qr_frame, (25, 25), cv2.BORDER_DEFAULT)
            hsv = cv2.cvtColor(blur_qr, cv2.COLOR_BGR2HSV)
            hsvMin = np.array((0, 0, 255), np.uint8)
            hsvMax = np.array((255, 255, 255), np.uint8)
            # накладываем фильтр на кадр в модели HSV
            thresh = cv2.inRange(hsv, hsvMin, hsvMax)
            _, counter, _ = cv2.findContours(thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
            #print(counter)
            if len(counter) > 1 and points is None:
                x, y, w, h = cv2.boundingRect(counter[1])
                qcd.setEpsY(0.2)
                qcd.setEpsX(0.2)
                decoded_info, points, _ = qcd.detectAndDecode(qr_frame_base)
        #decoded_info, points, _ = qcd.detectAndDecode(qr_frame)


        #print(decoded_info, points)
        print(f"{shelf}-я полка {ser}-й ряд. ",end="")
        if points is not None:
            decoded_info = decoded_info.split("; ")
            # print(decoded_info)
            print(decoded_info[0],end=". ")
            if shelf == int(decoded_info[1]) and ser == int(decoded_info[2]):
                print("Расположение верное.")
            else:
                print("Расположение неверное.")
        else:
            print("Товар отсутствует.")
        
        # cv2.imshow("",qr_frame)
        # while True:
        #     key = cv2.waitKey(0)
        #     if key == ord('q'):
        #         break


