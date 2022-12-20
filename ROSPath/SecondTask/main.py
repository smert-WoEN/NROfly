import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range, Image
import math
import cv2
from cv_bridge import CvBridge
import numpy as np
from pyzbar import pyzbar


def check_row_results(array):
    out_str = ""
    count = 0
    i = 0
    while i < len(array):
        mean = [array[i]]
        for j in range(i + 1, len(array)):
            if abs(mean[0][1] - array[j][1]) < 0.5:
                mean.append(array[j])
            else:
                i = j - 1
                break
        else:
            count += 1
            qr_msg = ""
            for k in mean:
                if len(k[3]) > 0:
                    qr_msg = k[3]
                    break
            else:
                qr_msg = "none"
            size = ""
            if mean[0][2] > 1.2:
                size = "Low"
            elif mean[0][2] > 0.6:
                size = "Middle"
            else:
                size = "High"
            mean_new = []
            for k in mean:
                mean_new.append([k[0], k[1]])
            pos = np.mean(np.array(mean_new), axis=0)
            out_str += (str(count) + ")" + size + " place, cords: x = " + str(round(pos[0], 1)) + ", y = " + str(round(pos[1], 1)) + ", z = ")
            if size == "Low":
                out_str += "0.5"
            elif size == "Middle":
                out_str += "1.0"
            else:
                out_str += "1.5"
            out_str += (", Qr code: " + qr_msg + "\n")
            break
        count += 1
        qr_msg = ""
        for k in mean:
            if len(k[3]) > 0:
                qr_msg = k[3]
                break
        else:
            qr_msg = "none"
        size = ""
        if mean[0][2] > 1.2:
            size = "Low"
        elif mean[0][2] > 0.6:
            size = "Middle"
        else:
            size = "High"
        mean_new = []
        for k in mean:
            mean_new.append([k[0], k[1]])
        pos = np.mean(np.array(mean_new), axis=0)
        out_str += (str(count) + ")" + size + " place, cords: x = " + str(round(pos[0], 1)) + ", y = " + str(round(pos[1], 1)) + ", z = ")
        if size == "Low":
            out_str += "0.5"
        elif size == "Middle":
            out_str += "1.0"
        else:
            out_str += "1.5"
        out_str += (", Qr code: " + qr_msg + "\n")
        i += 1
    return out_str


rospy.init_node('flight')
bridge = CvBridge()


get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy("led/set_effect", srv.SetLEDEffect)



def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=1, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


def pos_wait(x=0, y=0, z=0, yaw=float('nan'), speed=1, frame_id='', auto_arm=False, tolerance=0.2):
    set_position(x=x, y=y, z=z, yaw=yaw, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def takeoff(z=1.8):
    navigate_wait(x=0, y=0, z=0, frame_id='body', auto_arm=True)
    set_effect(effect='blink', r=0, g=0, b=255)
    navigate_wait(x=0, y=0, z=z, frame_id='body')


def landing():
    set_effect(effect='blink', r=255, g=0, b=0)
    land()
    rospy.sleep(4)


def navigate_with_green_light(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.1):
    set_effect(r=0, b=0, g=255)
    navigate_wait(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm, tolerance=tolerance)


def get_distant():
    return rospy.wait_for_message('rangefinder/range', Range).range


def get_image():
    return bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')


def check_pos():
    distant = get_distant()


def get_pos():
    pos = get_telemetry(frame_id='aruco_map')
    return [pos.x, pos.y]


def read_qrCode(x, y, distant):
    set_effect(effect='blink', r=255, g=255, b=0)
    navigate_wait(x=x, z = 2 - (distant - 0.2), y = y, frame_id='aruco_map', yaw=0.0, tolerance=0.1)
    pos = get_pos()
    image = get_image()
    data = pyzbar.decode(image)
    read_data = []
    if len(data) > 0:
        read_data = [pos[0], pos[1], distant, (data[0].data.decode("utf-8").replace("Â", "").replace("\xa0", " "))]
    else:
        read_data = [pos[0], pos[1], distant, ""]
    navigate_wait(x=x, z = 2, y = y, frame_id='aruco_map', yaw=0.0, tolerance=0.1)
    return read_data


def check_row(x, y):
    array=[]
    for i in range(0, y):
        navigate_with_green_light(x=x, z=2, y=i/10, frame_id='aruco_map', yaw=0.0, tolerance=0.1)
        distant = get_distant()
        if distant < 1.6:
            pos = get_pos()
            array.append([pos[0], pos[1], distant, ""])
            array.append(read_qrCode(x, y=i/10, distant=distant))
            pos = get_pos()
            array.append([pos[0], pos[1], distant, ""])
    return array


set_effect(effect='blink', r=0, g=0, b=255)
takeoff()
out_str = "Left shelving:\n" + (check_row_results(check_row(0.93, 56)))
out_str += ("Right shelving:\n" + (check_row_results(check_row(2.73, 56))))
navigate_with_green_light(x=0, y=0, z=2, frame_id='aruco_map')
landing()
file = open("result.txt", 'w')
file.write(out_str)
file.close()
print(out_str)




