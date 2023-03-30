import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range, Image
import math
import cv2
from cv_bridge import CvBridge
import numpy as np
from pyzbar import pyzbar

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


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), set_angle=0, speed=1, frame_id='', auto_arm=False, tolerance=0.1, yaw_rate=0.0, tolerance_angle = math.radians(5.0)):
    if (set_angle < 0):
        yaw_rate *= -1
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm, yaw_rate=yaw_rate)
    
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance and abs(set_angle - telem.yaw) < tolerance_angle:
            navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm, yaw_rate=0.0)
            break
        rospy.sleep(0.2)
        

def landing():
    set_effect(effect='blink', r=255, g=0, b=0)
    land()
    rospy.sleep(4)


def get_altitude():
    return rospy.wait_for_message('rangefinder/range', Range).range


def get_forward_range():
    return rospy.wait_for_message('rangefinder_forward/range', Range).range


def get_right_range():
    return rospy.wait_for_message('rangefinder_right/range', Range).range


def get_left_range():
    return rospy.wait_for_message('rangefinder_left/range', Range).range


def check_forward_wall(dist=1.2):
    return get_forward_range() < dist


def check_right_wall(dist=1.2):
    return get_right_range() < dist


def check_left_wall(dist=1.2):
    return get_left_range() < dist



def takeoff(z=1.1):
    navigate_wait(x=0, y=0, z=0, frame_id='body', auto_arm=True, speed=0.2)
    set_effect(effect='blink', r=0, g=0, b=255)
    navigate_wait(x=0, y=0, z=z, frame_id='body', speed=0.2)


def rotate(angle):
    navigate_wait(set_angle=math.radians(angle), frame_id='body', speed=0.2, yaw_rate=0.2)


def move_forward(dist):
    navigate_wait(x = dist, frame_id='body', speed=0.2)


takeoff()
import time
start_time = time.time()
print('no')
while time.time() - start_time < 300:
    print('yes')
    if not check_right_wall():
        rotate(-90.0)
        move_forward(1.5)
    elif not check_forward_wall():
        move_forward(1.5)
    elif not check_left_wall():
        rotate(90.0)
        move_forward(1.5)
    else:
        rotate(180.0)
        move_forward(1.5)


landing()
