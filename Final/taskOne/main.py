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
        # print(telem.x,telem.y,telem.z,telem.yaw)
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance and abs(telem.yaw) < tolerance_angle:
            # navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm, yaw_rate=0.0)
            break
        rospy.sleep(0.05)
        

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


def move(dir, x_, y_, z_, yaw_):
    if dir == 0:
        x_ += 1.5
        yaw_=0
    elif dir == 1:
        y_ -= 1.5
        yaw_=-90
    elif dir == 2:
        x_ -= 1.5
        yaw_=180
    elif dir == 3:
        y_ += 1.5
        yaw_=90
    elif dir == 4:
        z_ = 1.1
    
    print(f"Going to {x_}, {y_}, {z_}\n")

    navigate_wait(x = x_,y=y_,z=z_,yaw=math.radians(yaw_), frame_id='map', speed=0.1)
    set_position(x_,y_,z_)
    return x_, y_, z_, yaw_


x, y , z, yaw = 0, 0 ,1.1, 0
direction = 4 # 0 - increase x, 1 - decrease y, 2 - decrease x, 3 - increase y, 4 - increase z
navigate_wait(x=0, y=0, z=0,yaw=yaw, frame_id='body', auto_arm=True, speed=0.1)
move(direction,x,y,z,yaw)
import time
start_time = time.time()
direction = 0
while time.time() - start_time < 300:
    if not check_right_wall():
        if direction == 0:
            direction = 1
        elif direction == 1:
            direction = 2
        elif direction == 2:
            direction = 3
        elif direction == 3:
            direction = 0
        print("RIGHT")
    elif not check_forward_wall():
        print("STRAIGHT")
    elif not check_left_wall():
        if direction == 0:
            direction = 3
        elif direction == 1:
            direction = 0
        elif direction == 2:
            direction = 1
        elif direction == 3:
            direction = 2
        print("LEFT")
    else:
        if direction == 0:
            direction = 2
        elif direction == 1:
            direction = 3
        elif direction == 2:
            direction = 0
        elif direction == 3:
            direction = 1
        print("DOWN")
    x, y, z, yaw = move(direction, x ,y, z,yaw)
    


landing()

