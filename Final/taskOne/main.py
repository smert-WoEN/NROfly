import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range, Image
import math
import cv2
from cv_bridge import CvBridge
import numpy as np
from pyzbar import pyzbar
import time

X_OFFSET = 0.05
Y_OFFSET = 0.0
STEP_X = 1.5
STEP_Y = 1.5
WALL_DIST = 1.25

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

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), set_angle=0, speed=1, frame_id='', auto_arm=False, tolerance=0.12, yaw_rate=0.0, tolerance_angle = math.radians(5.0)):
    if (set_angle < 0):
        yaw_rate *= -1
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm, yaw_rate=yaw_rate)
    
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        # print(telem.x,telem.y,telem.z,telem.yaw)
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance and abs(telem.yaw) < tolerance_angle:
            # navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm, yaw_rate=0.0)
            return telem.x, telem.y, telem.z, telem.yaw
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


def get_image():
    return bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')


def read_qr(image):
    data = pyzbar.decode(image)
    return data


def data0_from_qr_data(data):
    return data[0].data.decode("utf-8")



def check_forward_wall(dist=1.0):
    return get_forward_range() < dist


def check_right_wall(dist=1.0):
    return get_right_range() < dist


def check_left_wall(dist=1.0):
    return get_left_range() < dist


def get_image():
    return bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')


def read_qr(image):
    data = pyzbar.decode(image)
    return data


def data0_from_qr_data(data):
    return data[0].data.decode("utf-8")


def move(dir, x_, y_, z_, yaw_):
    if dir == 0:
        x_ += STEP_X
        yaw_=0
    elif dir == 1:
        y_ -= STEP_Y
        yaw_=-90
    elif dir == 2:
        x_ -= STEP_X
        yaw_=180
    elif dir == 3:
        y_ += STEP_Y
        yaw_=90

    print(f"Going to {x_}, {y_}, {z_}\n")

    navigate_wait(x = x_,y=y_,z=z_,yaw=math.radians(yaw_), frame_id='map', speed=0.3)
    # set_position(x_,y_,z_)
    return x_, y_, z_, yaw_

def move_body(dir,last_dir,  x_,y_,z_,yaw_, delta_):
    if dir == 0:
        x_ += STEP_X
        yaw_=0
    elif dir == 1:
        y_ -= STEP_X
        yaw_=-90
    elif dir == 2:
        x_ -= STEP_X
        yaw_=180
    elif dir == 3:
        y_ += STEP_X
        yaw_=90
    print(f"Going to {x_}, {y_}, {z_}\n")
    delta_ = navigate_wait(x=delta_[0],y=delta_[1],z=0,yaw=math.radians((-dir+last_dir)*90)- delta_[3], frame_id='body', speed=0.3)
    delta_ = navigate_wait(x = ((STEP_X - Y_OFFSET) if dir % 2 != 0 else STEP_X - X_OFFSET)*(2 - math.cos(delta_[3]))
                                 - (delta[0] if dir % 2 == 0 else delta[1]),
                           y = STEP_X*(-math.sin(delta_[3]))
                                 - (delta[1] if dir % 2 == 0 else delta[0]),
                           yaw=-delta_[3], frame_id='body', speed=0.3)
    return x_, y_, z_, yaw_, delta_

class Cell:
    def __init__(self, sides=None):
        """

        :param point: position af a cell
        :param sides: forward, back, left, right, True if a wall exists
        """

        self.forward = sides[0]
        self.right = sides[1]
        self.back = sides[2]
        self.left = sides[3]
        

        self.sides = [
            self.forward,
            self.back,
            self.left,
            self.right
        ]

    def set_sides(self, sides):
        self.sides = sides  
        self.forward = sides[0]
        self.right = sides[1]
        self.back = sides[2]
        self.left = sides[3]
        

maze_map = [[Cell([False]*4)]*5]*6

def print_maze_map(maze_map):
    # maze_size_x = len(maze_map)
    # maze_size_y = len(maze_map[0])
    # print(f"maze_size_x: {maze_size_x} \nmaze_size_y: {maze_size_x}\n\n")

    for i, row in enumerate(maze_map, 0):
        for j, cell in enumerate(row, 0):
            messages = []
            if not cell.back:
                messages.append(f"{j + i*4 + 1}-{j + (i-1)*4 + 1}")
            if not cell.left:
                messages.append(f"{j + i*4 + 1}-{j - 1 + i*4 + 1}")
            if not cell.right:
                messages.append(f"{j + i*4 + 1}-{j + 1 + i*4 + 1}")
            if not cell.forward:
                messages.append(f"{j + i*4 + 1}-{j + (i+1)*4 + 1}")
            for k, message in enumerate(messages, 0):
                if k != len(messages) - 1:
                    print(message, end="; ")
                else:
                    print(message, end="")

            print("")



# print(read_qr(get_image()))
# cv2.imshow('a', get_image())
# cv2.waitKey(0)
# exit()
x, y, z, yaw = 0, 0, 0.75, 0
direction = 4 # 0 - increase x, 1 - decrease y, 2 - decrease x, 3 - increase y, 4 - increase z
delta = navigate_wait(x=0, y=0, z=0,yaw=yaw, frame_id='body', auto_arm=True, speed=0.3)
delta = navigate_wait(x = 0,y=0,z=z,yaw=math.radians(yaw) + delta[3], frame_id='body', speed=0.3)



print(read_qr(get_image()))
print(read_qr(get_image()))
print(read_qr(get_image()))

start_time = time.time()
direction = 0
while time.time() - start_time < 360:
    right =  check_right_wall(WALL_DIST)
    left = check_left_wall(WALL_DIST)
    forward = check_forward_wall(WALL_DIST)
    sd = [False]*4
    if right:
        sd[(direction+1)%4] = True
    if left:
        sd[(direction-1)%4] = True
    if forward:
        sd[direction] = True
    maze_map[round(x/STEP_X)][round(y/STEP_Y)].set_sides(sd)
    print(sd)

    last_direction = direction
    if not right:
        direction = (direction + 1) % 4
        print("RIGHT")
    elif not forward:
        print("FORWARD")
    elif not left:
        direction = (direction - 1) % 4
        print("LEFT")
    else:
        direction = (direction + 2) % 4
        print("BACK")
    x, y, z, yaw, delta = move_body(direction,last_direction, x, y, z, yaw, delta)
    if x == 0.0 and y == 0.0:
        break
    print(read_qr(get_image()))


landing()

print_maze_map(maze_map)
