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
import re
from pymavlink import mavutil
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import State

X_OFFSET = 0.20
Y_OFFSET = -0.0
STEP_X = 1.5
STEP_Y = 1.5
WALL_DIST = 1.4

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

send_command = rospy.ServiceProxy('mavros/cmd/command', CommandLong)

def calibrate_gyro():
    rospy.loginfo('Calibrate gyro')
    if not send_command(command=mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, param1=1).success:
        return False

    calibrating = False
    while not rospy.is_shutdown():
        state = rospy.wait_for_message('mavros/state', State)
        if state.system_status == mavutil.mavlink.MAV_STATE_CALIBRATING or state.system_status == mavutil.mavlink.MAV_STATE_UNINIT:
            calibrating = True
        elif calibrating and state.system_status == mavutil.mavlink.MAV_STATE_STANDBY:
            rospy.loginfo('Calibrating finished')
            return True

calibrate_gyro()

MAP_SIZE= (6,4)
x, y, z, yaw = 0, 0, 0.65, 0
qr_data = dict()
def try_decode(x_,y_):
    global qr_data
    zone = round(x_/1.5)*MAP_SIZE[1] + 1 + round(-y_/1.5)
    qrs = read_qr(get_image())
    for i, qr in enumerate(qrs,1):
        st = qr.data.decode('utf-8')
        if st in qr_data.keys():
            qr_data[st].append(zone)
        else:
            qr_data[st] = [zone]
        print(f"Zone {zone}, Cargo={st}")

def navigate_wait(x_=0, y_=0, z_=0, yaw_=float('nan'), set_angle=0, speed=1, frame_id='', auto_arm=False, tolerance=0.12, yaw_rate=0.0, tolerance_angle = math.radians(5.0)):
    if (set_angle < 0):
        yaw_rate *= -1
    navigate(x=x_, y=y_, z=z_, yaw=yaw_, speed=speed, frame_id=frame_id, auto_arm=auto_arm, yaw_rate=yaw_rate)
    start_time1 = time.time()
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        try_decode(x ,y )
        if time.time() - start_time1 < 15 and math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance and abs(telem.yaw) < tolerance_angle:
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
    delta_ = navigate_wait(x_=delta_[0],y_=delta_[1],z_=0,yaw_=math.radians((-dir+last_dir)*90)- delta_[3], frame_id='body', speed=0.3)
    delta_ = navigate_wait(x_ = (STEP_X - (X_OFFSET if dir % 2 == 0 else 0)) #(-Y_OFFSET if dir == 1 else (-X_OFFSET if dir == 2 else Y_OFFSET))))
                                    *(2 - math.cos(delta_[3]))
                                 - (delta_[0] if dir % 2 == 0 else delta_[1]),
                           y_ = (STEP_X) # - (X_OFFSET if dir == 0 else (-Y_OFFSET if dir == 1 else (-X_OFFSET if dir == 2 else Y_OFFSET))))
                                    *(-math.sin(delta_[3]))
                                 - (delta_[1] if dir % 2 == 0 else delta_[0]),
                           yaw_=-delta_[3], frame_id='body', speed=0.3)
    # delta_ = navigate_wait(x_ = (STEP_X - X_OFFSET)*(2 - math.cos(delta_[3]))
    #                              - (delta_[0] if dir % 2 == 0 else delta_[1]),
    #                        y_ = (STEP_Y-Y_OFFSET)*(-math.sin(delta_[3]))
    #                              - (delta_[1] if dir % 2 == 0 else delta_[0]),
    #                        yaw_=-delta_[3], frame_id='body', speed=0.3)
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
        

maze_map = []
for i in range(MAP_SIZE[0]):
    maze_map.append([])
    for j in range(MAP_SIZE[1]):
        maze_map[i].append(Cell([True,True,True,True]))

def print_maze_map(maze_map):
    print("----Start map----")
    for i, row in enumerate(maze_map, 0):
        for j, cell in enumerate(row, 0):
            messages = []
            if not cell.back:
                messages.append(f"{j + i*MAP_SIZE[1] + 1}-{j + (i-1)*MAP_SIZE[1] + 1}")
            if not cell.left:
                messages.append(f"{j + i*MAP_SIZE[1] + 1}-{j - 1 + i*MAP_SIZE[1] + 1}")
            if not cell.right:
                messages.append(f"{j + i*MAP_SIZE[1] + 1}-{j + 1 + i*MAP_SIZE[1] + 1}")
            if not cell.forward:
                messages.append(f"{j + i*MAP_SIZE[1] + 1}-{j + (i+1)*MAP_SIZE[1] + 1}")
            for k, message in enumerate(messages, 0):
                if k != len(messages) - 1:
                    print(message,end="; ")
                else:
                    print(message)

            # print("")
    print("----End map----")



# print(read_qr(get_image()))
# cv2.imshow('a', get_image())
# cv2.waitKey(0)
# exit()

direction = 4 # 0 - increase x, 1 - decrease y, 2 - decrease x, 3 - increase y, 4 - increase z
delta = navigate_wait(x_=0, y_=0, z_=0,yaw_=yaw, frame_id='body', auto_arm=True, speed=0.3)
delta = navigate_wait(x_ = 0,y_=0,z_=z,yaw_=math.radians(yaw) + delta[3], frame_id='body', speed=0.3)

num_qrs = 0
st_time = time.time()
while time.time() - st_time < 10:
    qrs = read_qr(get_image())
    if len(qrs) > 0:
        num_qrs = int(re.findall(r'\d+',qrs[0].data.decode("utf-8"))[0])
        print(num_qrs)
        break



def most_common(lst):
    return max(set(lst), key=lst.count)

def print_report():
    print("----Start report----")
    print(f"Start point. In stock {num_qrs} cargo.")
    i=1
    for st, zones in zip(qr_data.keys(), qr_data.values()):
        print(f"{i}. Zone {most_common(zones)}, Cargo={st}")
        i+=1
    print("----End report----")
    
    

start_time = time.time()
direction = 0

while time.time() - start_time < 420:
    right =  check_right_wall(WALL_DIST)
    left = check_left_wall(WALL_DIST)
    forward = check_forward_wall(WALL_DIST)
    if not right:
        maze_map[round(x/STEP_X)][round(-y/STEP_Y)].right = False
    if not left:
        maze_map[round(x/STEP_X)][round(-y/STEP_Y)].left = False
    if not forward:
        maze_map[round(x/STEP_X)][round(-y/STEP_Y)].forward = False

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
    try:
        x, y, z, yaw, delta = move_body(direction,last_direction, x, y, z, yaw, delta)
    except:
        print("Error")
    if x == 0.0 and y == 0.0:
        break

    


landing()

print_report()
print_maze_map(maze_map)
