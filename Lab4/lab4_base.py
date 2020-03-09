import sys
import rospy
import json
import copy
import argparse
import time
import math
import numpy as np
from PIL import Image, ImageTk
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16

if sys.version_info[0] == 2:
  import Tkinter as tk # Python 2
else:
  import tkinter as tk # Python 3

g_namespace = ""

# Driver Constants
CYCLE_TIME = 0.05 # 50ms cycle time
MAP_RESOLUTION = 0.0015 # meters per pixel
MAP_SIZE_X = 1200 # Default map size in pixels
MAP_SIZE_Y = 800 # Default map size in pixels
SPARKI_SIZE_RADIUS = 0.08 # 0.08m radius == 6.29in diameter 
SPARKI_ULTRASONIC_MAX_DIST = .75 # 0.75m max range for ultrasonic sensor

# ***** SERVO POSITIONS ***** #
SPARKI_SERVO_LEFT = 80
SPARKI_SERVO_CENTER = 0
SPARKI_SERVO_RIGHT = -80
SPARKI_SPEED = 0.0278 # 100% speed in m/s
SPARKI_AXLE_DIAMETER = 0.085 # Distance between wheels, meters 
SPARKI_WHEEL_RADIUS = 0.03 # Radius of wheels, meters
SPAKI_VELOCITY=3


# GLOBALS 
pose2d_sparki_odometry = None #Pose2D message object, contains x,y,theta members in meters and radians
#TODO: Track servo angle in radians
#TODO: Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)
#TODO: Create data structure to hold map representation

g_tk_map_resolution = 400
g_tk_map_width = int(MAP_SIZE_X * MAP_RESOLUTION * g_tk_map_resolution)
g_tk_map_height = int(MAP_SIZE_Y * MAP_RESOLUTION * g_tk_map_resolution)
g_tk_map = np.zeros([g_tk_map_height, g_tk_map_width])
g_tk_window = None
g_tk_label = None

# TODO: Use these variables to hold your publishers and subscribers
publisher_motor = None
publisher_odom = None
publisher_ping = None
publisher_servo = None
publisher_render = None
subscriber_odometry = None
subscriber_state = None

# CONSTANTS 
IR_THRESHOLD = 300 # IR sensor threshold for detecting black track. Change as necessary.
CYCLE_HZ = 20 # In seconds
PING_CYCLE_INTERVAL = 2 #execute a ping every 2 cycles (.1 seconds)

def main(args):
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry
    global line_center, line_right, line_left, ping_dist

    #TODO: Init your node to register it with the ROS core
    init(args)
    rate = rospy.Rate(CYCLE_HZ)
    cycleCounter = 0
    lastPingCycle = 0
    while not rospy.is_shutdown():
        #TODID: Implement CYCLE TIME
        
        #TODO: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))
        motorSpeeds = Float32MultiArray()
        #motorSpeeds.data=[1.0, 1.0]
        #print("sensor:                     %f, %f, %f" % (line_left, line_center, line_right))
        if line_center < IR_THRESHOLD and line_left < IR_THRESHOLD and line_right < IR_THRESHOLD:
            # Reset Odometery
            # Move forward
            motorSpeeds.data=[SPAKI_VELOCITY, SPAKI_VELOCITY]
            rospy.loginfo("Loop Closure Triggered")
            originPose = Pose2D(0, 0, 0)
            publisher_odom.publish(originPose)
            pose2d_sparki_odometry = originPose
        elif line_left < IR_THRESHOLD:
            # Turn Left
            motorSpeeds.data=[-1 * SPAKI_VELOCITY, SPAKI_VELOCITY]
        elif line_right < IR_THRESHOLD:
            # Turn right
            motorSpeeds.data=[SPAKI_VELOCITY, -1 * SPAKI_VELOCITY]
        elif line_center < IR_THRESHOLD and line_left > IR_THRESHOLD and line_right > IR_THRESHOLD:
            # Move forward
            motorSpeeds.data=[SPAKI_VELOCITY, SPAKI_VELOCITY]


        publisher_motor.publish(motorSpeeds)




        if(cycleCounter - lastPingCycle == PING_CYCLE_INTERVAL):
            publisher_ping.publish()
            lastPingCycle = cycleCounter
            display_map()

        publisher_render.publish()

        cycleCounter += 1
        rate.sleep()



def init(args):
    global g_namespace
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_render
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry
    global line_center, line_right, line_left
    global ping_dist
    global g_tk_window, g_tk_canvas
    global g_tk_map_width, g_tk_map_height, g_tk_map_resolution, g_tk_map

    line_center = 0
    line_right = 0
    line_left = 0
    ping_dist = 0
    g_namespace = args.namespace
    rospy.init_node("sparki_mapper_%s" % g_namespace)


    g_tk_window = tk.Tk()
    img = ImageTk.PhotoImage('RGB', (g_tk_map_width, g_tk_map_height))
    g_tk_canvas = tk.Canvas(g_tk_window, width=g_tk_map_width, height=g_tk_map_height)
    g_tk_canvas.pack(fill="both", expand="yes")

    
    #TODID: Set up your publishers and subscribers
    subscriber_odometry = rospy.Subscriber("/%s/odometry" % g_namespace, Pose2D, callback_update_odometry)
    subscriber_state = rospy.Subscriber('/%s/state' % g_namespace, String, callback_update_state)

    publisher_motor = rospy.Publisher('/%s/motor_command' % g_namespace, Float32MultiArray, queue_size=10)
    publisher_odom = rospy.Publisher('/%s/set_odometry' % g_namespace, Pose2D, queue_size=10)
    publisher_ping = rospy.Publisher('/%s/ping_command' % g_namespace, Empty, queue_size=10)
    publisher_servo = rospy.Publisher('/%s/set_servo' % g_namespace, Int16, queue_size=10)
    publisher_render = rospy.Publisher('/%s/render_sim' % g_namespace, Empty, queue_size=10) 
    
    rospy.sleep(1)
    #TODID: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    pose2d_sparki_odometry = Pose2D()
    pose2d_sparki_odometry.x, pose2d_sparki_odometry.y, pose2d_sparki_odometry.theta = args.startingpose[0], args.startingpose[1], args.startingpose[2]
    
    #TODID: Set sparki's servo to an angle pointing inward to the map (e.g., 90)
    publisher_servo.publish(SPARKI_SERVO_LEFT)
    publisher_render.publish()
    print("Init ran...")

def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry
    #print(data)
    #TODID: Copy this data into your local odometry variable
    pose2d_sparki_odometry = Pose2D(data.x, data.y, data.theta)
    #print(pose2d_sparki_odometry)

def callback_update_state(data):
    #TODID: Load data into your program's local state variables
    global line_center, line_right, line_left, ping_dist
    global pose2d_sparki_odometry
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    line_center = state_dict['light_sensors'][2]
    line_right = state_dict['light_sensors'][3]
    line_left = state_dict['light_sensors'][1]

    
    tempPing = state_dict.get('ping', -1)
    if(tempPing != -1):
        #copy variables to make sure they are not updated mid-process
        tmpPose = Pose2D(pose2d_sparki_odometry.x, pose2d_sparki_odometry.y, pose2d_sparki_odometry.theta)
        ping_dist = tempPing
        (x_r, y_r) = convert_ultrasonic_to_robot_coords(tmpPose, ping_dist)
        (x_w, y_w) = convert_robot_coords_to_world(tmpPose, x_r, y_r)
        #(x_w, y_w) = convert_ultrasonic_to_world(tmpPose, ping_dist)

        #print("REL: ", x_r, ", ", y_r)
        #print("ABS: ", x_w, ", ", y_w)
        #print("ROB: ", tmpPose.x, ", ", tmpPose.y)

        populate_map_from_ping(x_w, y_w)
    else:
        ping_dist = -1
        
    
def convert_ultrasonic_to_world(robotPose, x_us):
    #TEST REFERENCE
    #slightly better results than chaining `convert_ultrasonic_to_robot_coords` and `convert_robot_coords_to_world`

    global SPARKI_SERVO_LEFT
    x_w, y_w = 0., 0.
    tmpServoRad = (SPARKI_SERVO_LEFT * 3.14159) / 180.0
    actualAngleRad = tmpServoRad + robotPose.theta

    x_w = robotPose.x + x_us * math.cos(actualAngleRad)
    y_w = robotPose.y + x_us * math.sin(actualAngleRad)

    return x_w, y_w


def convert_ultrasonic_to_robot_coords(robotPose, x_us):
    #TODID: Using US sensor reading and servo angle, return value in robot-centric coordinates
    global SPARKI_SERVO_LEFT
    x_r, y_r = 0,0
    tmpServoRad = (SPARKI_SERVO_LEFT * 3.14159) / 180.0

    x_r = x_us * math.cos(tmpServoRad)      #straight ahead
    y_r = x_us * math.sin(tmpServoRad)      #orthogonal
    
    return x_r, y_r

def convert_robot_coords_to_world(robotPose, x_r, y_r):
    #TODID: Using odometry, convert robot-centric coordinates into world coordinates
    x_w, y_w = 0., 0.


    x_w = robotPose.x + (x_r * math.cos(robotPose.theta)) + (y_r * math.sin(robotPose.theta) * -1.0)
    y_w = robotPose.y + (x_r * math.sin(robotPose.theta)) + (y_r * math.cos(robotPose.theta))

    return x_w, y_w


def populate_map_from_ping(x_ping, y_ping):
    global g_tk_window, g_tk_canvas
    global g_tk_map_width, g_tk_map_height, g_tk_map_resolution, g_tk_map

    #TODID: Given world coordinates of an object detected via ping, fill in the corresponding part of the map
    pxlX = int(x_ping * g_tk_map_resolution)
    pxlY = g_tk_map_height - int(y_ping * g_tk_map_resolution)
    g_tk_map[pxlY, pxlX] = 255


def display_map():
    global pose2d_sparki_odometry
    global g_tk_window, g_tk_canvas
    global g_tk_map_width, g_tk_map_height, g_tk_map_resolution, g_tk_map

    robotX = int(pose2d_sparki_odometry.x * g_tk_map_resolution)
    robotY = g_tk_map_height - int(pose2d_sparki_odometry.y * g_tk_map_resolution)
    img = Image.fromarray(g_tk_map).convert('RGB')

    robotRadius = int(g_tk_map_resolution / 100) + 1
    if(robotX > robotRadius and robotY > robotRadius):
        for x in range(robotX - robotRadius, robotX + robotRadius + 1):
            for y in range(robotY - robotRadius, robotY + robotRadius + 1):
                img.putpixel((x, y), (0, 255, 0))

    tkimg = ImageTk.PhotoImage(img, master=g_tk_window)


    #position,  top left datum, image
    g_tk_canvas.create_image(0, 0, anchor="nw", image=tkimg)

    g_tk_window.update_idletasks()
    g_tk_window.update()    


def ij_to_cell_index(i,j):
    #TODO: Convert from i,j coordinates to a single integer that identifies a grid cell
    return 0

def cell_index_to_ij(cell_index):
    #TODO: Convert from cell_index to (i,j) coordinates
    return 0, 0


def cost(cell_index_from, cell_index_to):
    #TODO: Return cost of traversing from one cell to another
    return 0

if __name__ == "__main__":
    default_x, default_y, default_theta = 0.0, 0.0, 0.0

    parser = argparse.ArgumentParser(description="Sparki Simulation Environment")
    parser.add_argument('-n','--namespace', type=str, nargs='?', default='sparki', help='Prepended string for all topics')
    parser.add_argument('-p','--startingpose', nargs=3, default=[default_x, default_y, default_theta], help='Starting x, y, theta of Sparki in world coords')
    args = parser.parse_args()

    main(args)


