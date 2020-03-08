import rospy
import json
import copy
import argparse
import time
import math
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16

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
PING_CYCLE_INTERVAL = 5 #execute a ping every 5 cycles (.25 seconds)

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
        elif line_left < IR_THRESHOLD:
            # Turn Left
            motorSpeeds.data=[-1 * SPAKI_VELOCITY, SPAKI_VELOCITY]
        elif line_right < IR_THRESHOLD:
            # Turn right
            motorSpeeds.data=[SPAKI_VELOCITY, -1 * SPAKI_VELOCITY]
        elif line_center < IR_THRESHOLD and line_left > IR_THRESHOLD and line_right > IR_THRESHOLD:
            # Move forward
            motorSpeeds.data=[SPAKI_VELOCITY, SPAKI_VELOCITY]
        #TODID: Implement loop closure here
        elif line_center < IR_THRESHOLD and line_left < IR_THRESHOLD and line_right < IR_THRESHOLD:
            rospy.loginfo("Loop Closure Triggered")
            motorSpeeds.data=[SPAKI_VELOCITY, SPAKI_VELOCITY]
            originPose = Pose2D(0, 0, 0)
            publisher_odom.publish(originPose)
            pose2d_sparki_odometry = originPose


        publisher_motor.publish(motorSpeeds)




        if(cycleCounter - lastPingCycle == PING_CYCLE_INTERVAL):
            publisher_ping.publish()
            lastPingCycle = cycleCounter

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
    line_center = 0
    line_right = 0
    line_left = 0
    ping_dist = 0
    g_namespace = args.namespace
    rospy.init_node("sparki_mapper_%s" % g_namespace)


    
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
    print(pose2d_sparki_odometry)

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
        (x_w, y_w) = convert_robot_coords_to_world(tmpPose, x_r ,y_r)

        
        #print("REL: ", x_r, ", ", y_r)
        print("ABS: ", x_w, ", ", y_w)
        #print("ROB: ", tmpPose.x, ", ", tmpPose.y)
    else:
        ping_dist = -1
        

def convert_ultrasonic_to_robot_coords(robotPose, x_us):
    #TODID: Using US sensor reading and servo angle, return value in robot-centric coordinates
    global SPARKI_SERVO_LEFT
    x_r, y_r = 0,0
    tmpThetaRad = (robotPose.theta * 180) / 3.14159    

    x_r = x_us * math.sin(tmpThetaRad)      #straight ahead
    y_r = x_us * math.cos(tmpThetaRad)      #orthogonal
    
    return x_r, y_r

def convert_robot_coords_to_world(robotPose, x_r, y_r):
    #TODID: Using odometry, convert robot-centric coordinates into world coordinates
    x_w, y_w = 0., 0.
    tmpThetaRad = (robotPose.theta * 180) / 3.14159


    #get world position of object based on robot world position and object relative position
    #robot coordinate frame is orthogonal to world coordindate frame
    #   so cross product and dot product functions are reversed
    
    x_w = robotPose.x + x_r * math.cos(robotPose.theta) + (y_r * math.sin(robotPose.theta))
    y_w = robotPose.y + x_r * math.sin(robotPose.theta) + (y_r * math.cos(robotPose.theta) * -1)


    return x_w, y_w

def populate_map_from_ping(x_ping, y_ping):
    #TODO: Given world coordinates of an object detected via ping, fill in the corresponding part of the map
    pass

def display_map():
    #TODO: Display the map
    pass

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


