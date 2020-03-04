import rospy
import json
import copy
import argparse
import time
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
line_center = 0
line_right = 0
line_left = 0

# ***** SERVO POSITIONS ***** #
SPARKI_SERVO_LEFT = 80
SPARKI_SERVO_CENTER = 0
SPARKI_SERVO_RIGHT = -80
SPARKI_SPEED = 0.0278 # 100% speed in m/s
SPARKI_AXLE_DIAMETER = 0.085 # Distance between wheels, meters 
SPARKI_WHEEL_RADIUS = 0.03 # Radius of wheels, meters


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

def main(args):
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry

    #TODO: Init your node to register it with the ROS core
    init(args)
    rate = rospy.Rate(CYCLE_HZ)
    while not rospy.is_shutdown():
        #TODO: Implement CYCLE TIME
        
        #TODO: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))
        motorSpeeds = Float32MultiArray()
        motorSpeeds.data=[1.0, 1.0]
        publisher_motor.publish(motorSpeeds)
        
        
        #TODO: Implement loop closure here
        if False:
            rospy.loginfo("Loop Closure Triggered")
        publisher_render.publish()
        rate.sleep()



def init(args):
    global g_namespace
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_render
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry

    g_namespace = args.namespace
    rospy.init_node("sparki_mapper_%s" % g_namespace)


    
    #TODO: Set up your publishers and subscribers
    subscriber_odometry = rospy.Subscriber("/%s/odometry" % g_namespace, Pose2D, callback_update_odometry)
    subscriber_state = rospy.Subscriber('/%s/state' % g_namespace, String, callback_update_state)

    publisher_motor = rospy.Publisher('/%s/motor_command' % g_namespace, Float32MultiArray, queue_size=10)
    publisher_odom = rospy.Publisher('/%s/set_odometry' % g_namespace, Pose2D, queue_size=10)
    publisher_ping = rospy.Publisher('/%s/ping_command' % g_namespace, Empty, queue_size=10)
    publisher_servo = rospy.Publisher('/%s/set_servo' % g_namespace, Int16, queue_size=10)
    publisher_render = rospy.Publisher('/%s/render_sim' % g_namespace, Empty, queue_size=10) 
    
    rospy.sleep(1)
    
    #TODO: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    pose2d_sparki_odometry = Pose2D()
    pose2d_sparki_odometry.x, pose2d_sparki_odometry.y, pose2d_sparki_odometry.theta = args.startingpose[0], args.startingpose[1], args.startingpose[2]
    
    #TODO: Set sparki's servo to an angle pointing inward to the map (e.g., 90)
    publisher_servo.publish(SPARKI_SERVO_LEFT)
    publisher_render.publish()
    print("Init ran...")

def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry
    #TODO: Copy this data into your local odometry variable

def callback_update_state(data):
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    line_center = state_dict['light_sensors'][1]
    line_right = state_dict['light_sensors'][2]
    line_left = state_dict['light_sensors'][3]
    #TODO: Load data into your program's local state variables

def convert_ultrasonic_to_robot_coords(x_us):
    #TODO: Using US sensor reading and servo angle, return value in robot-centric coordinates
    x_r, y_r = 0., 0.
    return x_r, y_r

def convert_robot_coords_to_world(x_r, y_r):
    #TODO: Using odometry, convert robot-centric coordinates into world coordinates
    x_w, y_w = 0., 0.

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
    default_x, default_y = 800 * MAP_RESOLUTION, 656 * MAP_RESOLUTION

    parser = argparse.ArgumentParser(description="Sparki Simulation Environment")
    parser.add_argument('-n','--namespace', type=str, nargs='?', default='sparki', help='Prepended string for all topics')
    parser.add_argument('-p','--startingpose', nargs=3, default=[default_x, default_y, 0.], help='Starting x, y, theta of Sparki in world coords')
    args = parser.parse_args()

    main(args)


