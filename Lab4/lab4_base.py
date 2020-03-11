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
    import Tkinter as tk  # Python 2
else:
    import tkinter as tk  # Python 3

g_namespace = ""

# Driver Constants
CYCLE_TIME = 0.05  # 50ms cycle time
MAP_RESOLUTION = 0.0015  # meters per pixel
MAP_SIZE_X = 1200  # Default map size in pixels
MAP_SIZE_Y = 800  # Default map size in pixels
SPARKI_SIZE_RADIUS = 0.08  # 0.08m radius == 6.29in diameter
SPARKI_ULTRASONIC_MAX_DIST = .75  # 0.75m max range for ultrasonic sensor

# ***** SERVO POSITIONS ***** #
SPARKI_SERVO_LEFT = 80
SPARKI_SERVO_CENTER = 0
SPARKI_SERVO_RIGHT = -80
SPARKI_SPEED = 0.0278  # 100% speed in m/s
SPARKI_AXLE_DIAMETER = 0.085  # Distance between wheels, meters
SPARKI_WHEEL_RADIUS = 0.03  # Radius of wheels, meters
SPAKI_VELOCITY = 3

# GLOBALS
pose2d_sparki_odometry = None  # Pose2D message object, contains x,y,theta members in meters and radians
# TODO: Track servo angle in radians
# TODO: Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)
# TODO: Create data structure to hold map representation

isStarting = True
originPose = Pose2D(0, 0, 0)

g_tk_map_resolution = 30
g_tk_map_render_multiplier = 20
g_tk_map_width = int(MAP_SIZE_X * MAP_RESOLUTION * g_tk_map_resolution * g_tk_map_render_multiplier)
g_tk_map_height = int(MAP_SIZE_Y * MAP_RESOLUTION * g_tk_map_resolution * g_tk_map_render_multiplier)
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
IR_THRESHOLD = 300  # IR sensor threshold for detecting black track. Change as necessary.
CYCLE_HZ = 20  # In seconds
PING_CYCLE_INTERVAL = 2  # execute a ping every 2 cycles (.1 seconds)


def main(args):
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry
    global line_center, line_right, line_left, ping_dist
    global isStarting, originPose

    # TODO: Init your node to register it with the ROS core
    init(args)
    rate = rospy.Rate(CYCLE_HZ)
    cycleCounter = 0
    lastPingCycle = 0

    while not rospy.is_shutdown():
        # TODID: Implement CYCLE TIME

        # TODO: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))
        motorSpeeds = Float32MultiArray()
        # motorSpeeds.data=[1.0, 1.0]
        # print("sensor:                     %f, %f, %f" % (line_left, line_center, line_right))
        if isStarting and originPose.x != 0 and (
                line_center > IR_THRESHOLD or line_left > IR_THRESHOLD or line_right > IR_THRESHOLD):
            isStarting = False

        if line_center < IR_THRESHOLD and line_left < IR_THRESHOLD and line_right < IR_THRESHOLD:
            # Reset Odometery
            # Move forward
            rospy.loginfo("Loop Closure Triggered")
            crrntPose = copy.copy(originPose)
            publisher_odom.publish(crrntPose)
            pose2d_sparki_odometry = crrntPose
            motorSpeeds.data = [SPAKI_VELOCITY, SPAKI_VELOCITY]
        elif line_left < IR_THRESHOLD:
            # Turn Left
            motorSpeeds.data = [-1 * SPAKI_VELOCITY, SPAKI_VELOCITY]
        elif line_right < IR_THRESHOLD:
            # Turn right
            motorSpeeds.data = [SPAKI_VELOCITY, -1 * SPAKI_VELOCITY]
        elif line_center < IR_THRESHOLD and line_left > IR_THRESHOLD and line_right > IR_THRESHOLD:
            # Move forward
            motorSpeeds.data = [SPAKI_VELOCITY, SPAKI_VELOCITY]
        else:
            # forward
            motorSpeeds.data = [SPAKI_VELOCITY, SPAKI_VELOCITY]

        publisher_motor.publish(motorSpeeds)

        if (cycleCounter - lastPingCycle == PING_CYCLE_INTERVAL):
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

    # TODID: Set up your publishers and subscribers
    subscriber_odometry = rospy.Subscriber("/%s/odometry" % g_namespace, Pose2D, callback_update_odometry)
    subscriber_state = rospy.Subscriber('/%s/state' % g_namespace, String, callback_update_state)

    publisher_motor = rospy.Publisher('/%s/motor_command' % g_namespace, Float32MultiArray, queue_size=10)
    publisher_odom = rospy.Publisher('/%s/set_odometry' % g_namespace, Pose2D, queue_size=10)
    publisher_ping = rospy.Publisher('/%s/ping_command' % g_namespace, Empty, queue_size=10)
    publisher_servo = rospy.Publisher('/%s/set_servo' % g_namespace, Int16, queue_size=10)
    publisher_render = rospy.Publisher('/%s/render_sim' % g_namespace, Empty, queue_size=10)

    rospy.sleep(1)
    # TODID: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    pose2d_sparki_odometry = Pose2D()
    pose2d_sparki_odometry.x, pose2d_sparki_odometry.y, pose2d_sparki_odometry.theta = args.startingpose[0], \
                                                                                       args.startingpose[1], \
                                                                                       args.startingpose[2]

    # TODID: Set sparki's servo to an angle pointing inward to the map (e.g., 90)
    publisher_servo.publish(SPARKI_SERVO_LEFT)
    publisher_render.publish()
    print("Init ran...")


def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry
    global isStarting, originPose
    # print(data)
    # TODID: Copy this data into your local odometry variable
    pose2d_sparki_odometry = Pose2D(data.x, data.y, data.theta)
    if (isStarting):
        originPose = copy.copy(data)

    # print(pose2d_sparki_odometry)


def callback_update_state(data):
    # TODID: Load data into your program's local state variables
    global line_center, line_right, line_left, ping_dist
    global pose2d_sparki_odometry
    state_dict = json.loads(data.data)  # Creates a dictionary object from the JSON string received from the state topic
    line_center = state_dict['light_sensors'][2]
    line_right = state_dict['light_sensors'][3]
    line_left = state_dict['light_sensors'][1]

    tempPing = state_dict.get('ping', -1)
    if (tempPing != -1):
        # copy variables to make sure they are not updated mid-process
        tmpPose = Pose2D(pose2d_sparki_odometry.x, pose2d_sparki_odometry.y, pose2d_sparki_odometry.theta)
        ping_dist = tempPing
        # (x_r, y_r) = convert_ultrasonic_to_robot_coords(tmpPose, ping_dist)
        # (x_w, y_w) = convert_robot_coords_to_world(tmpPose, x_r, y_r)
        (x_w, y_w) = convert_ultrasonic_to_world(tmpPose, ping_dist)

        # print("REL: ", x_r, ", ", y_r)
        # print("ABS: ", x_w, ", ", y_w)
        # print("ROB: ", tmpPose.x, ", ", tmpPose.y)

        populate_map_from_ping(x_w, y_w)
    else:
        ping_dist = -1


def convert_ultrasonic_to_world(robotPose, x_us):
    # TEST REFERENCE
    # slightly better results than chaining `convert_ultrasonic_to_robot_coords` and `convert_robot_coords_to_world`

    global SPARKI_SERVO_LEFT
    x_w, y_w = 0., 0.
    tmpServoRad = (SPARKI_SERVO_LEFT * 3.14159) / 180.0
    actualAngleRad = tmpServoRad + robotPose.theta

    x_w = robotPose.x + x_us * math.cos(actualAngleRad)
    y_w = robotPose.y + x_us * math.sin(actualAngleRad)

    return x_w, y_w


def convert_ultrasonic_to_robot_coords(robotPose, x_us):
    # TODID: Using US sensor reading and servo angle, return value in robot-centric coordinates
    global SPARKI_SERVO_LEFT
    x_r, y_r = 0, 0
    tmpServoRad = (SPARKI_SERVO_LEFT * 3.14159) / 180.0

    x_r = x_us * math.cos(tmpServoRad)  # straight ahead
    y_r = x_us * math.sin(tmpServoRad)  # orthogonal

    return x_r, y_r


def convert_robot_coords_to_world(robotPose, x_r, y_r):
    # TODID: Using odometry, convert robot-centric coordinates into world coordinates
    x_w, y_w = 0., 0.

    x_w = robotPose.x + (x_r * math.cos(robotPose.theta)) + (y_r * math.sin(robotPose.theta) * -1.0)
    y_w = robotPose.y + (x_r * math.sin(robotPose.theta)) + (y_r * math.cos(robotPose.theta))

    return x_w, y_w


def populate_map_from_ping(x_ping, y_ping):
    global g_tk_window, g_tk_canvas
    global g_tk_map_width, g_tk_map_height, g_tk_map_resolution, g_tk_map

    # TODID: Given world coordinates of an object detected via ping, fill in the corresponding part of the map

    for x in range(int(g_tk_map_render_multiplier / 2) * -1, int(g_tk_map_render_multiplier / 2) + 1):
        for y in range(int(g_tk_map_render_multiplier / 2) * -1, int(g_tk_map_render_multiplier / 2) + 1):
            pxlX = int(x_ping * g_tk_map_resolution * g_tk_map_render_multiplier + x)
            pxlY = g_tk_map_height - int(y_ping * g_tk_map_resolution * g_tk_map_render_multiplier + y)
            g_tk_map[pxlY, pxlX] = 255


def display_map():
    global pose2d_sparki_odometry
    global g_tk_window, g_tk_canvas
    global g_tk_map_render_multiplier
    global g_tk_map_width, g_tk_map_height, g_tk_map_resolution, g_tk_map

    robotX = int(pose2d_sparki_odometry.x * g_tk_map_resolution * g_tk_map_render_multiplier)
    robotY = g_tk_map_height - int(pose2d_sparki_odometry.y * g_tk_map_resolution * g_tk_map_render_multiplier)
    img = Image.fromarray(g_tk_map).convert('RGB')

    robotRadius = (int(g_tk_map_resolution / 100) + 1) * g_tk_map_render_multiplier
    if (robotX > robotRadius and robotY > robotRadius):
        for x in range(robotX - robotRadius, robotX + robotRadius + 1):
            for y in range(robotY - robotRadius, robotY + robotRadius + 1):
                img.putpixel((x, y), (0, 255, 0))

    tkimg = ImageTk.PhotoImage(img, master=g_tk_window)

    # position,  top left datum, image
    g_tk_canvas.create_image(0, 0, anchor="nw", image=tkimg)

    g_tk_window.update_idletasks()
    g_tk_window.update()


def ij_to_cell_index(i,j):
    global g_tk_map_width, g_tk_map_height, g_tk_map

    """
    Given the matrix with (i, j) points:
    (1, 1), (1, 2), (1, 3)
    (2, 1), (2, 2), (2, 3)
    (3, 1), (3, 2), (3, 3)
    
    we want cell indices like so:
    1     2     3
    4     5     6
    7     8     9
    """

    # i = row, j = column in matrix
    # Add 1 to both of them so that we are indexed starting at 1 instead of 0
    i += 1
    j += 1
    cellIndex = (len(g_tk_map) * (i - 1)) + j  # widthOfMap(i - 1) + j
    return int(cellIndex)

def cell_index_to_ij(cell_index):
    global g_tk_map_width, g_tk_map_height, g_tk_map
    col = cell_index % len(g_tk_map)  # Get the column of the cell index

    # If it goes into our width perfectly, it's the furthest right column...
    if col == 0:
        col = len(g_tk_map)

    row = cell_index - col
    row /= len(g_tk_map)
    rowFinal = row + 1

    # Convert back to 0-indexed
    return int(rowFinal - 1), int(col - 1)


def cost(cell_index_from, cell_index_to):
    # Instead of implementing an actual path-finding algorithm like Dijkstras (lab 5), try using manhattan + diagonal
    # distance and see which has the lowest cost
    # We first see if there is a straight vertical/horizontal path no matter what
    # Then, for each possible scenario (each of the four diagonals), we try to calculate both manhattan + diagonal dist

    global g_tk_map
    # Get (x, y) coordinates of both points
    x1, y1 = cell_index_to_ij(cell_index_from)
    x2, y2 = cell_index_to_ij(cell_index_to)

    # Try going left/right (no change in y axis)
    costHorizontal = 9999
    if y1 == y2:
        costHorizontal = 0
        if x2 > x1:
            for x in range(x1, x2 + 1):
                costHorizontal += 1
                if g_tk_map[x][y1] == 255:
                    costHorizontal = 9999
                    break
        elif x2 < x1:
            for x in range(x2, x1, -1):
                costHorizontal += 1
                if g_tk_map[x][y1] == 255:
                    costHorizontal = 9999
                    break

    # Try going up/down (no change in x axis)
    costVertical = 9999
    if x1 == x2:
        costVertical = 0
        if y2 > y1:
            for y in range(y1, y2 + 1):
                costVertical += 1
                if g_tk_map[x1][y] == 255:
                    costVertical = 9999
                    break
        elif y2 < y1:
            for y in range(y2, y1, -1):
                costVertical += 1
                if g_tk_map[x1][y] == 255:
                    costVertical = 9999
                    break

    # Try going right then up (assuming (0, 0) is top left corner)
    costDiagonal = 0
    cost1 = 0
    if x2 > x1 and y2 < y1:
        # Go right
        for x in range(x1, x2 + 1):
            cost1 += 1
            # If we hit an obstacle...
            if g_tk_map[x][y1] == 255:
                cost1 = 9999
                break
        if cost1 < 9999:
            for y in range(y1, y2, -1):
                cost1 += 1
                if g_tk_map[x2 + 1][y] == 255:
                    cost1 = 9999
                    break

        # Try to get diagonal cost...
        y = y2
        for x in range(x1, x2 + 1):
            if y > y2:
                y -= 1
            costDiagonal += 1
            if g_tk_map[x][y] == 255:
                costDiagonal = 9999
                break

        return min(costHorizontal, costVertical, costDiagonal, cost1)

    # Try going left then up (assuming (0, 0) is top left corner)
    cost2 = 0
    if x2 < x1 and y2 < y1:
        for x in range(x1, x2, -1):
            cost2 += 1
            # If we hit an obstacle...
            if g_tk_map[x][y1] == 255:
                cost2 = 9999
                break
        if cost2 < 9999:
            for y in range(y1, y2, -1):
                cost2 += 1
                if g_tk_map[x1][y] == 255:
                    cost2 = 9999
                    break

        # Try to get diagonal cost...
        y = y2
        costDiagonal = 0
        for x in range(x1, x2, -1):
            if y > y2:
                y -= 1
            costDiagonal += 1
            if g_tk_map[x][y] == 255:
                costDiagonal = 9999
                break

        return min(costHorizontal, costVertical, costDiagonal, cost2)

    # Try going right then down (assuming (0, 0) is top left corner)
    cost3 = 0
    if x2 > x1 and y2 > y1:
        for x in range(x1, x2 + 1):
            cost3 += 1
            # If we hit an obstacle...
            if g_tk_map[x][y1] == 255:
                cost3 = 9999
                break
        if cost3 < 9999:
            for y in range(y1, y2 + 1):
                cost3 += 1
                if g_tk_map[x2 + 1][y] == 255:
                    cost3 = 9999
                    break

        # Try to get diagonal cost...
        costDiagonal = 0
        y = y2
        for x in range(x1, x2 + 1):
            if y < y2:
                y += 1
            costDiagonal += 1
            if g_tk_map[x][y] == 255:
                costDiagonal = 9999
                break

        return min(costHorizontal, costVertical, costDiagonal, cost3)

    # Try going left then down (assuming (0, 0) is top left corner)
    cost4 = 0
    if x2 < x1 and y2 > y1:
        for x in range(x1, x2, -1):
            cost4 += 1
            # If we hit an obstacle...
            if g_tk_map[x][y1] == 255:
                cost4 = 9999
                break
        if cost4 < 9999:
            for y in range(y1, y2 + 1):
                cost4 += 1
                if g_tk_map[x1][y] == 255:
                    cost4 = 9999
                    break

        # Try to get diagonal cost...
        costDiagonal = 0
        y = y2
        for x in range(x1, x2, -1):
            if y < y2:
                y += 1
            costDiagonal += 1
            if g_tk_map[x][y] == 255:
                costDiagonal = 9999
                break

        return min(costHorizontal, costVertical, costDiagonal, cost4)

    return min(costHorizontal, costVertical, cost1, cost2, cost3, cost4, costDiagonal)


# A few tests I wrote to test the cost() function
def mapTests():
    # Generate random map with some random obstacles
    randomMap = np.random.randint(2, size=[len(g_tk_map), len(g_tk_map)]) * 255

    # Generate 2 (x, y) point pairs on our map (with a little padding)
    randX = np.random.randint(low=3, high=len(g_tk_map) - 3)
    randY = np.random.randint(low=3, high=len(g_tk_map) - 3)
    randX2 = np.random.randint(low=3, high=len(g_tk_map) - 3)
    randY2 = np.random.randint(low=3, high=len(g_tk_map) - 3)

    # Set the positions on the map to a unique number so we can visualize start/end positions
    randomMap[randX][randY] = 55
    randomMap[randX2][randY2] = 55

    # Get cell indexes from each (x, y) point pair
    cellIndex1 = ij_to_cell_index(randX, randY)
    cellIndex2 = ij_to_cell_index(randX2, randY2)

    print(f"x/y location 1: {(randX, randY)}\t x/y location 2: {(randX2, randY2)}")
    print(f"Cell Index 1: {cellIndex1}\t Cell Index2: {cellIndex2}")
    print(randomMap)

    # Caluclate cost to travel between cells
    costToTravel = cost(cellIndex1, cellIndex2)

    print(f"Cost to Travel: {costToTravel}")

if __name__ == "__main__":
    default_x, default_y, default_theta = 0.0, 0.0, 0.0

    parser = argparse.ArgumentParser(description="Sparki Simulation Environment")
    parser.add_argument('-n','--namespace', type=str, nargs='?', default='sparki', help='Prepended string for all topics')
    parser.add_argument('-p','--startingpose', nargs=3, default=[default_x, default_y, default_theta], help='Starting x, y, theta of Sparki in world coords')
    args = parser.parse_args()

    main(args)
