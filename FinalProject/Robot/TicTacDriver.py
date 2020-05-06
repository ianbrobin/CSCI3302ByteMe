# Game grid standard cell notation
#
#   _11_|_12_|_13_
#   _21_|_22_|_23_
#   _31_|_32_|_33_
#

import rospy
import copy
import math
import random
import argparse
import heapq
from heapq import *
from PIL import Image, ImageDraw
import numpy as np
from pprint import pprint
from math import ceil
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32MultiArray, String, Int16
from std_msgs.msg import Empty as msgEmpty
from std_srvs.srv import Empty as srvEmpty
from turtlesim.srv import TeleportAbsolute, SetPen


g_Namespace = ""
#SUBS
sub_GameReset = ""
sub_HumanTurnSubmitted = ""
sub_RobotTurnSubmitted = ""
sub_GameCompleted = ""
sub_Turtle1Pose = ""
#PUBS
pub_Turtle1Command = ""
#SVCS
svc_TurtleClear = ""
svc_Turtle1Pen = ""
svc_Turtle1TeleportAbs = ""

ttl_X = 0.0
ttl_Y = 0.0
ttl_Theta = 0.0

taskQueue = []

#coords
coordDict = {"11":[2.5,8.5],"12":[6.0,8.5],"13":[8.5,8.5],"21":[2.5,5],"22":[6.0,5],"23":[8.5,5],"31":[2.5,2],"32":[6.0,2],"33":[8.5,2]}


def radToDegree(rad):
    return (rad * 180.0) / math.pi

def degreeToRad(deg):
    return (deg * math.pi) / 180.0

def coordDist(x1, x2, y1, y2):
    return math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))


#arg type = string
#arg data = empty
def callback_ResetGame(arg):
    global svc_TurtleClear

    taskQueue.clear()
    #clear screen command?
    print("Drawing board call")

    worker = lambda: (
        svc_TurtleClear(),
        drawBoard()
    )
    taskQueue.append(worker)
    pass



#arg type = string
#arg data = standard notation cell position (ie: 12)
def callback_HumanTurnSubmitted(arg):
    #paint new player token on the tic tac grid
    print("human place move")
    global coordDict
    global taskQueue
    coord = coordDict.get(arg.data)

    worker = lambda: (
        goToSquare(coord[0],coord[1]),
        drawX()
    )
    taskQueue.append(worker)


#arg type = string
#arg data = standard notation cell position (ie: 12)
def callback_RobotTurnSubmitted(arg):
    #paint new player token on the tic tac grid by driving the robot to the position and drawing a shape
    print("robot place move")
    global coordDict
    global taskQueue
    coord = coordDict.get(arg.data)

    worker = lambda: (
        goToSquare(coord[0],coord[1]),
        drawO()
    )
    taskQueue.append(worker)


#arg type = string
#arg data = winner name (Human/Robot)
def callback_GameCompleted(arg):
    pass



def callback_TurtlePose(arg):
    global ttl_X
    global ttl_Y
    global ttl_Theta
    ttl_X = arg.x
    ttl_Y = arg.y
    ttl_Theta = radToDegree(arg.theta)


def disableTurtle1Pen():
    r = 255
    g = 255
    b = 255
    width = 3
    off = 1
    svc_Turtle1Pen(r, g, b, width, off)

def enableTurtle1Pen():
    r = 255
    g = 255
    b = 255
    width = 3
    off = 0
    svc_Turtle1Pen(r, g, b, width, off)

def rotate(angle, clockwise):
    global pub_Turtle1Command
    veloCmd = Twist()
    speed = 70
    PI = 3.1415926535897
    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360

    #We wont use linear components
    veloCmd.linear.x=0
    veloCmd.linear.y=0
    veloCmd.linear.z=0
    veloCmd.angular.x = 0
    veloCmd.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        veloCmd.angular.z = -abs(angular_speed)
    else:
        veloCmd.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    rospy.sleep(.1)
    while(current_angle <= relative_angle):
        pub_Turtle1Command.publish(veloCmd)
        rospy.sleep(.1)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)
        rospy.sleep(.1)
    #Forcing our robot to stop
    veloCmd.angular.z = 0
    pub_Turtle1Command.publish(veloCmd)
    #rospy.spin()

def drawBoard():
    global pub_Turtle1Command
    global svc_Turtle1TeleportAbs

    PI = 3.1415926535897
    veloCmd = Twist()
    #bottom horiz
    disableTurtle1Pen()
    svc_Turtle1TeleportAbs(1, 3, 0 )
    enableTurtle1Pen()
    veloCmd.linear.x=9
    pub_Turtle1Command.publish(veloCmd)
    rospy.sleep(1)
    #top horiz
    disableTurtle1Pen()
    svc_Turtle1TeleportAbs(1, 7, 0)
    enableTurtle1Pen()
    veloCmd.linear.x=9
    pub_Turtle1Command.publish(veloCmd)
    rospy.sleep(1)
    #left vert
    disableTurtle1Pen()
    svc_Turtle1TeleportAbs(4, 1, 90*2*PI/360 )
    enableTurtle1Pen()
    veloCmd.linear.x=9
    pub_Turtle1Command.publish(veloCmd)
    rospy.sleep(1)
    #right vert
    disableTurtle1Pen()
    svc_Turtle1TeleportAbs(7, 1, 90*2*PI/360 )
    enableTurtle1Pen()
    veloCmd.linear.x=9
    pub_Turtle1Command.publish(veloCmd)
    rospy.sleep(1)


def drawX():
    global ttl_X
    global ttl_Y
    global ttl_Theta
    global pub_Turtle1Command

    multFactor = 4.0
    veloCmd = Twist()


    enableTurtle1Pen()
    rotate(ttl_Theta,False)
    veloCmd.linear.x=1 * multFactor
    pub_Turtle1Command.publish(veloCmd)
    rospy.sleep(1 / multFactor)
    #stop
    veloCmd.linear.x=0
    pub_Turtle1Command.publish(veloCmd)

    
    rotate(180,False)
    veloCmd.linear.x=2 * multFactor
    pub_Turtle1Command.publish(veloCmd)
    rospy.sleep(1 / multFactor)
    #stop
    veloCmd.linear.x=0
    pub_Turtle1Command.publish(veloCmd)


    rotate(180,False)
    veloCmd.linear.x=1 * multFactor
    pub_Turtle1Command.publish(veloCmd)
    rospy.sleep(1 / multFactor)
    #stop
    veloCmd.linear.x=0
    pub_Turtle1Command.publish(veloCmd)


    rotate(90,False)
    veloCmd.linear.x=1 * multFactor
    pub_Turtle1Command.publish(veloCmd)
    rospy.sleep(1 / multFactor)
    #stop
    veloCmd.linear.x=0
    pub_Turtle1Command.publish(veloCmd)


    rotate(180,False)
    veloCmd.linear.x=2 * multFactor
    pub_Turtle1Command.publish(veloCmd)
    rospy.sleep(1 / multFactor)
    #stop
    veloCmd.linear.x=0
    pub_Turtle1Command.publish(veloCmd)

def drawO():
    global ttl_X
    global ttl_Y
    global ttl_Theta
    global pub_Turtle1Command

    multFactor = 3.0
    veloCmd = Twist()


    enableTurtle1Pen()
    rotate(abs(ttl_Theta-90),True)
    veloCmd.linear.x=0.5 * multFactor
    pub_Turtle1Command.publish(veloCmd)
    rospy.sleep(1 / multFactor)
    #stop
    veloCmd.linear.x=0
    pub_Turtle1Command.publish(veloCmd)

    enableTurtle1Pen()
    rotate(45,False)
    pub_Turtle1Command.publish(veloCmd)
    rospy.sleep(1 / multFactor)
    #stop
    veloCmd.linear.x=0
    pub_Turtle1Command.publish(veloCmd)

    rotate(90,False)
    veloCmd.linear.x=0.7 * multFactor
    pub_Turtle1Command.publish(veloCmd)
    rospy.sleep(1 / multFactor)
    #stop
    veloCmd.linear.x=0
    pub_Turtle1Command.publish(veloCmd)

    rotate(90,False)
    veloCmd.linear.x=0.7 * multFactor
    pub_Turtle1Command.publish(veloCmd)
    rospy.sleep(1 / multFactor)
    #stop
    veloCmd.linear.x=0
    pub_Turtle1Command.publish(veloCmd)

    rotate(90,False)
    veloCmd.linear.x=0.7 * multFactor
    pub_Turtle1Command.publish(veloCmd)
    rospy.sleep(1 / multFactor)
    #stop
    veloCmd.linear.x=0
    pub_Turtle1Command.publish(veloCmd)



def goToSquare(x,y):
    multFactor = 1.5

    print("GOTO: (", x, ", ", y, ")")
    global ttl_X
    global ttl_Y
    global ttl_Theta
    global pub_Turtle1Command
    print("POS: (", ttl_X, ", ", ttl_Y, ", ", ttl_Theta, ")")
    disableTurtle1Pen()
    veloCmd = Twist()
    distanceBetween = coordDist(x, ttl_X, y, ttl_Y)
    print("DIST: ", distanceBetween)
    theta = radToDegree(math.atan2(y-ttl_Y, x-ttl_X))
    print("ABS THETA: ", theta)
    print("REL THETA: ", theta - ttl_Theta)
    clockwise = False
    if((theta - ttl_Theta) < 0):
        clockwise = True
    
    rotate(abs(theta - ttl_Theta), clockwise)
    veloCmd.linear.x=distanceBetween * multFactor
    pub_Turtle1Command.publish(veloCmd)
    rospy.sleep(1 / multFactor)
    
    #stop
    veloCmd.linear.x=0
    pub_Turtle1Command.publish(veloCmd)


def init(args):
    global g_Namespace
    #SUBS
    global sub_GameReset
    global sub_HumanTurnSubmitted
    global sub_RobotTurnSubmitted
    global sub_GameCompleted
    global sub_Turtle1Pose
    #PUBS
    global pub_Turtle1Command
    #SVCS
    global svc_TurtleClear
    global svc_Turtle1Pen
    global svc_Turtle1TeleportAbs

    global taskQueue


    g_Namespace = args.namespace
    g_PlayerToken = args.player_token
    g_RobotToken = args.robot_token
    rospy.init_node("%s_TurtleDriver" % g_Namespace)

    # init subs
    sub_GameReset = rospy.Subscriber("/%s/GameReset" % g_Namespace, String, callback_ResetGame)
    sub_HumanTurnSubmitted = rospy.Subscriber('/%s/HumanTurnSubmitted' % g_Namespace, String, callback_HumanTurnSubmitted)
    sub_RobotTurnSubmitted = rospy.Subscriber('/%s/RobotTurnSubmitted' % g_Namespace, String, callback_RobotTurnSubmitted)
    sub_GameCompleted = rospy.Subscriber('/%s/GameCompleted' % g_Namespace, String, callback_GameCompleted)
    sub_Turtle1Pose = rospy.Subscriber('/turtle1/pose', Pose, callback_TurtlePose)

    # init pubs
    pub_Turtle1Command = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    #init services
    rospy.wait_for_service('/clear')
    rospy.wait_for_service('/turtle1/teleport_absolute')
    rospy.wait_for_service('/turtle1/set_pen')
    svc_TurtleClear = rospy.ServiceProxy('/clear', srvEmpty)
    svc_Turtle1TeleportAbs = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
    svc_Turtle1Pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)

    #Start of the game
    #drawBoard()
    disableTurtle1Pen()


    while 1==1:
        if(len(taskQueue) == 0):
            rospy.sleep(.5)
            continue
        
        nextTask = taskQueue.pop(0)
        nextTask()





if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="TicTac_GameCore")
  parser.add_argument('-n','--namespace', type=str, nargs='?', default='TicTac', help='Prepended string for all topics')
  parser.add_argument('-pt','--player_token', type=str, nargs='?', default="X", help='Default player symbol')
  parser.add_argument('-rt','--robot_token', type=str, nargs='?', default="O", help='Default robot symbol')
  args = parser.parse_args()

  init(args)
