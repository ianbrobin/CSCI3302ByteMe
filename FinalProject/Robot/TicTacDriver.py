# Game grid standard cell notation
#
#   _11_|_12_|_13_
#   _21_|_22_|_23_
#   _31_|_32_|_33_
#

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


g_Namespace = ""
#SUBS
sub_GameReset = ""
sub_HumanTurnSubmitted = ""
sub_RobotTurnSubmitted = ""
sub_GameCompleted = ""


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="TicTac_GameCore")
  parser.add_argument('-n','--namespace', type=str, nargs='?', default='TicTac', help='Prepended string for all topics')
  parser.add_argument('-pt','--player_token', type=str, nargs='?', default="X", help='Default player symbol')
  parser.add_argument('-rt','--robot_token', type=str, nargs='?', default="O", help='Default robot symbol')
  args = parser.parse_args()

  init()


def init(args):
    global g_Namespace
    #SUBS
    global sub_GameReset
    global sub_HumanTurnSubmitted
    global sub_RobotTurnSubmitted
    global sub_GameCompleted


    g_Namespace = args.namespace
    g_PlayerToken = args.player_token
    g_RobotToken = args.robot_token
    rospy.init_node("%s_TurtleDriver" % g_Namespace)

    # init subs
    sub_GameReset = rospy.Subscriber("/%s/reset" % g_na-mespace, Pose2D, callback_ResetGame)
    sub_HumanTurnSubmitted = rospy.Subscriber('/%s/HumanTurnSubmitted' % g_Namespace, String, callback_HumanTurnSubmitted)
    sub_RobotTurnSubmitted = rospy.Subscriber('/%s/RobotTurnSubmitted' % g_Namespace, String, callback_RobotTurnSubmitted)
    sub_GameCompleted = rospy.Subscriber('/%s/GameCompleted' % g_Namespace, String, callback_GameCompleted)

    rospy.sleep(1)

    # TODID: Set sparki's servo to an angle pointing inward to the map (e.g., 90)
    publisher_servo.publish(SPARKI_SERVO_LEFT)
    publisher_render.publish()
    print("Init ran...")


#arg type = string
#arg data = empty
def callback_ResetGame(arg):
    #clear screen and redraw tic tac grid
    pass


#arg type = string
#arg data = standard notation cell position (ie: 12)
def callback_HumanTurnSubmitted(arg):
    #paint new player token on the tic tac grid
    pass


#arg type = string
#arg data = standard notation cell position (ie: 12)
def callback_RobotTurnSubmitted(arg):
    #paint new player token on the tic tac grid by driving the robot to the position and drawing a shape
    pass


#arg type = string
#arg data = winner name (Human/Robot)
def callback_GameCompleted(arg):
    pass