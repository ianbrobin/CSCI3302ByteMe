# Game grid standard cell notation
#
#   _11_|_12_|_13_
#   _21_|_22_|_23_
#   _31_|_32_|_33_
#

import sys

# Had to add this for me and Chandler because our paths get dicked on...
# This fixed "import rospy" error finally after like weeks
if sys.argv[-1] == 'mac':
  sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
    
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
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
from srv._TicTacSolver import TicTacSolver
import string


g_Namespace = ""
g_PlayerToken = ""
g_RobotToken = ""
g_GamestateArr = ["_"] * 9
#SUBS
sub_GameReset = ""
sub_HumanTurnSubmitted = ""
sub_GameCompleted = ""
#PUBS
pub_RobotTurnSubmitted = ""
pub_GameCompleted = ""
#SERVICES
svc_GameSolver = ""


def cellIDToArrIdx(cellID):
    if(cellID == ""):
        print("INVALID MOVE")
        return -1

    row = int(cellID[0]) - 1 #convert to 0 indexed int
    col = int(cellID[1]) - 1 #convert to 0 indexed int
    return (row * 3) + col

def getGameStateStr(gameArr):
    gameStr = ""
    gameStr = gameArr[0] + "," + gameArr[1] + "," + gameArr[2]
    gameStr = gameStr + "|" + gameArr[3] + "," + gameArr[4] + "," + gameArr[5]
    gameStr = gameStr + "|" + gameArr[6] + "," + gameArr[7] + "," + gameArr[8]
    return gameStr

def callback_ResetGame(arg):
    global g_GamestateArr

    #reset stored game state
    g_GamestateArr = ["_"] * 9
    print("Reset Game")


def callback_HumanTurnSubmitted(arg):
    global g_PlayerToken
    global g_RobotToken
    global g_GamestateArr
    global svc_GameSolver
    global pub_RobotTurnSubmitted
    arg = str(arg.data)
    
    #add human move to arr
    humanArrIdx = cellIDToArrIdx(arg)
    g_GamestateArr[humanArrIdx] = g_PlayerToken

    #serialize gamestate to string
    gameStateStr = getGameStateStr(g_GamestateArr)
    gameStateStr = g_RobotToken + ":" + gameStateStr


    #request robot move
    robotMove = svc_GameSolver(gameStateStr).str

    #add robot move to gamestate
    robotArrIdx = cellIDToArrIdx(robotMove)
    g_GamestateArr[robotArrIdx] = g_RobotToken

    #push robot move to msg hub
    pub_RobotTurnSubmitted.publish(robotMove)
    print(arg, " => ", robotMove)




def callback_GameCompleted(arg):
    #display winning player (hmn/rbt)
    print("Game completed! Winner: ", arg.data)



def init(args):
    global g_Namespace
    global g_PlayerToken
    global g_RobotToken
    #SUBS
    global sub_GameReset
    global sub_HumanTurnSubmitted
    global sub_GameCompleted
    #PUBS
    global pub_RobotTurnSubmitted
    global pub_GameCompleted
    #SVC
    global svc_GameSolver

    g_Namespace = args.namespace
    g_PlayerToken = args.player_token
    g_RobotToken = args.robot_token
    rospy.init_node("%s_GameCore" % g_Namespace)

    # init subs
    sub_GameReset = rospy.Subscriber("/%s/GameReset" % g_Namespace, String, callback_ResetGame)
    sub_HumanTurnSubmitted = rospy.Subscriber('/%s/HumanTurnSubmitted' % g_Namespace, String, callback_HumanTurnSubmitted)
    sub_GameCompleted = rospy.Subscriber('/%s/GameCompleted' % g_Namespace, String, callback_GameCompleted)

    #init pubs
    pub_RobotTurnSubmitted = rospy.Publisher('/%s/RobotTurnSubmitted' % g_Namespace, String, queue_size=10)
    pub_GameCompleted = rospy.Publisher('/%s/GameCompleted' % g_Namespace, String, queue_size=10)

    #init services
    rospy.wait_for_service('%s_CalculateBestMove' % g_Namespace)
    svc_GameSolver = rospy.ServiceProxy('%s_CalculateBestMove' % g_Namespace, TicTacSolver)
    #response = svc_GameSolver(String(gameState)).str

    rospy.sleep(1)

    # TODID: Set sparki's servo to an angle pointing inward to the map (e.g., 90)
    print("Init ran...")

    rospy.spin()


    

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="TicTac_GameCore")
  parser.add_argument('-n','--namespace', type=str, nargs='?', default='TicTac', help='Prepended string for all topics')
  parser.add_argument('-pt','--player_token', type=str, nargs='?', default="p1", help='Default player symbol')
  parser.add_argument('-rt','--robot_token', type=str, nargs='?', default="p2", help='Default robot symbol')
  parser.add_argument('-p', '--platform', type=str, nargs='?', default='windows', help='Platform you are running on')
  args = parser.parse_args()

  init(args)
