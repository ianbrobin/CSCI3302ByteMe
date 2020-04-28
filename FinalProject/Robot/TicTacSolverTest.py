# The game grid below is encoded into the following string.
# Positions are marked with either p1, p2, or _ 
# Game state string is prefixed by the current player
#
#   _11_|_12_|_13_
#   _21_|_22_|_23_
#   _31_|_32_|_33_
#
# p1:11,12,13|21,22,23|31,32,33
#
#
#
# Here is an actual game state representation
#   __|p1|p2
#   p2|p1|__
#   __|__|__
#
# p1:_,p1,p2|p2,p1,_|_,_,_
#
#
#
# The calculator should determine the best play position and return the string representation of the cell
# In the above example, the solver should return 32, which will win the game for player 1
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
#SERVICES
svc_GameSolver = ""

testNum = 1
allTestsTrue = True


def caseTest(recv, correct):
  global testNum
  global allTestsTrue
  if (recv == correct):
    print(f"Case {testNum} passed")
  else:
    allTestsTrue = False
    print(f"Case {testNum} failed!!!")
  testNum += 1


def tests(svc_GameSolver):
  global allTestsTrue
  game1 = "p1:p1,p2,p2|_,p1,_|_,_,_"
  game1Sol = "33"
  game1Recv = svc_GameSolver(game1).str
  caseTest(game1Recv, game1Sol)
  print("---------");
  if (allTestsTrue):
    print("All tests passed")
  else:
    print("At least 1 test failed!!!")


def init(args):
  global g_Namespace

  g_Namespace = args.namespace
  rospy.init_node("%s_SolverTest" % g_Namespace)

  #init services
  rospy.wait_for_service('%s_CalculateBestMove' % g_Namespace)
  svc_GameSolver = rospy.ServiceProxy('%s_CalculateBestMove' % g_Namespace, TicTacSolver)
  #response = svc_GameSolver(String(gameState)).str
  #request = "SERVICE INIT"
  #response = svc_GameSolver(request).str
  #print(request, ":", response)
  tests(svc_GameSolver)
    



if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="TicTac_GameCore")
  parser.add_argument('-n','--namespace', type=str, nargs='?', default='TicTac', help='Prepended string for all topics')
  args = parser.parse_args()

  init(args)