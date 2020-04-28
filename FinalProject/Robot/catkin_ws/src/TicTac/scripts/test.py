#!/usr/bin/env python

# Game grid standard cell notation
#
#   _11_|_12_|_13_
#   _21_|_22_|_23_
#   _31_|_32_|_33_
#
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


def init(args):
    g_Namespace = ""
    rospy.init_node("solverTest")

    #init services
    rospy.wait_for_service('CalculateBestMove' % g_Namespace)
    svc_GameSolver = rospy.ServiceProxy('CalculateBestMove' % g_Namespace, AddTwoInts)
    #response = svc_GameSolver(String(gameState))

    rospy.sleep(1)

    print(svc_GameSolver("ABC"))




if __name__ == "__main__":
  init(args)