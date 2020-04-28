#!/usr/bin/env python

# Game grid standard cell notation
#
#   _11_|_12_|_13_
#   _21_|_22_|_23_
#   _31_|_32_|_33_
#
import rospy
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
from TicTac.srv import CalculateBestMove, CalculateBestMoveResponse


def init():
    g_Namespace = ""
    rospy.init_node("solverTest")

    #init services
    rospy.wait_for_service('CalculateBestMove')
    svc_GameSolver = rospy.ServiceProxy('CalculateBestMove', CalculateBestMove)
    #response = svc_GameSolver(String(gameState))

    rospy.sleep(1)
    
    send = String()
    send.data = "ABC"
    print(svc_GameSolver(send))




if __name__ == "__main__":
  init()
