#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
from TicTac.srv import CalculateBestMove, CalculateBestMoveResponse
import inspect

#arg is string representation of game grid 

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


def calculateBestMoveReturn(arg):
    gameState = arg.input.data
    #inspect.getmembers(MyClass, lambda a:not(inspect.isroutine(a)))
    result = String()
    result.data = gameState
    return result


def init():
    rospy.init_node("GameSolver")
    s = rospy.Service('CalculateBestMove', CalculateBestMove, calculateBestMoveReturn)
    print("Ready to calculate best move")
    rospy.spin()

if __name__ == "__main__":
  init()

