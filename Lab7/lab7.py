import sys
import rospy
from geometry_msgs.msg import PoseStamped
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Move Robot to position")
    parser.add_argument('-x', type=float, nargs=1, default=0.5, help='Goal x location')
    parser.add_argument('-y', type=float, nargs=1, default=0.5, help='Goal y location')
    parser.add_argument('-theta', type=float, nargs=1, default=1.0, help="Goal theta location")
    args = parser.parse_args()
    rospy.init_node("goalPublisher")
    publisher_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    goalpose = PoseStamped()
    goalpose.pose.position.x = args.x
    goalpose.pose.position.y = args.y
    # goalpose.pose.position.z = args.theta
    goalpose.pose.position.z = 0
    publisher_goal.publish(goalpose)
    print(args.x,args.y,args.theta)
    print(goalpose)
