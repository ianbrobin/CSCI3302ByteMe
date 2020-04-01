import sys
import rospy
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Move Robot to position")
    parser.add_argument('-x', type=float, nargs=1, default=0.5, help='Goal x location')
    parser.add_argument('-y', type=float, nargs=1, default=0.5, help='Goal y location')
    parser.add_argument('-theta', type=float, nargs=1, default=1.0, help="Goal theta location")
    args = parser.parse_args()

    print(args)
    print(args.x)
