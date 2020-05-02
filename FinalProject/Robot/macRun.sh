source ~/catkin_ws/devel/setup.bash
x-terminal-emulator -e "roscore"
sleep 2s
x-terminal-emulator -e "roslaunch rosbridge_server rosbridge_websocket.launch"
sleep 1s
x-terminal-emulator -e "rosrun turtlesim turtlesim_node"
x-terminal-emulator -e "python3 TicTacSolver.py"
x-terminal-emulator -e "python3 TicTacDriver.py"
python3 TicTacCore.py --platform mac
sleep 2s
firefox ../Web/index.html
