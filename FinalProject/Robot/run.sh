x-terminal-emulator -e roscore
x-terminal-emulator -e roslaunch rosbridge_server rosbridge_websocket.launch
x-terminal-emulator -e rosrun turtlesim turtlesim_node
x-terminal-emulator -e python3 TicTacSolver.py
x-terminal-emulator -e python3 TicTacDriver.py
x-terminal-emulator -e "cd ../web && python3 -m http.server 8080"
python3 TicTacCore.py