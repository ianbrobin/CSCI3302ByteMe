x-terminal-emulator -e roscore
x-terminal-emulator -e roslaunch rosbridge_server rosbridge_websocket.launch
x-terminal-emulator -e rosrun turtlesim TicTacTurtle
x-terminal-emulator -e python3 TicTacSolver.py
python3 TicTacCore.py