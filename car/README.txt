UMD ENPM662 Project 1

Student Information:
Brendan Neal
Directory ID: bneal12.
UID: 119471128.

Adam Lobo
Directory ID: alobo.
UID: 115806078.

Project Information:
Goal: Design and control a mobile robot via ROS2

Package Name: car

Recommended IDE: Visual Studio Code

Python Version: 3

How to Run Code:
SETUP:
1. Create a ROS2 Workspace
2. Download car to the src folder.
3. Build and source the package.

DEBUG LAUNCH:
1. After building and sourcing, in the terminal, run "ros2 launch car debug.launch.py"
2. Configure RVIZ to show robot.

GAZEBO LAUNCH:
1. After building and sourcing, in the terminal, run "ros2 launch car gazebo.launch.py"

DISPLAY LAUNCH:
1. After building and sourcing, in the terminal, run "ros2 launch car display.launch.py"
2. Configure RVIZ to show robot.

TELE-OP SCRIPT:
1. Open the package in your IDE.
2. Go to the launch folder and open spawn_robot_ros2.launch.py.
3. Comment out lines 32-36.
4. Comment in lines 26-30.
5. In one terminal, build, source, then run "ros2 launch car competition.launch.py"
6. Open another terminal, cd to the workspace, source, then run "ros2 run car teleop.py"
7. Follow instructions in the terminal to control the robot!

CONTROLLER SCRIPT:
1. Open the package in your IDE.
2. Go to the launch folder and open spawn_robot_ros2.launch.py.
3. Comment in lines 32-36. (If you ran the teleop script prior to this one).
4. Comment out lines 26-30. (If you ran the teleop script prior to this one).
5. In one terminal, build, source, then run "ros2 launch car gazebo.launch.py"
6. Open another terminal, cd to the workspace, source, then run "ros2 run car controller.py"
7. Observe the robot being controlled via a proportional controller.
8. Once the robot has moved to (10,10), Ctrl. C in the controller window and observe the error graph.
9. Close the error graph and observe the control input graph.


