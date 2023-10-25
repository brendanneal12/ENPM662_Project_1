#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
import sys
import select
import tty
import termios
#from pynput import keyboard


class IMU_Sub(Node): #Define the IMU Subscriber Node

    def __init__(self): #Init Function
        super().__init__("imu_sub") #Name the Node
        self.get_logger().info(f'IMU subscription on.') #Confirm the Node is Spun Properly

        self.subscription = self.create_subscription(Imu, 'imu_plugin/out', self.callback, 10) #Subscribe to the 'odom' topic
        self.subscription  # Prevent unused variable warning

    def callback(self, msg): #Define the Callback Function
        timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec)/(1000000000) #Grab the Time of record.
        current_heading = msg.orientation.z

        self.get_logger().info(f'Received imu data at time: {timestamp}') #Confirm the Subscriber has recieved data.

class TF_Sub(Node): #Define the TF Subscriber Node

    def __init__(self): #Init Function
        super().__init__("TF_sub") #Name the Node
        self.get_logger().info(f'TF subscription on.') #Confirm the Node is Spun Properly

        self.subscription = self.create_subscription(Transform, 'tf', self.callback, 10)
        self.subscription  # Prevent unused variable warning

    def callback(self, msg): #Define the Callback Function
        timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec)/(1000000000) #Grab the Time of record.
        self.get_logger().info(f'Received position data at time: {timestamp}') #Confirm the Subscriber has recieved data.




# def main(args = None): #Defining the 'Main' Function
#     print('Starting OL Control: Move TurtleBot3 1m in 10s.') #Confirmation that Main Functions Properly
#     rclpy.init(args=args)

#     cmd_vel_pub = CV_Publisher() #Establish the Publisher Node

#     try:
#         rclpy.spin(cmd_vel_pub) #Spin the Publisher Node
#     except KeyboardInterrupt:
#         pass

#     cmd_vel_pub.destroy_node()
#     rclpy.shutdown()



# if __name__ == '__main__':
#     main()