#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import numpy as np



class P_Controller(Node): #Define the Command Velocity Publisher Node

    def __init__(self): #Init Function

        super().__init__('p_controller_node') #Naming the Node
        self.get_logger().info(f'Proportional controller node on.') #Confirmation that the Node is Spun Properly

        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        self.imu_sub = self.create_subscription(Imu, 'imu_plugin/out', self.imu_callback, 10) #Subscribe to the 'imu_plugin/out' topic
        self.imu_sub  # Prevent unused variable warning

        timer_period = 0.5 #Pub Frequency
        self.timer = self.create_timer(timer_period, self.timer_callback) #Initialize Timer Callback

    def imu_callback(self, msg): #Define the Callback Function
        imu_timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec)/(1000000000) #Grab the Time of record.
        current_heading = msg.orientation.z

        self.current_heading = current_heading
        self.timestamp = imu_timestamp

        self.get_logger().info(f'Received IMU data at time: {imu_timestamp}') #Confirm the Subscriber has recieved data.


    def timer_callback(self): #Defining the Callback Function
        K1 = 1
        # K2 = 2
        # des_phi = np.deg2rad(45)
        # x_final = 10
        # y_final = 10
        # time2reach = 30

        # x_des = x_final - x_final*(1-(self.timestamp)/time2reach)
        # y_des = y_final - y_final*(1-(self.timestamp)/time2reach)

        # wheel_velocities = Float64MultiArray()
        # joint_positions = Float64MultiArray()

        # linear_vel=0.0
        # steer_angle=0.0

        # phi_error = des_phi - self.current_heading
        # steer_angle = K2*phi_error

        # dist_error = np.sqrt((x_des-CURRENTX)^2 +(y_des-CURRENTY)^2)





        # wheel_velocities.data = [linear_vel,-linear_vel, linear_vel,-linear_vel]
        # joint_positions.data = [steer_angle,steer_angle]

        # self.joint_position_pub.publish(joint_positions)
        # self.wheel_velocities_pub.publish(wheel_velocities)





def main(args = None): #Defining the 'Main' Function
    print('Starting Proportional Control: Moving to (10,10) in 30s') #Confirmation that Main Functions Properly
    rclpy.init(args=args)

    Controller = P_Controller() #Establish the Publisher Node

    try:
        rclpy.spin(Controller) #Spin the Publisher Node
    except KeyboardInterrupt:
        pass

    Controller.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()