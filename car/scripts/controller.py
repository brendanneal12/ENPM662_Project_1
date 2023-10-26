#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from matplotlib import pyplot as plt

#Data Logging Variables
TimeVector = []
Ctrl_Inputs = []
Error_Vector = []


class P_Controller(Node): #Define the Command Velocity Publisher Node

    def __init__(self): #Init Function

        super().__init__('p_controller_node') #Naming the Node
        self.get_logger().info(f'Proportional controller node on.') #Confirmation that the Node is Spun Properly
        
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,depth=10)

        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10) #Create a publisher to the wheel velocoty controller
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10) #Create a publisher to the joint position controller

        self.imu_sub = self.create_subscription(Imu, 'imu_plugin/out', self.imu_callback, qos_profile) #Create a subscriber to the 'imu_plugin/out' topic
        self.imu_sub  # Prevent unused variable warning

        timer_period = 0.1 #Pub Frequency
        self.timer = self.create_timer(timer_period, self.timer_callback) #Initialize Timer Callback

        self.counter = 0 #Initialize Counter

    def imu_callback(self, msg): #Define the Callback Function
        imu_timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec)/(1000000000) #Grab the Time of record.
        current_heading = msg.orientation.z #Grab the robot's current yaw (orientation).

        self.timestamp = imu_timestamp #Update the node's timestamp.
        self.current_heading = current_heading #Update the robot's current heading.


    def timer_callback(self): #Defining the Callback Function
        K = 10 #Proportional Controller Gain

        # des_phi = np.deg2rad(45) #Desired phi, but changed due to IMU data.
        des_phi = 0.375 #Adjusted for IMU Errors

        linear_vel=3.0 #Constant Velocity Profile
        steer_angle=0.0 #Initialize steer angle to 0 rad.

        phi_error = des_phi - self.current_heading #Calculate the error between current heading and desired heading.
        steer_angle = -K*phi_error #Generate Control Input

        #Implement joint limits because an error is thrown even with the XACRO file limits.
        if steer_angle>1.0:
            steer_angle=1.0

        if steer_angle<-1.0:
            steer_angle=-1.0

        #Initialize Messages
        wheel_velocities = Float64MultiArray()
        joint_positions = Float64MultiArray()

        #Implement proper stop command based on counts. More detail in report.
        if self.counter < 638:
            wheel_velocities.data = [linear_vel,-linear_vel, linear_vel,-linear_vel]
            joint_positions.data = [steer_angle,steer_angle]
        else:
            wheel_velocities.data = [0.0,0.0, 0.0,0.0]
            joint_positions.data = [0.0,0.0]


        #Publish joint and velocity commands.
        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)

        #Terminal output.
        self.get_logger().info(f'Current Heading (Subscription): {self.current_heading}, Steer Control Input (Publisher): {steer_angle}')

        #Append data for plotting.
        TimeVector.append(self.timestamp)
        Ctrl_Inputs.append(steer_angle)
        Error_Vector.append(phi_error)

        self.counter += 1 #Increase Counter




def main(args = None): #Defining the 'Main' Function
    print('Starting Proportional Control: Moving from (0,0) to (10,10)') #Confirmation that Main Functions Properly
    rclpy.init(args=args)

    Controller = P_Controller() #Establish the Publisher Node

    try:
        rclpy.spin(Controller) #Spin the Publisher Node
    except KeyboardInterrupt:
        pass

    Controller.destroy_node() #Destroy node when Ctrl. C is pressed
    rclpy.shutdown() #Shut the Node Down

    #After Ctl. C, Plot the Results.
    print('End of monitoring: prepare for graphs!')



    ##-------------------Plotting Results---------------------------##
    plt.title('Yaw Error vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Error (rad)')
    plt.plot(TimeVector, Error_Vector, 'b-', label = 'Yaw Error')
    plt.legend()
    plt.show()

    plt.title('Steer Angle vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Steer Angle (rad)')
    plt.plot(TimeVector, Ctrl_Inputs, 'b-', label = 'Steer Angle')
    plt.legend()
    plt.show()



if __name__ == '__main__':
    main()