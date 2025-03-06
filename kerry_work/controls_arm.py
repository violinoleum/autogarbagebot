import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix  # Import both IMU and GPS message types
from geometry_msgs.msg import Twist # Twist message for velocity control
from std_msgs.msg import Float64
import time
import numpy as np
from simple_pid import PID
from pyproj import Proj, Transformer

##############################--JOINT CLASS--##############################

class JointCommandNode(Node):
    def __init__(self):
        super().__init__('joint_command_node')

        self.joint_1_publisher = self.create_publisher(Float64, '/joint_1_prismatic_pos', 10)
        self.joint_2_publisher = self.create_publisher(Float64, '/joint_2_revolute_pos', 10)
        self.joint_3_publisher = self.create_publisher(Float64, '/joint_3_prismatic_pos', 10)
        self.joint_4_publisher = self.create_publisher(Float64, '/joint_4_revolute_pos', 10)
        self.joint_5_publisher = self.create_publisher(Float64, '/joint_5_prismatic_pos', 10)
        self.joint_6_publisher = self.create_publisher(Float64, '/joint_6_prismatic_pos', 10)

        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def send_joint_positions(self):
        """Publishes joint commands to Gazebo"""
        joint_1_msg = Float64()
        joint_1_msg.data = self.joint_positions[0]
        joint_2_msg = Float64()
        joint_2_msg.data = self.joint_positions[1]
        joint_3_msg = Float64()
        joint_3_msg.data = self.joint_positions[2]
        joint_4_msg = Float64()
        joint_4_msg.data = self.joint_positions[3]
        joint_5_msg = Float64()
        joint_5_msg.data = self.joint_positions[4]
        joint_6_msg = Float64()
        joint_6_msg.data = self.joint_positions[5]
        
        self.joint_1_publisher.publish(joint_1_msg)
        self.joint_2_publisher.publish(joint_2_msg)
        self.joint_3_publisher.publish(joint_3_msg)
        self.joint_4_publisher.publish(joint_4_msg)
        self.joint_5_publisher.publish(joint_5_msg)
        self.joint_6_publisher.publish(joint_6_msg)

        self.get_logger().info(f'Sent Joint Positions: [{joint_1_msg.data}, {joint_2_msg.data}, {joint_3_msg.data}, {joint_4_msg.data}, {joint_5_msg.data}, {joint_6_msg.data}]')

    def move_to_joint_position(self, desired_joint_positions, expected_time):
        """Moves to joint position when called."""
        self.get_logger().info(f"Moving to joint position: {str(desired_joint_positions)}")
        while rclpy.ok():
            self.joint_positions = desired_joint_positions
            self.send_joint_positions()
            # TODO: check joint states
            rclpy.spin_once(self, timeout_sec=expected_time) # delay
            break

        self.get_logger().info(f"Moved to joint position: {str(desired_joint_positions)}")


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandNode()
    first_debug = True
    angle_achieved = False
    double_check = False

    garbage_bin_pos = 0.0
    garbage_bin_depth = 0.8

    print("Starting garbage retrieval...")

    try:
        start_time = time.time()

        while rclpy.ok():
            # rclpy.spin_once(node)

            elapsed_time = time.time() - start_time
            
            aligned_level_position = [garbage_bin_pos, 1.57, 0.0, 1.57, 0.0, 0.0]
            open_grasping_position = [garbage_bin_pos, 1.57, garbage_bin_depth, 1.57, 0.0, 0.0]
            closed_grasping_position = [garbage_bin_pos, 1.57, garbage_bin_depth, 1.57, 0.4, 0.4]
            dumping_position = [garbage_bin_pos, 0.0, 0.8, 1.57, 0.4, 0.4]
            returning_position = [garbage_bin_pos, 1.57, garbage_bin_depth, 1.57, 0.4, 0.4]
            rest_position = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0]
            print("rest_position")
            node.move_to_joint_position(rest_position, 1)
            print("aligned_level_position")
            node.move_to_joint_position(aligned_level_position, 15)
            print("open_grasping_position")
            node.move_to_joint_position(open_grasping_position, 8)
            print("closed_grasping_position")
            node.move_to_joint_position(closed_grasping_position, 8)
            print("dumping_position")
            node.move_to_joint_position(dumping_position, 15)
            print("returning_position")
            node.move_to_joint_position(returning_position, 15)
            print("open_grasping_position")
            node.move_to_joint_position(open_grasping_position, 5)
            print("rest_position")
            node.move_to_joint_position(rest_position, 15)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
