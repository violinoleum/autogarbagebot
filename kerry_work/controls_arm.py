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
            
            # TODO: check joint states
            for x in range(int(expected_time)):
                self.send_joint_positions()
                rclpy.spin_once(self, timeout_sec=1) # delay
            break

        self.get_logger().info(f"Moved to joint position: {str(desired_joint_positions)}")


# class CameraCommandNode(Node):
#     def __init__(self):
#         super().__init__('camera_command_node')

#         # Camera Subscriber
#         self.camera_subscription = self.create_subscription(
#             DepthCameraSensor,
#             '/depthcamerasensor',  # Make sure this matches your ROS 2 IMU topic name
#             self.camera_callback,
#             10  # QoS (queue size)
#         )
#         self.camera_subscription  # Prevent unused variable warning

#     def camera_callback(self, msg):
#         self.get_logger().info("Camera Data Received:")
#         # self.get_logger().info(f"Linear Acceleration: x={msg.linear_acceleration.x}, y={msg.linear_acceleration.y}, z={msg.linear_acceleration.z}")
#         # self.get_logger().info(f"Angular Velocity: x={msg.angular_velocity.x}, y={msg.angular_velocity.y}, z={msg.angular_velocity.z}")
#         self.linear_acceleration = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
#         self.angular_velocity = [msg.angular_velocity.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
#         self.roll, self.pitch, self.yaw = quaternion_to_euler_np(msg.orientation)
#         self.get_logger().info(f"Angle: roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}")

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandNode()
    first_debug = True
    angle_achieved = False
    double_check = False

    garbage_bin_pos = 0.0
    garbage_bin_depth = 0.8
    garbage_bin_width = 0.45
    grip_width = (1.0 - garbage_bin_width) / 2.0

    print("Starting garbage retrieval...")

    try:
        start_time = time.time()

        while rclpy.ok():
            # rclpy.spin_once(node)

            elapsed_time = time.time() - start_time
            
            aligned_level_position = [garbage_bin_pos, 1.57, 0.0, 2.0, 0.0, 0.0]
            open_grasping_position = [garbage_bin_pos, 1.57, garbage_bin_depth, 2.0, 0.0, 0.0]
            closed_grasping_position = [garbage_bin_pos, 1.57, garbage_bin_depth, 2.0, grip_width, grip_width]
            dumping_position = [garbage_bin_pos, 0.0, 0.8, 2.0, grip_width, grip_width]
            returning_position = [garbage_bin_pos, 1.57, garbage_bin_depth, 2.0, grip_width, grip_width]
            rest_position = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0]
            print("rest_position")
            node.move_to_joint_position(rest_position, 1)
            print("aligned_level_position")
            node.move_to_joint_position(aligned_level_position, 10)
            print("open_grasping_position")
            node.move_to_joint_position(open_grasping_position, 6)
            print("closed_grasping_position")
            node.move_to_joint_position(closed_grasping_position, 6)
            print("dumping_position")
            node.move_to_joint_position(dumping_position, 10)
            print("returning_position")
            node.move_to_joint_position(returning_position, 8)
            print("open_grasping_position")
            node.move_to_joint_position(open_grasping_position, 3)
            print("rest_position")
            node.move_to_joint_position(rest_position, 8)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
