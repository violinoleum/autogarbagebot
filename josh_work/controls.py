import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix  # Import both IMU and GPS message types
from geometry_msgs.msg import Twist # Twist message for velocity control
import time
import numpy as np
from simple_pid import PID

def quaternion_to_euler_np(quat):
    """Convert quaternion to Euler angles using NumPy."""
    x, y, z, w = quat.x, quat.y, quat.z, quat.w

    # Roll (X)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1) * (180.0 / 3.141592653589793)

    # Pitch (Y)
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2) * (180.0 / 3.141592653589793)

    # Yaw (Z)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4) * (180.0 / 3.141592653589793)

    return roll, pitch, yaw

class SensorCommandNode(Node):
    def __init__(self):
        super().__init__('sensor_command_node')

        # IMU Subscriber
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',  # Make sure this matches your ROS 2 IMU topic name
            self.imu_callback,
            10  # QoS (queue size)
        )
        self.imu_subscription  # Prevent unused variable warning

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear_acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0] # Probably not needed
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0 # Used to determine curent angle

        self.pid = PID(0.5, 0.002, 0.2, setpoint=90.0)
        self.pid.output_limits = (-0.5, 0.5)  # Prevent extreme turns

        # GPS Subscriber
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps',  # Make sure this matches your ROS 2 GPS topic name
            self.gps_callback,
            10  # QoS (queue size)
        )
        self.gps_subscription  # Prevent unused variable warning

        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0 # Probably not needed

    def imu_callback(self, msg):
        self.get_logger().info("IMU Data Received:")
        # self.get_logger().info(f"Linear Acceleration: x={msg.linear_acceleration.x}, y={msg.linear_acceleration.y}, z={msg.linear_acceleration.z}")
        # self.get_logger().info(f"Angular Velocity: x={msg.angular_velocity.x}, y={msg.angular_velocity.y}, z={msg.angular_velocity.z}")
        self.linear_acceleration = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        self.angular_velocity = [msg.angular_velocity.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        self.roll, self.pitch, self.yaw = quaternion_to_euler_np(msg.orientation)
        self.get_logger().info(f"Angle: roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}")

    def gps_callback(self, msg):
        self.get_logger().info("GPS Data Received:")
        self.get_logger().info(f"Latitude: {msg.latitude}, Longitude: {msg.longitude}, Altitude: {msg.altitude}")
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude

    def send_velocity(self, linear_x, angular_z):
        """Publishes velocity commands to Gazebo"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent Velocity: linear={msg.linear.x}, angular={msg.angular.z}')

    def turn_to_angle(self, target_angle):
        """Uses PID to smoothly turn to the target angle"""
        self.pid.setpoint = target_angle
        self.get_logger().info(f"Turning to {target_angle} degrees")

        while abs(self.yaw - target_angle) > 0.1 and rclpy.ok():
            correction = self.pid(self.yaw)  # PID correction
            print("Angle correction:", correction)
            self.send_velocity(0.0, correction)  # Only turn
            rclpy.spin_once(self, timeout_sec=0.05)  # Adds delay (50ms)  # Process IMU data

        self.send_velocity(0.0, 0.0)  # Stop when done
        self.get_logger().info(f"Reached {target_angle} degrees")

    def get_acceleration(self):
        return np.sqrt(self.linear_acceleration[0]**2 + self.linear_acceleration[1]**2)

def main(args=None):
    rclpy.init(args=args)
    node = SensorCommandNode()
    first_debug = True

    try:
        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(node)

            elapsed_time = time.time() - start_time

            if elapsed_time < 2:
                pass
            elif elapsed_time < 7:
                node.get_logger().info("Moving Forward")
                node.send_velocity(5.0, 0.0)
            else:
                if first_debug:
                    node.send_velocity(0.0, 0.0)
                    first_debug = False
                if node.get_acceleration() < 0.01:
                    print("Yaw:", node.yaw)
                    node.turn_to_angle(-90)
                

            # if elapsed_time < 5:
            #     node.get_logger().info("Moving Forward")
            #     node.send_velocity(5.0, 0.0)
            # elif elapsed_time < 10:
            #     node.get_logger().info("Moving Backwards")
            #     node.send_velocity(-5.0, 0.0)
            # elif elapsed_time < 15:
            #     node.get_logger().info("Turning Right")
            #     node.send_velocity(0.0, -5.0)
            # elif elapsed_time < 20:
            #     node.get_logger().info("Turning Left")
            #     node.send_velocity(0.0, 5.0)
            # else:
            #     start_time = time.time()

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
