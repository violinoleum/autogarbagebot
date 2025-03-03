import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix  # Import both IMU and GPS message types
from geometry_msgs.msg import Twist # Twist message for velocity control
import time

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
        self.get_logger().info(f"Linear Acceleration: x={msg.linear_acceleration.x}, y={msg.linear_acceleration.y}, z={msg.linear_acceleration.z}")
        self.get_logger().info(f"Angular Velocity: x={msg.angular_velocity.x}, y={msg.angular_velocity.y}, z={msg.angular_velocity.z}")
        self.linear_acceleration = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        self.angular_velocity = [msg.angular_velocity.x, msg.linear_acceleration.y, msg.linear_acceleration.z]

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

def main(args=None):
    rclpy.init(args=args)
    node = SensorCommandNode()

    try:
        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(node)

            elapsed_time = time.time() - start_time

            if elapsed_time < 5:
                node.get_logger().info("Moving Forward")
                node.send_velocity(5.0, 0.0)
            elif elapsed_time < 10:
                node.get_logger().info("Moving Backwards")
                node.send_velocity(-5.0, 0.0)
            elif elapsed_time < 15:
                node.get_logger().info("Turning Right")
                node.send_velocity(0.0, -5.0)
            elif elapsed_time < 20:
                node.get_logger().info("Turning Left")
                node.send_velocity(0.0, 5.0)
            else:
                start_time = time.time()

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
