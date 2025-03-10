import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix  # Import both IMU and GPS message types
from geometry_msgs.msg import Twist # Twist message for velocity control
import time
import numpy as np
from simple_pid import PID
from pyproj import Proj, Transformer

from controls_arm import CommandNode

##############################--GPS--##############################

# Gazebo world's reference position
REF_LAT = -22.986687
REF_LON = -43.202501

# Convert GPS to local X, Y coordinates using UTM
proj = Proj(proj="utm", zone=23, ellps="WGS84", preserve_units=False)  # Zone 23 for Rio de Janeiro
transformer = Transformer.from_proj("epsg:4326", proj)

def gps_to_local(lat, lon):
    """Convert GPS (lat, lon) to local (x, y) ENU coordinates."""
    x, y = transformer.transform(lat, lon)  # Convert to UTM
    ref_x, ref_y = transformer.transform(REF_LAT, REF_LON)  # Convert reference point
    return x - ref_x, y - ref_y  # Return local coordinates relative to origin

def compute_heading(current_x, current_y, goal_x, goal_y):
    """Compute the heading (angle) to reach the goal, normalized to [-180, 180] degrees"""
    # Compute angle from current position to goal
    angle = np.arctan2(goal_y - current_y, goal_x - current_x) * (180.0 / np.pi)
    # Normalize the angle to the range [-180, 180] degrees
    angle = (angle + 180) % 360 - 180  # Ensures the shortest turn direction
    return angle

##############################--IMU--##############################

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

##############################--SENSOR CLASS--##############################

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

        self.angle_pid = PID(0.5, 0.0, 1.0, setpoint=90.0)
        self.angle_pid.output_limits = (-1.0, 1.0)  # Prevent extreme turns

        # GPS Subscriber
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps',  # Make sure this matches your ROS 2 GPS topic name
            self.gps_callback,
            10  # QoS (queue size)
        )
        self.gps_subscription  # Prevent unused variable warning

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_heading = 0.0 # Probably not needed
        
        self.waypoints = [
            (-22.986686843915418,-43.202364958557176), # House 1
            (-22.986688865887537,-43.202225905237064), # House 2
            (-22.986685240914493,-43.20210002069815), # First Corner (turn right)
            (-22.986868062083754,-43.20209806460287), # Second Corner (turn right)
            (-22.98686357765648,-43.20221154858844), # House 3
            (-22.986861961414412,-43.20234522618422), # House 4
            (-22.98686168211502,-43.20246992252178), # Third Corner (left turn)
            (-22.986877587004628,-43.2024629309687), # Third Corner (left turn x2)
            (-22.986885932567255,-43.20211233344233), # Second Corner (turn left)
            (-22.98667249322756,-43.202102736467005), # First Corner (turn left)
            (-22.98665745868786,-43.20221060262935), # House 5
            (-22.986655275783868,-43.20234704773879), # House 6
            (-22.986651959099575,-43.20249602272118)] # Exit
        self.arm_actions = [1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0]
        self.current_waypoint_index = 0

        self.speed_pid = PID(0.8, 0.0, 2.0, setpoint=0.0)
        self.speed_pid.output_limits = (0.0, 5.0) # Speed range (min, max)

    def imu_callback(self, msg):
        # self.get_logger().info("IMU Data Received:")
        # self.get_logger().info(f"Linear Acceleration: x={msg.linear_acceleration.x}, y={msg.linear_acceleration.y}, z={msg.linear_acceleration.z}")
        # self.get_logger().info(f"Angular Velocity: x={msg.angular_velocity.x}, y={msg.angular_velocity.y}, z={msg.angular_velocity.z}")
        self.linear_acceleration = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        self.angular_velocity = [msg.angular_velocity.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        self.roll, self.pitch, self.yaw = quaternion_to_euler_np(msg.orientation)
        # self.get_logger().info(f"Angle: roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}")

    def gps_callback(self, msg):
        # self.get_logger().info("GPS Data Received:")
        # self.get_logger().info(f"Latitude: {msg.latitude}, Longitude: {msg.longitude}, Altitude: {msg.altitude}")
        self.current_x, self.current_y = gps_to_local(msg.latitude, msg.longitude)

    def send_velocity(self, linear_x, angular_z):
        """Publishes velocity commands to Gazebo"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Sent Velocity: linear={msg.linear.x}, angular={msg.angular.z}')

    def compute_signed_distance(self, goal_x, goal_y):
        """Compute signed distance to the goal based on current position and heading."""

        # Compute vector from current position to target
        delta_x = goal_x - self.current_x
        delta_y = goal_y - self.current_y
        distance = np.sqrt(delta_x**2 + delta_y**2)  # Euclidean distance

        # Compute direction to target (angle in degrees)
        target_angle = np.arctan2(delta_y, delta_x) * (180.0 / np.pi)

        # Compute signed angle difference between current heading and target direction
        angle_diff = self.yaw - target_angle  # Yaw is vehicle's current heading

        # Normalize angle to range [-180, 180] (avoids issues with wrap-around)
        angle_diff = (angle_diff + 180) % 360 - 180

        # Determine whether to move forward or backward
        sign = 1 if abs(angle_diff) < 90 else -1  # If overshot, reverse

        return sign * distance  # Positive = move forward, Negative = move backward

    def wait_for_stop(self):
        self.get_logger().info("Waiting for truck to stop")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks

            # Compute total acceleration (excluding gravity)
            total_accel = np.linalg.norm(self.linear_acceleration[:2])  # Ignore Z-axis (gravity)

            # Check if acceleration is below threshold
            if total_accel < 0.02:
                self.get_logger().info("Vehicle has stopped!")
                break

    def turn_to_angle(self, target_angle):
        """Uses PID to smoothly turn to the target angle"""
        self.angle_pid.setpoint = target_angle
        self.get_logger().info(f"Turning to {target_angle} degrees")

        last_error = 0
        overshoot_detected = False
        stable_counter = 0  # Track how long we remain near target

        while rclpy.ok():
            error = self.yaw - target_angle
            correction = self.angle_pid(self.yaw)  # PID correction

            # print(f"Yaw: {self.yaw:.2f}, Target: {target_angle}, Error: {error:.2f}, Correction: {correction:.2f}")

            # Stop condition: If we stay within 0.5° for 25 consecutive loops (~0.5s at 50Hz update)
            if abs(error) < 0.5:
                stable_counter += 1
            else:
                stable_counter = 0  # Reset if error goes up again

            if stable_counter >= 25:
                # self.get_logger().info(f"Reached {target_angle} degrees and stabilized.")
                self.send_velocity(0.0, 0.0)
                break

            # Overshoot Detection: Stop overshooting corrections early
            if last_error * error < 0 and not overshoot_detected:
                # self.get_logger().info("Overshoot detected! Applying final correction.")
                overshoot_detected = True

            self.send_velocity(0.0, correction)  # Only turn
            rclpy.spin_once(self, timeout_sec=0.1)  # Small delay (10Hz update)

            last_error = error

    def move_to_next_waypoint(self):
        """Moves to the next waypoint when called."""
    
        # Check if all waypoints are reached
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached! Stopping.")
            self.send_velocity(0.0, 0.0)
            return

        # Get target waypoint
        goal_lat, goal_lon = self.waypoints[self.current_waypoint_index]
        goal_x, goal_y = gps_to_local(goal_lat, goal_lon)

        # Compute heading to waypoint
        desired_heading = compute_heading(self.current_x, self.current_y, goal_x, goal_y)

        # Call turn_to_angle() to face the waypoint
        self.turn_to_angle(desired_heading)

        # Drive towards waypoint using PID
        while rclpy.ok():
            # Compute distance to target
            distance = self.compute_signed_distance(goal_x, goal_y)
            # self.get_logger().info(f"Distance from target: {distance}")

            # Compute linear velocity using PID
            self.speed_pid.setpoint = distance
            speed = self.speed_pid(-1*distance)  # PID controls speed
            # self.get_logger().info(f"Speed sent: {speed}")

            # Move towards waypoint
            self.send_velocity(speed, 0.0)

            # Stop when close enough
            if distance < 0.2:
                self.send_velocity(0.0, 0.0)
                break

            rclpy.spin_once(self, timeout_sec=0.1) # Small delay (10Hz update)

        # Stop when arrived
        self.send_velocity(0.0, 0.0)
        self.get_logger().info(f"Arrived at waypoint {self.current_waypoint_index}")

    def get_acceleration(self):
        return np.sqrt(self.linear_acceleration[0]**2 + self.linear_acceleration[1]**2)

def main(args=None):
    rclpy.init(args=args)
    node = SensorCommandNode()
    arm_node = CommandNode()
    grabbbing_trash = False

    try:
        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(node)

            elapsed_time = time.time() - start_time

            time.sleep(1)
            if not grabbbing_trash:
                print("Going to next waypoint")
                node.move_to_next_waypoint()
            
            if node.arm_actions[node.current_waypoint_index] == 1: 
                print("Waiting to grab trash")
                node.wait_for_stop()
                print("Starting to grab trash")
                arm_node.grab_garbage()

            node.current_waypoint_index += 1
                

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
