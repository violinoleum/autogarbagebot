import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, Image, JointState  # Import both IMU and GPS message types
from geometry_msgs.msg import Twist # Twist message for velocity control
from std_msgs.msg import Float64
import time
import numpy as np
from simple_pid import PID
from pyproj import Proj, Transformer
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

##############################--COMMAND NODE CLASS--##############################

class CommandNode(Node):
    def __init__(self):
        super().__init__('joint_command_node')

        self.joint_publishers = [
            self.create_publisher(Float64, '/joint_1_prismatic_pos', 10),
            self.create_publisher(Float64, '/joint_2_revolute_pos', 10),
            self.create_publisher(Float64, '/joint_3_prismatic_pos', 10),
            self.create_publisher(Float64, '/joint_4_revolute_pos', 10),
            self.create_publisher(Float64, '/joint_5_prismatic_pos', 10),
            self.create_publisher(Float64, '/joint_6_prismatic_pos', 10)
        ]

        self.joint_names = ["link1_to_link2", "link2_to_link3", "link3_to_link4", "link4_to_link6_flange", "gripper_base_to_gripper_left", "gripper_base_to_gripper_right"]
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_tolerances = [0.1, 0.1, 0.1, 0.5, 0.05, 0.05]

        self.cv_bridge = CvBridge()
        self.detection_publisher = self.create_publisher(Image, '/detection_image', 10)
        self.depth_image = None
        self.model = YOLO("garbage_bin_training/best_2_24.pt")

        self.garbage_bin_pos_x = 0.0
        self.garbage_bin_pos_y = 0.0
        self.arm_pos = 0.0
        self.best_confidence = 0.0
        self.garbage_bin_depth = 2.0
        self.arm_depth = 0.8

        self.bin_found = False

        self.rate = self.create_rate(1) # 1 Hz

        # Image Subscriber
        self.image_subscription = self.create_subscription(
            Image,
            '/rgbd_camera/image',
            self.image_callback,
            10  # QoS (queue size)
        )
        self.image_subscription  # Prevent unused variable warning

        # DepthImage Subscriber
        self.depth_image_subscription = self.create_subscription(
            Image,
            '/rgbd_camera/depth_image',
            self.depth_image_callback,
            10  # QoS (queue size)
        )
        self.depth_image_subscription  # Prevent unused variable warning

        # JointState Subscriber
        self.joint_states_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10  # QoS (queue size)
        )
        self.joint_states_subscription  # Prevent unused variable warning

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

        # self.get_logger().info(f'Sent Joint Positions: [{joint_1_msg.data}, {joint_2_msg.data}, {joint_3_msg.data}, {joint_4_msg.data}, {joint_5_msg.data}, {joint_6_msg.data}]')

    def move_to_joint_position(self, desired_joint_positions):
        """Moves to joint position when called."""
        self.get_logger().info(f"Moving to joint position: {str(desired_joint_positions)}")
        while rclpy.ok():
            self.joint_positions = desired_joint_positions

            move_done = True
            for i in range(len(desired_joint_positions)):
                if abs(self.current_joint_positions[i] - desired_joint_positions[i]) > self.joint_tolerances[i]:
                    joint_msg = Float64()
                    joint_msg.data = desired_joint_positions[i]
                    self.get_logger().info(f"Joint {self.joint_names[i]} {self.current_joint_positions[i]} {desired_joint_positions[i]} err: {abs(self.current_joint_positions[i] - desired_joint_positions[i])}")
                    self.joint_publishers[i].publish(joint_msg)
                    move_done = False
                # else: 
                #     self.get_logger().info(f"Joint {self.joint_names[i]} reached target position: {desired_joint_positions[i]}")
            if move_done:
                break
            rclpy.spin_once(self, timeout_sec=1) # delay

        self.get_logger().info(f"Moved to joint position: {str(desired_joint_positions)}")

    def send_detection_image(self, detection_image):
        # self.get_logger().info(f"Sending detection image")
        try:
            self.detection_publisher.publish(self.cv_bridge.cv2_to_imgmsg(detection_image, "rgb8"))
        except CvBridgeError as e:
            print(e)

    def image_callback(self, msg):
        # self.get_logger().info("Camera Image Received")
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        (rows, cols, channels) = cv_image.shape
        # cv2.imshow("ROS Image Window", cv_image)
        # cv2.waitKey(1)

        results = self.model(cv_image, verbose=False)
        # print(results)
        detection_image = cv_image
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
                confidence = box.conf[0]  # Confidence score
                if confidence > self.best_confidence:
                    self.best_confidence = confidence
                    self.garbage_bin_pos_x = (x1 + x2) / 2.0
                    self.garbage_bin_pos_y = (y1 + y2) / 2.0
                    self.arm_pos = (self.garbage_bin_pos_x/320.0) * 2.0 + -1.0
                    print(f"arm_pos updated: {self.arm_pos}")
                    if self.depth_image is not None:
                        self.garbage_bin_depth = self.depth_image[int(self.garbage_bin_pos_y)][int(self.garbage_bin_pos_x)]
                        self.arm_depth = self.garbage_bin_depth - 1.2
                        if self.arm_depth < 0.0:
                            self.arm_depth = 0.0
                        if self.arm_depth > 0.8:
                            self.arm_depth = 0.8
                        print(f"garbage_bin_depth updated: {self.garbage_bin_depth}")
                        print(f"arm_depth updated: {self.arm_depth}")
                    self.bin_found = True

                class_id = int(box.cls[0])  # Class ID
                label = f"{self.model.names[class_id]}: {confidence:.2f}"

                # Draw bounding box
                cv2.rectangle(detection_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                cv2.circle(detection_image, (int(self.garbage_bin_pos_x), int(self.garbage_bin_pos_y)), 3, (255,0,0), -1)
                cv2.putText(detection_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        detected_bins = len(results)
        # cv2.imshow("Detection Image", detection_image)
        # cv2.waitKey(1)

        self.send_detection_image(detection_image)

        self.get_logger().info(f"Detected bins: {detected_bins}")

    def depth_image_callback(self, msg):
        # self.get_logger().info("Camera Depth Image Received")
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, "32FC1")
        except CvBridgeError as e:
            print(e)
        # print(self.depth_image.shape)

    def joint_states_callback(self, msg):
        # print(msg)
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_joint_positions[self.joint_names.index(name)] = msg.position[i]
        # print(self.current_joint_positions)

    def grab_garbage(self):
        garbage_bin_width = 0.45
        grip_width = (1.0 - garbage_bin_width) / 2.0

        while not self.bin_found:
            rclpy.spin_once(self, timeout_sec=0.1)

        print("rest_position")
        rest_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.move_to_joint_position(rest_position)

        time.sleep(0.1)
        print("aligned_horizontal_position")
        aligned_horizontal_position = [self.arm_pos, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.move_to_joint_position(aligned_horizontal_position)

        print("aligned_level_position")
        aligned_level_position = [self.arm_pos, 1.57, 0.0, 2.0, 0.0, 0.0]
        self.move_to_joint_position(aligned_level_position)

        print("open_grasping_position")
        open_grasping_position = [self.arm_pos, 1.57, self.arm_depth, 2.0, 0.0, 0.0]
        self.move_to_joint_position(open_grasping_position)

        print("closed_grasping_position")
        closed_grasping_position = [self.arm_pos, 1.57, self.arm_depth, 2.0, grip_width, grip_width]
        self.move_to_joint_position(closed_grasping_position)

        print("dumping_position")
        dumping_position = [self.arm_pos, 0.0, 0.8, 2.0, grip_width, grip_width]
        self.move_to_joint_position(dumping_position)

        print("returning_position")
        returning_position = [self.arm_pos, 1.57, self.arm_depth, 2.0, grip_width, grip_width]
        self.move_to_joint_position(returning_position)

        print("open_grasping_position")
        open_grasping_position = [self.arm_pos, 1.57, self.arm_depth, 2.0, 0.0, 0.0]
        self.move_to_joint_position(open_grasping_position)

        print("rest_position")
        rest_position = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0]
        self.move_to_joint_position(rest_position)     

        # Reset garbage bin detection
        self.best_confidence = 0.0
        self.garbage_bin_pos_x = 0.0
        self.garbage_bin_pos_y = 0.0
        self.arm_pos = 0.0
        self.bin_found = False

def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    first_debug = True
    angle_achieved = False
    double_check = False

    garbage_bin_width = 0.45
    grip_width = (1.0 - garbage_bin_width) / 2.0

    print("Starting garbage retrieval...")

    try:
        start_time = time.time()

        while rclpy.ok():
            # rclpy.spin_once(node)

            node.grab_garbage()

            # elapsed_time = time.time() - start_time
            # movement_time_start = time.time()
            # print(f"Time taken: {time.time() - movement_time_start} s")

            # movement_time_start = time.time()
            # print("rest_position")
            # rest_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # node.move_to_joint_position(rest_position, 3)
            # print(f"Time taken: {time.time() - movement_time_start} s")

            # movement_time_start = time.time()
            # print("aligned_level_position")
            # aligned_level_position = [node.arm_pos, 1.57, 0.0, 2.0, 0.0, 0.0]
            # node.move_to_joint_position(aligned_level_position, 10)
            # print(f"Time taken: {time.time() - movement_time_start} s")

            # movement_time_start = time.time()
            # print("open_grasping_position")
            # open_grasping_position = [node.arm_pos, 1.57, node.arm_depth, 2.0, 0.0, 0.0]
            # node.move_to_joint_position(open_grasping_position, 6)
            # print(f"Time taken: {time.time() - movement_time_start} s")

            # movement_time_start = time.time()
            # print("closed_grasping_position")
            # closed_grasping_position = [node.arm_pos, 1.57, node.arm_depth, 2.0, grip_width, grip_width]
            # node.move_to_joint_position(closed_grasping_position, 6)
            # print(f"Time taken: {time.time() - movement_time_start} s")

            # movement_time_start = time.time()
            # print("dumping_position")
            # dumping_position = [node.arm_pos, 0.0, 0.8, 2.0, grip_width, grip_width]
            # node.move_to_joint_position(dumping_position, 15)
            # print(f"Time taken: {time.time() - movement_time_start} s")

            # movement_time_start = time.time()
            # print("returning_position")
            # returning_position = [node.arm_pos, 1.57, node.arm_depth, 2.0, grip_width, grip_width]
            # node.move_to_joint_position(returning_position, 8)
            # print(f"Time taken: {time.time() - movement_time_start} s")

            # movement_time_start = time.time()
            # print("open_grasping_position")
            # open_grasping_position = [node.arm_pos, 1.57, node.arm_depth, 2.0, 0.0, 0.0]
            # node.move_to_joint_position(open_grasping_position, 3)
            # print(f"Time taken: {time.time() - movement_time_start} s")

            # movement_time_start = time.time()
            # print("rest_position")
            # rest_position = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0]
            # node.move_to_joint_position(rest_position, 8)
            # print(f"Time taken: {time.time() - movement_time_start} s")

            # # Reset garbage bin detection
            # node.best_confidence = 0.0
            # node.garbage_bin_pos_x = 0.0
            # node.garbage_bin_pos_y = 0.0
            # node.arm_pos = 0.0

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
