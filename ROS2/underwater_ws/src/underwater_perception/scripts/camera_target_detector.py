#!/usr/bin/env python3
"""OpenCV-based Underwater Target Detector Node (Anggota 3).

Subscribes to `/camera/image_raw` and `/odom`, detects the blue Hider AUV
using HSV color segmentation + contour filtering, transforms pixel coordinates
to approximate world coordinates, and publishes `/target/position` and `/target/status`.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

try:
    import cv2
    from cv_bridge import CvBridge
    HAS_CV = True
except ImportError:
    HAS_CV = False


class CameraTargetDetectorNode(Node):
    def __init__(self):
        super().__init__('camera_target_detector')
        self.bridge = CvBridge() if HAS_CV else None

        self.seeker_pos = np.array([0.0, 0.0, -10.0])
        self.seeker_yaw = 0.0
        self.has_odom = False

        self.img_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.target_pub = self.create_publisher(PoseStamped, '/target/position', 10)
        self.status_pub = self.create_publisher(String, '/target/status', 10)

        self.get_logger().info(
            f'CameraTargetDetectorNode initialized (OpenCV available: {HAS_CV}).')

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.seeker_pos = np.array([p.x, p.y, p.z])
        self.seeker_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.has_odom = True

    def image_callback(self, msg: Image):
        status_msg = String()
        if not HAS_CV:
            status_msg.data = 'SEARCHING'
            self.status_pub.publish(status_msg)
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Blue Hider AUV HSV mask (tuned for underwater cyan-blue background)
        lower_blue = np.array([95, 110, 70])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            status_msg.data = 'SEARCHING'
            self.status_pub.publish(status_msg)
            return

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < 120.0:
            status_msg.data = 'SEARCHING'
            self.status_pub.publish(status_msg)
            return

        x, y, w, h = cv2.boundingRect(largest)
        cx = x + w / 2.0
        cy = y + h / 2.0

        # Pinhole camera depth & bearing approximation (640x480, HFOV=80 deg)
        img_h, img_w = frame.shape[:2]
        fx = (img_w / 2.0) / math.tan(math.radians(40.0))
        est_range = max(1.5, min(25.0, 220.0 / math.sqrt(area)))
        az = math.atan2((img_w / 2.0) - cx, fx)
        el = math.atan2((img_h / 2.0) - cy, fx)

        bx = est_range * math.cos(el) * math.cos(az)
        by = est_range * math.cos(el) * math.sin(az)
        bz = est_range * math.sin(el)

        cyaw = math.cos(self.seeker_yaw)
        syaw = math.sin(self.seeker_yaw)
        wx = self.seeker_pos[0] + cyaw * bx - syaw * by
        wy = self.seeker_pos[1] + syaw * bx + cyaw * by
        wz = self.seeker_pos[2] + bz

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x = float(wx)
        pose.pose.position.y = float(wy)
        pose.pose.position.z = float(wz)
        pose.pose.orientation.w = 1.0
        self.target_pub.publish(pose)

        if est_range <= 3.0:
            status_msg.data = 'FOUND'
        elif est_range <= 18.0:
            status_msg.data = 'APPROACHING'
        else:
            status_msg.data = 'DETECTED'
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraTargetDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
