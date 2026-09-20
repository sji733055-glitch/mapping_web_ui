#!/usr/bin/env python3
"""Publish a short synthetic room scan for hardware-free integration checks."""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class SyntheticMappingSource(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_mapping_source")
        self.cloud_publisher = self.create_publisher(
            PointCloud2, "/cloud_registered", qos_profile_sensor_data
        )
        self.odom_publisher = self.create_publisher(Odometry, "/Odometry", qos_profile_sensor_data)

    @staticmethod
    def room_points(step: int) -> np.ndarray:
        rng = np.random.default_rng(2027 + step)
        z = np.linspace(0.08, 1.45, 30, dtype=np.float32)
        along_x = np.linspace(-3.8, 3.8, 150, dtype=np.float32)
        along_y = np.linspace(-2.8, 2.8, 112, dtype=np.float32)
        wall_x, wall_z_x = np.meshgrid(along_x, z)
        wall_y, wall_z_y = np.meshgrid(along_y, z)
        walls = [
            np.column_stack((wall_x.ravel(), np.full(wall_x.size, -2.8), wall_z_x.ravel())),
            np.column_stack((wall_x.ravel(), np.full(wall_x.size, 2.8), wall_z_x.ravel())),
            np.column_stack((np.full(wall_y.size, -3.8), wall_y.ravel(), wall_z_y.ravel())),
            np.column_stack((np.full(wall_y.size, 3.8), wall_y.ravel(), wall_z_y.ravel())),
        ]
        obstacle_z = np.linspace(0.08, 1.1, 24, dtype=np.float32)
        angle = np.linspace(0, 2 * math.pi, 100, endpoint=False, dtype=np.float32)
        obstacle_a, obstacle_h = np.meshgrid(angle, obstacle_z)
        cylinder = np.column_stack((
            1.2 + 0.42 * np.cos(obstacle_a.ravel()),
            0.4 + 0.42 * np.sin(obstacle_a.ravel()),
            obstacle_h.ravel(),
        ))
        points = np.concatenate(walls + [cylinder]).astype(np.float32)
        points += rng.normal(0.0, 0.006, points.shape).astype(np.float32)
        points[:, 0] += 0.015 * step
        return points

    def publish_scan(self, step: int) -> None:
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id="odom")
        self.cloud_publisher.publish(point_cloud2.create_cloud_xyz32(header, self.room_points(step)))
        odom = Odometry()
        odom.header = header
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = 0.12 * step
        odom.pose.pose.position.y = 0.04 * math.sin(step * 0.5)
        odom.pose.pose.orientation.w = 1.0
        self.odom_publisher.publish(odom)


def main() -> None:
    rclpy.init()
    node = SyntheticMappingSource()
    try:
        for step in range(12):
            node.publish_scan(step)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.08)
        time.sleep(0.3)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
