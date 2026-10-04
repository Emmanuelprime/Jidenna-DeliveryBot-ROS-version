#!/usr/bin/env python3
"""YDLidar ROS 2 node using the YDLidar SDK."""

import math
import time

import rclpy
import ydlidar
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class YDLidarNode(Node):
    def __init__(self):
        super().__init__('ydlidar_prime_node')

        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('scan_frequency', 10.0)
        self.declare_parameter('sample_rate', 3)
        self.declare_parameter('min_angle', -180.0)
        self.declare_parameter('max_angle', 180.0)
        self.declare_parameter('min_range', 0.08)
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('single_channel', True)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.scan_frequency = self.get_parameter('scan_frequency').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.min_angle = self.get_parameter('min_angle').value
        self.max_angle = self.get_parameter('max_angle').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.single_channel = self.get_parameter('single_channel').value

        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        ydlidar.os_init()
        self.laser = ydlidar.CYdLidar()
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, self.port)
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, self.baudrate)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.laser.setlidaropt(
            ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.laser.setlidaropt(ydlidar.LidarPropSampleRate, self.sample_rate)
        self.laser.setlidaropt(
            ydlidar.LidarPropScanFrequency, self.scan_frequency)
        self.laser.setlidaropt(
            ydlidar.LidarPropSingleChannel, self.single_channel)
        self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, self.max_angle)
        self.laser.setlidaropt(ydlidar.LidarPropMinAngle, self.min_angle)
        self.laser.setlidaropt(ydlidar.LidarPropMaxRange, self.max_range)
        self.laser.setlidaropt(ydlidar.LidarPropMinRange, self.min_range)

        self.get_logger().info(f'Initializing YDLidar on port {self.port}...')
        if not self.laser.initialize():
            self.get_logger().error('Failed to initialize lidar!')
            raise RuntimeError('Lidar initialization failed')

        self.get_logger().info('Lidar initialized successfully')
        time.sleep(1)

        if not self.laser.turnOn():
            self.get_logger().error('Failed to start lidar scan!')
            self.laser.disconnecting()
            raise RuntimeError('Failed to start scan')

        self.get_logger().info('Lidar scan started')
        self.scan_timer = self.create_timer(0.1, self.scan_callback)

    def scan_callback(self):
        """Process a lidar scan and publish a LaserScan message."""
        scan = ydlidar.LaserScan()
        if not self.laser.doProcessSimple(scan):
            self.get_logger().warning('Failed to get scan data')
            return

        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id
        scan_msg.angle_min = math.radians(self.min_angle)
        scan_msg.angle_max = math.radians(self.max_angle)

        num_points = scan.points.size()
        if num_points > 0:
            scan_msg.angle_increment = (
                scan_msg.angle_max - scan_msg.angle_min) / num_points
        else:
            scan_msg.angle_increment = math.radians(
                scan.config.angle_increment)

        scan_msg.scan_time = scan.config.scan_time
        scan_msg.time_increment = scan.config.time_increment
        scan_msg.range_min = scan.config.min_range
        scan_msg.range_max = scan.config.max_range

        num_ranges = int(
            (scan_msg.angle_max - scan_msg.angle_min)
            / scan_msg.angle_increment)
        scan_msg.ranges = [float('inf')] * num_ranges
        scan_msg.intensities = [0.0] * num_ranges

        for point in scan.points:
            ros_angle = -point.angle
            index = int(
                (ros_angle - scan_msg.angle_min)
                / scan_msg.angle_increment)
            if 0 <= index < num_ranges:
                scan_msg.ranges[index] = point.range
                scan_msg.intensities[index] = float(point.intensity)

        self.scan_pub.publish(scan_msg)

    def shutdown(self):
        """Stop the lidar and release its connection."""
        self.get_logger().info('Shutting down lidar...')
        self.laser.turnOff()
        self.laser.disconnecting()
        self.get_logger().info('Lidar shutdown complete')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YDLidarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
