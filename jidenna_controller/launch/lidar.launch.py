#!/usr/bin/env python3
"""
YDLidar Prime Node - ROS2 Python implementation using ydlidar SDK
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import ydlidar
import math
import time


class YDLidarNode(Node):
    def __init__(self):
        super().__init__('ydlidar_prime_node')
        
        # Declare parameters
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
        
        # Get parameters
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
        
        # Create publisher
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # Initialize ydlidar
        ydlidar.os_init()
        self.laser = ydlidar.CYdLidar()
        
        # Configure lidar
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, self.port)
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, self.baudrate)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.laser.setlidaropt(ydlidar.LidarPropSampleRate, self.sample_rate)
        self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, self.scan_frequency)
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, self.single_channel)
        self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, self.max_angle)
        self.laser.setlidaropt(ydlidar.LidarPropMinAngle, self.min_angle)
        self.laser.setlidaropt(ydlidar.LidarPropMaxRange, self.max_range)
        self.laser.setlidaropt(ydlidar.LidarPropMinRange, self.min_range)
        
        self.get_logger().info(f"Initializing YDLidar on port {self.port}...")
        
        # Initialize and start
        ret = self.laser.initialize()
        if not ret:
            self.get_logger().error("Failed to initialize lidar!")
            raise RuntimeError("Lidar initialization failed")
        
        self.get_logger().info("Lidar initialized successfully")
        time.sleep(1)  # Important delay
        
        ret = self.laser.turnOn()
        if not ret:
            self.get_logger().error("Failed to start lidar scan!")
            self.laser.disconnecting()
            raise RuntimeError("Failed to start scan")
        
        self.get_logger().info("Lidar scan started")
        
        # Create timer for scanning
        self.scan_timer = self.create_timer(0.1, self.scan_callback)
        
    def scan_callback(self):
        """Process lidar scan and publish LaserScan message"""
        scan = ydlidar.LaserScan()
        
        ret = self.laser.doProcessSimple(scan)
        
        if ret:
            # Create LaserScan message
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = self.frame_id
            
            # Force full 360 degree range (SDK sometimes reports incorrect angles)
            scan_msg.angle_min = math.radians(self.min_angle)  # Use configured value
            scan_msg.angle_max = math.radians(self.max_angle)  # Use configured value
            
            # Calculate angle increment based on number of points
            num_points = scan.points.size()
            if num_points > 0:
                scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / num_points
            else:
                scan_msg.angle_increment = math.radians(scan.config.angle_increment)
            scan_msg.scan_time = scan.config.scan_time
            scan_msg.time_increment = scan.config.time_increment
            scan_msg.range_min = scan.config.min_range
            scan_msg.range_max = scan.config.max_range
            
            # Create range array sized to match angle range and increment
            num_ranges = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
            scan_msg.ranges = [float('inf')] * num_ranges
            scan_msg.intensities = [0.0] * num_ranges
            
            # Fill in the scan data using actual point angles
            for point in scan.points:
                # Calculate index based on point angle relative to angle_min
                index = int((point.angle - scan_msg.angle_min) / scan_msg.angle_increment)
                
                if 0 <= index < num_ranges:
                    scan_msg.ranges[index] = point.range
                    scan_msg.intensities[index] = float(point.intensity)
            
            # Publish the scan
            self.scan_pub.publish(scan_msg)
            
            # Log occasionally
            if self.get_clock().now().nanoseconds % 10000000000 < 100000000:  # Every ~10 seconds
                self.get_logger().info(f"Publishing scan with {scan.points.size()} points")
        else:
            self.get_logger().warn("Failed to get scan data")
    
    def shutdown(self):
        """Cleanup when node is shutting down"""
        self.get_logger().info("Shutting down lidar...")
        self.laser.turnOff()
        self.laser.disconnecting()
        self.get_logger().info("Lidar shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YDLidarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()