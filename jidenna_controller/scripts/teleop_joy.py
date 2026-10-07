#!/usr/bin/env python3
"""
UDP joystick receiver for jidenna.

Listens on UDP port 4210 for JSON packets from the ESP32 joystick:
    {"t": <ms>, "lx": ..., "ly": ..., "rx": ..., "ry": ...}

Publishes:
    /jidenna_ros_controller/cmd_vel_unstamped  (geometry_msgs/Twist)

Safety:
    If no packet is received for `timeout_s`, publishes zero velocity and
    stops. This matches the diff_drive_controller's own cmd_vel_timeout.
"""

import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class JoystickUdpNode(Node):
    def __init__(self):
        super().__init__("joystick_udp")

        # ---- Parameters ----
        self.declare_parameter("udp_port", 4210)
        self.declare_parameter("listen_address", "0.0.0.0")
        self.declare_parameter("cmd_vel_topic",
                               "/jidenna_ros_controller/cmd_vel_unstamped")
        self.declare_parameter("publish_rate", 50.0)     # Hz
        self.declare_parameter("timeout_s", 0.5)         # stop if no packet
        self.declare_parameter("max_linear", 0.6)        # m/s
        self.declare_parameter("max_angular", 1.5)       # rad/s
        self.declare_parameter("deadzone", 0.05)         # ignore tiny stick values
        self.declare_parameter("invert_linear", False)
        self.declare_parameter("invert_angular", False)

        self.udp_port         = self.get_parameter("udp_port").value
        self.listen_address   = self.get_parameter("listen_address").value
        self.cmd_vel_topic    = self.get_parameter("cmd_vel_topic").value
        self.publish_rate     = self.get_parameter("publish_rate").value
        self.timeout_s        = self.get_parameter("timeout_s").value
        self.max_linear       = self.get_parameter("max_linear").value
        self.max_angular      = self.get_parameter("max_angular").value
        self.deadzone         = self.get_parameter("deadzone").value
        self.invert_linear    = self.get_parameter("invert_linear").value
        self.invert_angular   = self.get_parameter("invert_angular").value

        # ---- State (updated by UDP thread, read by publish timer) ----
        self._lock = threading.Lock()
        self._last_rx_time  = 0.0
        self._linear        = 0.0
        self._angular       = 0.0

        # ---- Publisher ----
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # ---- UDP socket ----
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.listen_address, self.udp_port))
        self.sock.settimeout(0.5)
        self.get_logger().info(
            f"Listening for joystick on UDP {self.listen_address}:{self.udp_port}")

        # ---- Threads ----
        self._stop = threading.Event()
        self.udp_thread = threading.Thread(target=self._udp_loop, daemon=True)
        self.udp_thread.start()

        # ---- Publish timer ----
        period = 1.0 / self.publish_rate
        self.create_timer(period, self._publish_cb)

    # ---------------------------------------------------------------- #
    def _udp_loop(self):
        while not self._stop.is_set():
            try:
                data, _addr = self.sock.recvfrom(512)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                parsed = json.loads(data.decode("utf-8", errors="ignore"))
                rx = float(parsed.get("rx", 0.0))
                ry = float(parsed.get("ry", 0.0))
            except Exception as exc:
                self.get_logger().warn(f"Bad UDP packet: {exc}",
                                       throttle_duration_sec=2.0)
                continue

            # Deadzone
            if abs(rx) < self.deadzone:
                rx = 0.0
            if abs(ry) < self.deadzone:
                ry = 0.0

            # Map to linear/angular
            lin = ry * self.max_linear
            ang = -rx * self.max_angular    # joystick right = CW = negative z

            if self.invert_linear:
                lin = -lin
            if self.invert_angular:
                ang = -ang

            with self._lock:
                self._linear = lin
                self._angular = ang
                self._last_rx_time = time.monotonic()

    # ---------------------------------------------------------------- #
    def _publish_cb(self):
        with self._lock:
            age = time.monotonic() - self._last_rx_time
            lin = self._linear
            ang = self._angular

        # Safety: if no packet received recently, stop.
        if age > self.timeout_s:
            lin = 0.0
            ang = 0.0

        msg = Twist()
        msg.linear.x  = lin
        msg.angular.z = ang
        self.pub.publish(msg)

    # ---------------------------------------------------------------- #
    def destroy_node(self):
        self._stop.set()
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickUdpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()