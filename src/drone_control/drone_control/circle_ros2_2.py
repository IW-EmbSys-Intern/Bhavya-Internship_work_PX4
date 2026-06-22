#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)


class CircleROS2(Node):

    def __init__(self):
        super().__init__('circle_ros2')

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10)

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10)

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10)

        # Circle parameters
        self.radius = 15.0          # meters
        self.angular_speed = 0.3    # rad/s (controls speed)
        self.yaw = 0.0

        self.angle = 0.0

        # Center of circle (IMPORTANT: NED frame)
        self.center_x = 0.0
        self.center_y = 0.0
        self.altitude = -50.0

        # State
        self.counter = 0
        self.state = "STREAM"

        # 10 Hz loop
        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info("Circle Mission Started")

    # ---------------- OFFBOARD ----------------
    def publish_offboard(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    # ---------------- SETPOINT ----------------
    def publish_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [x, y, z]
        msg.yaw = self.yaw
        self.setpoint_pub.publish(msg)

    # ---------------- COMMAND ----------------
    def send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.command = command
        msg.param1 = p1
        msg.param2 = p2

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.cmd_pub.publish(msg)

    def arm(self):
        self.send_command(400, 1.0)

    def offboard_mode(self):
        self.send_command(176, 1.0, 6.0)

    # ---------------- LOOP ----------------
    def loop(self):

        self.publish_offboard()

        # STREAMING PHASE (PX4 requirement)
        if self.state == "STREAM":

            # hold initial position before switching
            self.publish_setpoint(
                self.center_x,
                self.center_y,
                self.altitude
            )

            self.counter += 1

            if self.counter > 30:
                self.get_logger().info("Switching OFFBOARD + ARM")
                self.offboard_mode()
                self.arm()
                self.state = "MISSION"

            return

        # ---------------- CIRCLE MOTION ----------------
        # Increase angle
        self.angle += self.angular_speed * 0.1  # 0.1 = timer period

        x = self.center_x + self.radius * math.cos(self.angle)
        y = self.center_y + self.radius * math.sin(self.angle)
        z = self.altitude

        vx = -self.radius * self.angular_speed * math.sin(self.angle)
        vy =  self.radius * self.angular_speed * math.cos(self.angle)

        self.yaw = math.atan2(vy, vx)

        self.publish_setpoint(x, y, z)


def main():
    rclpy.init()
    node = CircleROS2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()