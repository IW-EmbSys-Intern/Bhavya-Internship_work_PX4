#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
)


class SquareROS2(Node):

    def __init__(self):
        super().__init__("square_ros2")

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.odom_callback, 10
        )

        self.current_x = 0.0
        self.current_y = 0.0

        # State
        self.counter = 0
        self.state = "STREAM"

        self.wp_index = 0
        self.hover_counter = 0

        # Square waypoints (meters, NED frame)
        self.waypoints = [
            (0.0, 0.0, -50.0),
            (30.0, 0.0, -50.0),
            (30.0, 30.0, -50.0),
            (0.0, 30.0, -50.0),
            (0.0, 0.0, -50.0),
        ]

        # 10 Hz loop
        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info("Square ROS2 Mission Started")

    def odom_callback(self, msg):
        # PX4 odometry is in NED frame
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]

    # ---------------- OFFBOARD MESSAGE ----------------
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
    def publish_setpoint(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [x, y, z]
        msg.yaw = yaw
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

    def compute_yaw_to_waypoint(self, current_x, current_y, target_x, target_y):
        dx = target_x - current_x
        dy = target_y - current_y
        return math.atan2(dy, dx)

    # ---------------- ACTIONS ----------------
    def arm(self):
        self.get_logger().info("ARM command sent")
        self.send_command(400, 1.0)

    def offboard_mode(self):
        self.get_logger().info("OFFBOARD command sent")
        self.send_command(176, 1.0, 6.0)

    # ---------------- LOOP ----------------
    def loop(self):

        # STOP IF DONE
        if self.state == "DONE":
            return

        # ALWAYS STREAM OFFBOARD + SETPOINTS (CRITICAL)
        self.publish_offboard()
        
        if self.wp_index >= len(self.waypoints):
            self.state = "DONE"
            return

        tx, ty, tz = self.waypoints[self.wp_index]
        yaw = self.compute_yaw_to_waypoint(
            self.current_x, self.current_y,
            tx, ty
        )
        print(yaw,'apple')
        self.publish_setpoint(tx, ty, tz, yaw)

        self.counter += 1

        # STEP 1: STREAMING PHASE (PX4 requirement)
        if self.state == "STREAM":

            if self.counter > 30:
                self.get_logger().info("Switching OFFBOARD + ARM")

                self.offboard_mode()
                self.arm()

                self.state = "MISSION"

        # elif self.state == "MISSION":
        #     # wait at waypoint for 2 seconds (10 Hz → 20 cycles)

        #     self.hover_counter += 1

        #     if self.hover_counter <= 60:
        #         return  # keep holding position

        #     # move to next waypoint
        #     self.hover_counter = 0
        #     self.wp_index += 1
            
        #     if self.wp_index >= len(self.waypoints):
        #         return

        #     if self.wp_index < len(self.waypoints):
        #         self.get_logger().info(f"Moving to WP {self.wp_index}")
        #     else:
        #         self.get_logger().info("Square complete → LANDING")
        #         self.send_command(21)  # LAND
        #         self.state = "DONE"

        elif self.state == "MISSION":

            if self.wp_index >= len(self.waypoints):
                self.get_logger().info("Square complete → LANDING")
                self.send_command(21)
                self.state = "LANDING"
                return

            tx, ty, tz = self.waypoints[self.wp_index]

            yaw = self.compute_yaw_to_waypoint(
                self.current_x, self.current_y,
                tx, ty
            )
            print(yaw)
            # waypoint switching logic (time-based hold)
            self.hover_counter += 1

            if self.hover_counter > 60:
                self.hover_counter = 0
                self.wp_index += 1

                if self.wp_index < len(self.waypoints):
                    self.get_logger().info(f"Moving to WP {self.wp_index}")

        elif self.state == "LANDING":
            # Keep publishing offboard briefly OR stop safely
            self.publish_offboard()

            # Optional: keep publishing last position or nothing
            self.get_logger().info("Landing in progress...")
            return

def main():
    rclpy.init()
    node = SquareROS2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
