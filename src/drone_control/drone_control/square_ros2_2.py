#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy


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

        # Subscriber
        # self.odom_sub = self.create_subscription(
        #     VehicleOdometry,
        #     "/fmu/out/vehicle_odometry",
        #     self.odom_callback,
        #     10
        # )
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.odom_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.odom_callback,
            qos
        )

        # State
        self.state = "STREAM"
        self.wp_index = 0

        # Counters
        self.counter = 0
        self.hover_counter = 0

        # Position (NED)
        self.current_x = 0.0
        self.current_y = 0.0
        
        self.odom_ready = False

        # Waypoints
        self.waypoints = [
            (0.0, 0.0, -5.0),
            (30.0, 0.0, -5.0),
            (30.0, 30.0, -5.0),
            (0.0, 30.0, -5.0),
            (0.0, 0.0, -5.0),
        ]

        # IMPORTANT tuning
        self.wp_tolerance = 3.0

        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info("Square mission started")



    # ---------------- CALLBACK ----------------
    
    def odom_callback(self, msg):
        print(msg)
        try:
            print("ODOM RECEIVED")

            self.current_x = msg.x
            self.current_y = msg.y

            print("x:", self.current_x, "y:", self.current_y)

            if not self.odom_ready:
                self.odom_ready = True
                self.get_logger().info("Odometry ONLINE")

        except Exception as e:
            print("ODOM ERROR:", e)
            
    # ---------------- UTIL ----------------
    def distance_to_wp(self, x, y, tx, ty):
        return math.sqrt((tx - x)**2 + (ty - y)**2)

    # def compute_yaw_to_waypoint(self, x, y, tx, ty):
    #     return math.atan2(ty - y, tx - x)
    
    def compute_yaw_to_waypoint(self, x, y, tx, ty):
        yaw = math.atan2(ty - y, tx - x)
        # self.get_logger().info(f"x={self.current_x}, y={self.current_y}")
        self.get_logger().info(
            f"pos=({self.current_x:.2f}, {self.current_y:.2f})"
        )
        return yaw

    # ---------------- PX4 MSGS ----------------
    def publish_offboard(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [x, y, z]
        msg.yaw = yaw
        self.setpoint_pub.publish(msg)

    def send_command(self, cmd, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.command = cmd
        msg.param1 = p1
        msg.param2 = p2

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.cmd_pub.publish(msg)

    # ---------------- ACTIONS ----------------
    def arm(self):
        self.get_logger().info("ARM")
        self.send_command(400, 1.0)

    def offboard_mode(self):
        self.get_logger().info("OFFBOARD")
        self.send_command(176, 1.0, 6.0)

    # ---------------- MAIN LOOP ----------------
    def loop(self):

        self.publish_offboard()
        
        # 🚨 DO NOT RUN MISSION YET
        if not self.odom_ready:
            self.get_logger().warn("Waiting for odometry...")
            return

        # ---------------- STREAM ----------------
        if self.state == "STREAM":

            tx, ty, tz = self.waypoints[0]

            yaw = self.compute_yaw_to_waypoint(
                self.current_x, self.current_y, tx, ty
            )

            self.publish_setpoint(tx, ty, tz, yaw)

            self.counter += 1

            if self.counter == 20:
                self.offboard_mode()

            if self.counter == 40:
                self.arm()
                self.state = "MISSION"
                self.get_logger().info("ENTER MISSION")

        # ---------------- MISSION ----------------
        elif self.state == "MISSION":

            self.publish_setpoint(
                self.waypoints[self.wp_index][0],
                self.waypoints[self.wp_index][1],
                self.waypoints[self.wp_index][2],
                self.compute_yaw_to_waypoint(
                    self.current_x, self.current_y,
                    self.waypoints[self.wp_index][0],
                    self.waypoints[self.wp_index][1]
                )
            )

            self.hover_counter += 1

            # stable timing-based switching (your proven method)
            if self.hover_counter > 60:

                self.hover_counter = 0
                self.wp_index += 1

                if self.wp_index < len(self.waypoints):
                    self.get_logger().info(f"Moving to WP {self.wp_index}")

                else:
                    self.get_logger().info("Square complete → LANDING")
                    self.send_command(21)
                    self.state = "LANDING"
                    
                    

        # ---------------- HOLD ----------------
        elif self.state == "HOLD":

            tx, ty, tz = self.waypoints[-1]

            self.publish_setpoint(tx, ty, tz, 0.0)

            self.hover_counter += 1

            if self.hover_counter > 30:
                self.get_logger().info("LANDING")

                self.send_command(21)
                self.state = "LANDING"

        # ---------------- LANDING ----------------
        elif self.state == "LANDING":

            self.publish_offboard()
            return


def main():
    rclpy.init()
    node = SquareROS2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
