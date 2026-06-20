#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)

class OffboardTakeoff(Node):

    def __init__(self):
        super().__init__('offboard_takeoff')

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode', 10)

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint', 10)

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command', 10)

        self.counter = 0
        self.offboard_enabled = False
        self.armed = False

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Offboard takeoff node started")

    # ---------------- OFFBOARD MODE ----------------
    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(msg)

    # ---------------- SETPOINT ----------------
    def publish_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -5.0]  # 5m takeoff (NED)
        msg.yaw = 0.0
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.setpoint_pub.publish(msg)

    # ---------------- COMMAND ----------------
    def send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.cmd_pub.publish(msg)

    # ---------------- MAIN LOOP ----------------
    def timer_callback(self):

        # 1. ALWAYS stream setpoints first
        self.publish_offboard_mode()
        self.publish_setpoint()

        self.counter += 1

        # 2. Switch to OFFBOARD after a short stream delay
        if self.counter == 20:
            self.get_logger().info("Switching to OFFBOARD mode")

            self.send_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,   # base mode: custom
                6.0    # PX4 OFFBOARD
            )
            self.offboard_enabled = True

        # 3. Arm AFTER OFFBOARD is accepted
        if self.counter == 40:
            self.get_logger().info("Arming vehicle")

            self.send_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0
            )
            self.armed = True

        if self.counter % 20 == 0:
            self.get_logger().info(
                f"Streaming... OFFBOARD={self.offboard_enabled}, ARMED={self.armed}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()