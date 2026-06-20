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

        self.counter = 0

        # 10 Hz loop (required for OFFBOARD)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("PX4 OFFBOARD Takeoff Node Started")

    # ---------------------------
    # Required OFFBOARD heartbeat
    # ---------------------------
    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(msg)

    # ---------------------------
    # Takeoff setpoint (NED frame)
    # ---------------------------
    def publish_setpoint(self):
        msg = TrajectorySetpoint()

        # NED: negative Z = UP
        msg.position = [0.0, 0.0, -5.0]
        msg.yaw = 0.0

        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.setpoint_pub.publish(msg)

    # ---------------------------
    # PX4 command helper
    # ---------------------------
    def send_command(self, command, param1=0.0, param2=0.0):

        msg = VehicleCommand()

        msg.command = command
        msg.param1 = param1
        msg.param2 = param2

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        self.cmd_pub.publish(msg)

    # ---------------------------
    # MAIN LOOP
    # ---------------------------
    def timer_callback(self):

        # ALWAYS stream first (CRITICAL for PX4 OFFBOARD)
        self.publish_offboard_mode()
        self.publish_setpoint()

        self.counter += 1

        # Step 1: let PX4 accept streaming setpoints
        if self.counter == 30:
            self.get_logger().info("Switching to OFFBOARD mode")

            self.send_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )

        # Step 2: arm vehicle
        if self.counter == 60:
            self.get_logger().info("Arming vehicle")

            self.send_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0
            )

        # Step 3: status
        if self.counter % 20 == 0:
            self.get_logger().info("Streaming OFFBOARD setpoints...")


def main(args=None):
    rclpy.init(args=args)

    node = OffboardTakeoff()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()