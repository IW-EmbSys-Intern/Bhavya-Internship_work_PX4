import rclpy
from rclpy.node import Node
import math

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleOdometry,
    VehicleCommand
)

class WaypointTrajectoryNode(Node):

    def __init__(self):
        super().__init__('waypoint_trajectory_node')

        self.offboard_counter = 0
        self.state = "INIT"

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.sp_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # Subscriber
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos
        )

        self.dt = 0.05

        self.state_start_time = self.get_clock().now()

        # ---------------- WAYPOINTS ----------------
        self.max_cmd_speed = 15.0
        self.vx_smooth = 0.0
        self.vy_smooth = 0.0
        self.vz_smooth = 0.0
        self.alpha = 0.2   # smoothing factor
        self.ref_set = False
        self.prev_pos = None
        self.next_pos = None

        self.waypoints = [
            {"pos": [0, 0, -15], "vmax": 3.0, "hold": 3},
            {"pos": [30, -20, -15], "vmax": 5.0, "hold": 5},
            {"pos": [100, 30, -15], "vmax": 10.0, "hold": 5},
            {"pos": [30, 80, -15], "vmax": 15.0, "hold": 5},
        ]

        self.land_z = 0.0

        self.timer = self.create_timer(self.dt, self.control_loop)

    # ---------------- CALLBACK ----------------
    def odom_callback(self, msg):
        self.position = [
            msg.position[0],
            msg.position[1],
            msg.position[2]
        ]

        if not self.ref_set:
            self.ref_set = True
            self.get_logger().info("Takeoff reference locked")

    # ---------------- OFFBOARD ----------------
    def publish_offboard(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    # ---------------- VELOCITY ----------------
    def publish_velocity(self, vx, vy, vz, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float(yaw)   # ✅ ADD THIS USAGE
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.sp_pub.publish(msg)

    def speed_profile(self, dist, vmax, total_dist): #Smooth acceleration + deceleration using simple triangular profile
        if total_dist < 1e-3:
            return 0.0

        ratio = dist / total_dist

        # acceleration phase (far from goal)
        if ratio > 0.5:
            return vmax * (1 - ratio) * 2  # ramps up

        # deceleration phase (close to goal)
        else:
            return vmax * ratio * 2  # ramps down

    # ---------------- POSITION ----------------
    def publish_position(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.sp_pub.publish(msg)

    # ---------------- UTIL ----------------
    def distance(self, a, b):
        return math.sqrt(
            (a[0] - b[0])**2 +
            (a[1] - b[1])**2 +
            (a[2] - b[2])**2
        )

    # ---------------- CONTROL ----------------
    def control_loop(self):

        self.publish_offboard()

        # ================= INIT =================
        if self.state == "INIT":

            self.publish_position(*self.position)

            self.offboard_counter += 1

            if self.offboard_counter == 20:
                self.arm()

            if self.offboard_counter == 40:
                self.set_offboard()

            if self.offboard_counter > 60:
                self.state = "TAKEOFF"

        # ================= TAKEOFF =================
        elif self.state == "TAKEOFF":

            target = self.waypoints[0]["pos"]
            self.publish_position(*target)

            if self.distance(self.position, target) < 0.5:
                self.get_logger().info("Takeoff complete → WP1")
                self.state = "WP1"

        # ================= WAYPOINTS (VELOCITY CONTROL) =================
        elif self.state in ["WP1", "WP2", "WP3", "WP4"]:

            idx = int(self.state[-1]) - 1
            wp = self.waypoints[idx]

            target = wp["pos"]
            vmax = wp["vmax"]

            direction = [
                target[0] - self.position[0],
                target[1] - self.position[1],
                target[2] - self.position[2],
            ]

            dist = max(math.sqrt(sum(d*d for d in direction)), 1e-3)

            # clamp requested speed
            speed = min(vmax, self.max_cmd_speed)

            # normalize + apply speed
            vx = direction[0] / dist * speed
            vy = direction[1] / dist * speed
            vz = direction[2] / dist * speed

            # ✅ ADD YAW (face direction of motion)
            yaw = math.atan2(direction[1], direction[0])

            # smooth velocity (prevents PX4 saturation issues)
            self.vx_smooth = self.vx_smooth + self.alpha * (vx - self.vx_smooth)
            self.vy_smooth = self.vy_smooth + self.alpha * (vy - self.vy_smooth)
            self.vz_smooth = self.vz_smooth + self.alpha * (vz - self.vz_smooth)

            self.publish_velocity(self.vx_smooth, self.vy_smooth, self.vz_smooth, yaw)

            if dist < 0.6:
                self.state_start_time = self.get_clock().now()
                self.state = f"HOLD{idx+1}"

        # ================= HOLD =================
        elif self.state.startswith("HOLD"):
            idx = int(self.state[-1]) - 1
            wp = self.waypoints[idx]

            # hover in place
            self.publish_position(*wp["pos"])

            elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds * 1e-9

            if elapsed > wp["hold"]:
                if self.state == "HOLD1":
                    self.state = "WP2"
                elif self.state == "HOLD2":
                    self.state = "WP3"
                elif self.state == "HOLD3":
                    self.state = "WP4"
                elif self.state == "HOLD4":
                    self.state = "LAND"

        # ================= LAND =================
        elif self.state == "LAND":

            target = [self.position[0], self.position[1], self.land_z]

            direction = [
                target[0] - self.position[0],
                target[1] - self.position[1],
                target[2] - self.position[2],
            ]

            dist = max(math.sqrt(sum(d*d for d in direction)), 1e-3)

            vx = direction[0] / dist * 2.0
            vy = direction[1] / dist * 2.0
            vz = direction[2] / dist * 2.0

            yaw = math.atan2(direction[1], direction[0])
            self.publish_velocity(vx, vy, vz, yaw)

            if dist < 0.3:
                self.get_logger().info("Landing complete.")

    # ---------------- ARM ----------------
    def arm(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.command = 400
        msg.param1 = 1.0

        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True

        self.cmd_pub.publish(msg)

    # ---------------- OFFBOARD ----------------
    def set_offboard(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.command = 176
        msg.param1 = 1.0
        msg.param2 = 6.0

        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()