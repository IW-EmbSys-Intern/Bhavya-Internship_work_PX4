#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from pynput import keyboard

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DroneTeleOp(Node):

    def __init__(self):
        super().__init__("drone_teleop")

        self.state = "IDLE"
        self.mode = "IDLE"
    
        self.target_altitude = -10.0

        # hold position
        self.hold_initialized = False
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_z = 0.0

        # Offboard Security
        self.offboard_armed  =False
        self.offboard_counter = 0
        self.offboard_ready = False

        # odometry
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        # mission control
        self.angle = 0.0
        self.step = 0
        self.mission_select_mode = False
        self.mission_type = None

        # Mission system
        self.mission_time = 0.0
        self.mission_active = False

        # Circle Mission
        self.circle_radius = 25.0 #meters
        self.circle_speed = 0.3 #rad/sec
        self.circle_angle = 0.0

        self.circle_center_x = 0.0
        self.circle_center_y = 0.0
        self.circle_altitude = -10.0

        # Square mission
        self.square_waypoints = []
        self.square_index = 0
        self.square_hover_counter = 0

        # Figure 8
        self.fig8_center_x = 0.0
        self.fig8_center_y = 0.0
        self.fig8_omega = 0.3

        # Takeoff Control
        self.arm_time = None
        self.waiting_for_takeoff = False
        self.takeoff_x = 0.0
        self.takeoff_y = 0.0
        self.takeoff_done = False


        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.odom_subscriber = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)
        self.hold_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.offboard_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode',10)
        self.cmd_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        
        self.keyboard_listener = keyboard.Listener(on_press = self.on_key_press)
        self.keyboard_listener.start()

        
        self.timer = self.create_timer(0.1, self.loop)

    

    def get_time(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def odom_callback(self, msg):
        # print('Odometry Received')
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_z = msg.position[2]

        self.last_odom_time = self.get_clock().now()

    def publish_hold_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_time()

        msg.position = [self.hold_x,
                        self.hold_y,
                        self.hold_z]
        
        msg.yaw = 0.0

        self.hold_setpoint_publisher.publish(msg)
        
    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_time()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_mode_publisher.publish(msg)

    def engage_offboard(self):
        msg = VehicleCommand()
        msg.timestamp = self.get_time()

        msg.command = 176  # SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0     # OFFBOARD mode

        self.cmd_publisher.publish(msg)

    def on_key_press(self, key):
        print("KEY EVENT TRIGGERED:", key)
        try:
            if not hasattr(key, "char") or key.char is None:
                return
            k = key.char.lower()

            # -------------------------
            # MISSION SELECTION MODE
            # -------------------------
            if self.mission_select_mode:

                if k == '1':
                    self.mission_type = "CIRCLE"
                    self.start_mission()
                    self.mission_select_mode = False
                    print("MISSION SELECTED: CIRCLE")

                elif k == '2':
                    self.mission_type = "SQUARE"
                    self.start_mission()
                    self.mission_select_mode = False
                    print("MISSION SELECTED: SQUARE")

                elif k == '3':
                    self.mission_type = "FIGURE8"
                    self.start_mission()
                    self.mission_select_mode = False
                    print("MISSION SELECTED: FIGURE-8")

                return  # IMPORTANT: block other keys during selection

            if k == 'a':
                print('Arming')
                self.engage_offboard()
                self.arm()

            elif k == 't':
                print('Taking-Off')
                # if self.state == "ARMED_IDLE":
                #     self.takeoff()
                #     self.waiting_for_takeoff = False
                #     self.arm_time = None
                if self.mode == "ARMED":
                    self.takeoff()

            elif k == 'm':
                print('\nSelect Mission:')
                print("1: CIRCLE")
                print("2: SQUARE")
                print("3: FIGURE-8")

                self.mission_select_mode = True

            elif k == 'l':
                print('Landing')
                self.land()

            elif k =='r':
                self.rtl()

            elif k == 'd':
                print('Disarming')
                self.disarm()

            elif k == 'h':
                print('Hold')
                self.capture_and_hold()
                
        except Exception as e:
            print("[KEYBOARD ERROR]", e)
    
    def arm(self):
        self.offboard_ready = False
        self.offboard_counter = 0
        self.mode = "ARMED"

        self.arm_time = self.get_clock().now().nanoseconds
        self.waiting_for_takeoff = True

        msg = VehicleCommand()
        msg.timestamp = self.get_time()

        msg.command = 400 #Arm/DIsarm
        msg.param1 = 1.0 #Arm

        self.cmd_publisher.publish(msg)

    def disarm(self):
        msg = VehicleCommand()
        msg.timestamp = self.get_time()

        msg.command = 400 #Arm/DIsarm
        msg.param1 = 0.0 #Disarm

        self.cmd_publisher.publish(msg)

    def land(self):
        msg = VehicleCommand()
        msg.timestamp = self.get_time()

        msg.command = 21 #NAV_LAND

        self.cmd_publisher.publish(msg)
        self.state = "LAND"
    
    def rtl(self):
        msg = VehicleCommand()
        msg.timestamp = self.get_time()

        msg.command = 20 #NAV_RETURN_TO_LAUNCH

        self.cmd_publisher.publish(msg)
        self.state = "RTL"

    def takeoff(self):
        self.state = "TAKEOFF"
        self.mode = "TAKEOFF"
        self.takeoff_done = False

        self.takeoff_x = self.current_x
        self.takeoff_y = self.current_y

        self.hold_initialized = False

        self.offboard_ready = False
        self.offboard_counter = 0

    def start_mission(self):
        self.state = "MISSION"
        self.mission_time = 0.0
        
        self.mission_active = True

        self.circle_center_x = self.current_x
        self.circle_center_y = self.current_y

        # square init
        L = 20.0
        self.square_waypoints = [
            (self.current_x, self.current_y),
            (self.current_x + L, self.current_y),
            (self.current_x + L, self.current_y + L),
            (self.current_x, self.current_y + L),
            (self.current_x, self.current_y),
        ]
        self.square_index = 0
        self.square_hover_counter = 0

        #figure 8 init
        self.fig8_center_x = self.current_x
        self.fig8_center_y = self.current_y
     
    def get_mission_setpoint(self):
        self.mission_time += 0.1
        t = self.mission_time

        # -------------------------
        # CIRCLE
        # -------------------------
        if self.mission_type == "CIRCLE":
            x = self.circle_center_x + self.circle_radius * math.cos(self.circle_speed * t)
            y = self.circle_center_y + self.circle_radius * math.sin(self.circle_speed * t)

            vx = -self.circle_radius * self.circle_speed * math.sin(self.circle_speed * t)
            vy =  self.circle_radius * self.circle_speed * math.cos(self.circle_speed * t)

            z = self.circle_altitude
            yaw = math.atan2(vy, vx)

            return x, y, z, yaw

        # -------------------------
        # FIGURE 8
        # -------------------------
        elif self.mission_type == "FIGURE8":

            omega = self.fig8_omega

            x = self.fig8_center_x + self.circle_radius * math.sin(omega * t)
            y = self.fig8_center_y + (self.circle_radius / 2) * math.sin(2 * omega * t)

            vx = self.circle_radius * omega * math.cos(omega * t)
            vy = self.circle_radius * omega * math.cos(2 * omega * t)

            z = self.circle_altitude
            yaw = math.atan2(vy, vx)

            return x, y, z, yaw
        
        # -------------------------
        # SQUARE
        # -------------------------
        # elif self.mission_type == "SQUARE":

        #     wp = self.square_waypoints

        #     i = self.square_index
        #     next_i = (i + 1) % len(wp)

        #     x1, y1 = wp[i]
        #     x2, y2 = wp[next_i]

        #     alpha = min(self.square_hover_counter / 50.0, 1.0)

        #     x = x1 + alpha * (x2 - x1)
        #     y = y1 + alpha * (y2 - y1)
        #     z = self.circle_altitude

        #     vx = x2 - x1
        #     vy = y2 - y1
        #     yaw = math.atan2(vy, vx)

        #     return x, y, z, yaw
        elif self.mission_type == "SQUARE":
            wp = self.square_waypoints

            i = self.square_index
            next_i = (i + 1) % len(wp)

            x1, y1 = wp[i]
            x2, y2 = wp[next_i]

            # progress along segment
            step_size = 0.02   # smoothness control (IMPORTANT)
            self.square_hover_counter += step_size

            alpha = self.square_hover_counter

            x = x1 + alpha * (x2 - x1)
            y = y1 + alpha * (y2 - y1)
            z = self.circle_altitude

            vx = x2 - x1
            vy = y2 - y1
            yaw = math.atan2(vy, vx)

            # -------------------------
            # segment completion logic
            # -------------------------
            if self.square_hover_counter >= 1.0:
                self.square_hover_counter = 0.0
                self.square_index += 1

                # loop or finish
                if self.square_index >= len(wp) - 1:
                    self.square_index = 0  # or stop mission

            return x, y, z, yaw


        #Fallback Safety
        return self.current_x, self.current_y, self.current_z, 0.0
            


    def capture_and_hold(self):
        self.hold_x = self.last_setpoint_x
        self.hold_y = self.last_setpoint_y
        self.hold_z = self.last_setpoint_z

        self.hold_initialized = True

        self.state = "HOLD"
        self.takeoff_done = True
        self.takeoff_x = self.hold_x
        self.takeoff_y = self.hold_y
        print(f"HOLD POSITION SET: {self.hold_x}, {self.hold_y}, {self.hold_z}")

    def check_watchdog(self):
        if self.mode != "ARMED":
            return
        if self.arm_time is None:
            return
        
        elapsed = (self.get_clock().now().nanoseconds - self.arm_time) / 1e9

        if elapsed > 5.0:
            print("No Takeoff Detected: AUTO DISARMING")
            self.disarm()
            self.waiting_for_takeoff = False
            self.arm_time = None
            self.state = "IDLE"
            

    def loop(self):
        
        self.publish_offboard_mode()

        if self.waiting_for_takeoff:
            self.check_watchdog()

        if not self.offboard_ready:
            self.publish_hold_setpoint()
            self.offboard_counter += 1

            if self.offboard_counter > 20: # ~2 seconds
                self.engage_offboard()
                self.offboard_ready = True
            return

        if self.state == "TAKEOFF" and not self.hold_initialized:
            self.last_setpoint_x = self.takeoff_x
            self.last_setpoint_y = self.takeoff_y
            self.last_setpoint_z = self.target_altitude
            msg = TrajectorySetpoint()
            msg.timestamp = self.get_time()

            msg.position = [self.takeoff_x,
                            self.takeoff_y,
                            self.target_altitude
                            ]
            
            msg.yaw = 0.0
            
            self.hold_setpoint_publisher.publish(msg)

            if abs(self.current_z - self.target_altitude) <0.5:
                self.takeoff_done = True

            if self.takeoff_done:
                self.capture_and_hold()

        elif self.state in ["IDLE", "HOLD"]:
            self.last_setpoint_x = self.hold_x
            self.last_setpoint_y = self.hold_y
            self.last_setpoint_z = self.hold_z
            self.publish_hold_setpoint()

        elif self.state == "MISSION":

            x, y, z, yaw = self.get_mission_setpoint()

            self.last_setpoint_x = x
            self.last_setpoint_y = y
            self.last_setpoint_z = z

            msg = TrajectorySetpoint()
            msg.timestamp = self.get_time()

            msg.position = [x, y, z]
            msg.yaw = yaw

            self.hold_setpoint_publisher.publish(msg)


def main():
    rclpy.init()
    node = DroneTeleOp()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()