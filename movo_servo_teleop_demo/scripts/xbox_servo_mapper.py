#!/usr/bin/env python3

import copy

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64, String


class XboxServoMapper:
    def __init__(self):
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 100.0))
        self.command_frame = rospy.get_param("~command_frame", "base_link")

        self.axis_linear_x = int(rospy.get_param("~axis_linear_x", 1))
        self.axis_linear_y = int(rospy.get_param("~axis_linear_y", 0))
        self.axis_linear_z = int(rospy.get_param("~axis_linear_z", 4))
        self.axis_angular_z = int(rospy.get_param("~axis_angular_z", 3))
        self.axis_angular_roll = int(rospy.get_param("~axis_angular_roll", 6))
        self.axis_angular_pitch = int(rospy.get_param("~axis_angular_pitch", 7))
        self.axis_trigger_left = int(rospy.get_param("~axis_trigger_left", 2))
        self.axis_trigger_right = int(rospy.get_param("~axis_trigger_right", 5))

        self.scale_linear_xy = float(rospy.get_param("~scale_linear_xy", 0.30))
        self.scale_linear_z = float(rospy.get_param("~scale_linear_z", 0.25))
        self.scale_angular_roll = float(rospy.get_param("~scale_angular_roll", 0.70))
        self.scale_angular_pitch = float(rospy.get_param("~scale_angular_pitch", 0.70))
        self.scale_angular_yaw = float(rospy.get_param("~scale_angular_yaw", 0.90))
        self.gripper_open_speed = float(rospy.get_param("~gripper_open_speed", 0.75))
        self.gripper_close_speed = float(rospy.get_param("~gripper_close_speed", 0.75))

        # Buttons (Ubuntu 20.04 joy_node default for Xbox pads):
        # A=0, B=1, X=2, Y=3, LB=4, RB=5
        self.start_button = int(rospy.get_param("~start_driver_button", 0))
        self.estop_button = int(rospy.get_param("~estop_button", 1))
        self.cycle_target_button = int(rospy.get_param("~cycle_target_button", 2))
        self.home_current_button = int(rospy.get_param("~home_current_button", 5))
        self.home_both_button = int(rospy.get_param("~home_both_button", 4))
        self.home_both_double_tap_window = float(rospy.get_param("~home_both_double_tap_window", 0.45))

        self.active_arm = self.normalize_arm_name(rospy.get_param("~default_active_arm", "right"))
        self.driver_enabled = bool(rospy.get_param("~driver_enabled_on_start", False))
        self.estop_latched = False
        self.last_home_both_press = rospy.Time(0)

        self.last_joy = Joy()
        self.have_joy = False
        self.prev_buttons = []

        self.right_twist_pub = rospy.Publisher(
            "/movo_servo_teleop_demo/right/delta_twist_cmds", TwistStamped, queue_size=20
        )
        self.left_twist_pub = rospy.Publisher(
            "/movo_servo_teleop_demo/left/delta_twist_cmds", TwistStamped, queue_size=20
        )

        self.right_cart_pub = rospy.Publisher(
            "/movo_servo_teleop_demo/right/cartesian_velocity_cmd", TwistStamped, queue_size=20
        )
        self.left_cart_pub = rospy.Publisher(
            "/movo_servo_teleop_demo/left/cartesian_velocity_cmd", TwistStamped, queue_size=20
        )

        self.home_request_pub = rospy.Publisher(
            "/movo_servo_teleop_demo/home_request", String, queue_size=2
        )
        self.active_arm_pub = rospy.Publisher(
            "/movo_servo_teleop_demo/active_arm", String, queue_size=2, latch=True
        )
        self.driver_enabled_pub = rospy.Publisher(
            "/movo_servo_teleop_demo/driver_enabled", Bool, queue_size=2, latch=True
        )

        self.right_gripper_pub = rospy.Publisher(
            "/movo_servo_teleop_demo/right/gripper_velocity_cmd", Float64, queue_size=20
        )
        self.left_gripper_pub = rospy.Publisher(
            "/movo_servo_teleop_demo/left/gripper_velocity_cmd", Float64, queue_size=20
        )

        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate_hz), self.timer_cb)

        self.publish_active_arm()
        self.publish_driver_enabled()
        rospy.loginfo("Xbox mapper ready. Active arm: %s", self.active_arm)

    def publish_active_arm(self):
        msg = String()
        msg.data = self.active_arm
        self.active_arm_pub.publish(msg)

    def publish_driver_enabled(self):
        msg = Bool()
        msg.data = self.driver_enabled and not self.estop_latched
        self.driver_enabled_pub.publish(msg)

    def publish_home_request(self, arm):
        msg = String()
        msg.data = arm
        self.home_request_pub.publish(msg)

    @staticmethod
    def normalize_arm_name(raw_name):
        arm_name = str(raw_name).strip().lower()
        if arm_name in ("left", "left_arm"):
            return "left"
        if arm_name in ("right", "right_arm"):
            return "right"
        return "right"

    @staticmethod
    def read_axis(joy_msg, idx, default_val=0.0):
        if idx < 0 or idx >= len(joy_msg.axes):
            return float(default_val)
        return float(joy_msg.axes[idx])

    @staticmethod
    def read_button(joy_msg, idx):
        if idx < 0 or idx >= len(joy_msg.buttons):
            return 0
        return int(joy_msg.buttons[idx])

    @staticmethod
    def trigger_to_unit(axis_val):
        # Common Xbox mapping is +1.0 unpressed to -1.0 fully pressed.
        return 0.5 * (1.0 - axis_val)

    def is_rising_edge(self, msg, button_idx):
        if button_idx < 0:
            return False
        pressed = self.read_button(msg, button_idx)
        prev = self.prev_buttons[button_idx] if button_idx < len(self.prev_buttons) else 0
        return pressed == 1 and prev == 0

    def joy_cb(self, msg):
        if not self.prev_buttons:
            self.prev_buttons = [0 for _ in msg.buttons]

        if self.is_rising_edge(msg, self.start_button):
            self.driver_enabled = True
            self.estop_latched = False
            self.publish_driver_enabled()
            rospy.loginfo("Driver enabled")

        if self.is_rising_edge(msg, self.estop_button):
            self.driver_enabled = False
            self.estop_latched = True
            self.publish_driver_enabled()
            rospy.logwarn("Emergency stop latched. Press A to re-enable driver.")

        # Cycle active arm on X
        if self.is_rising_edge(msg, self.cycle_target_button):
            self.active_arm = "left" if self.active_arm == "right" else "right"
            self.publish_active_arm()
            rospy.loginfo("Active arm toggled to: %s", self.active_arm)

        # Home active arm on RB
        if self.is_rising_edge(msg, self.home_current_button):
            self.publish_home_request(self.active_arm)
            rospy.loginfo("Home requested for %s arm", self.active_arm)

        # Double-tap LB to home both arms
        if self.is_rising_edge(msg, self.home_both_button):
            now = rospy.Time.now()
            dt = (now - self.last_home_both_press).to_sec()
            if self.last_home_both_press.to_sec() > 0.0 and dt <= self.home_both_double_tap_window:
                self.publish_home_request("right")
                self.publish_home_request("left")
                rospy.loginfo("Home requested for both arms")
                self.last_home_both_press = rospy.Time(0)
            else:
                self.last_home_both_press = now

        self.last_joy = copy.deepcopy(msg)
        self.have_joy = True
        self.prev_buttons = list(msg.buttons)

    def make_twist_from_joy(self):
        cmd = TwistStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = self.command_frame

        if not self.have_joy:
            return cmd

        if not self.driver_enabled or self.estop_latched:
            return cmd

        cmd.twist.linear.x = self.read_axis(self.last_joy, self.axis_linear_x) * self.scale_linear_xy
        cmd.twist.linear.y = self.read_axis(self.last_joy, self.axis_linear_y) * self.scale_linear_xy
        cmd.twist.linear.z = self.read_axis(self.last_joy, self.axis_linear_z) * self.scale_linear_z

        cmd.twist.angular.x = self.read_axis(self.last_joy, self.axis_angular_roll) * self.scale_angular_roll
        cmd.twist.angular.y = self.read_axis(self.last_joy, self.axis_angular_pitch) * self.scale_angular_pitch
        cmd.twist.angular.z = self.read_axis(self.last_joy, self.axis_angular_z) * self.scale_angular_yaw

        return cmd

    def make_gripper_velocity_from_joy(self):
        if not self.have_joy:
            return 0.0
        if not self.driver_enabled or self.estop_latched:
            return 0.0

        left_trigger = self.trigger_to_unit(self.read_axis(self.last_joy, self.axis_trigger_left, 1.0))
        right_trigger = self.trigger_to_unit(self.read_axis(self.last_joy, self.axis_trigger_right, 1.0))

        open_vel = left_trigger * self.gripper_open_speed
        close_vel = right_trigger * self.gripper_close_speed
        return close_vel - open_vel

    def timer_cb(self, _event):
        active_cmd = self.make_twist_from_joy()
        active_gripper = self.make_gripper_velocity_from_joy()

        zero_cmd = TwistStamped()
        zero_cmd.header.stamp = rospy.Time.now()
        zero_cmd.header.frame_id = self.command_frame

        zero_gripper = Float64()
        zero_gripper.data = 0.0
        active_gripper_msg = Float64()
        active_gripper_msg.data = active_gripper

        if self.active_arm == "right":
            self.right_twist_pub.publish(active_cmd)
            self.left_twist_pub.publish(zero_cmd)
            self.right_cart_pub.publish(active_cmd)
            self.left_cart_pub.publish(zero_cmd)
            self.right_gripper_pub.publish(active_gripper_msg)
            self.left_gripper_pub.publish(zero_gripper)
        else:
            self.right_twist_pub.publish(zero_cmd)
            self.left_twist_pub.publish(active_cmd)
            self.right_cart_pub.publish(zero_cmd)
            self.left_cart_pub.publish(active_cmd)
            self.right_gripper_pub.publish(zero_gripper)
            self.left_gripper_pub.publish(active_gripper_msg)


def main():
    rospy.init_node("xbox_servo_mapper")
    XboxServoMapper()
    rospy.spin()


if __name__ == "__main__":
    main()
