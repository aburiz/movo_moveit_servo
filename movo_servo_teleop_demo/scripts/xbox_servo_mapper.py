#!/usr/bin/env python3

import copy

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class XboxServoMapper:
    def __init__(self):
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 100.0))
        self.command_frame = rospy.get_param("~command_frame", "base_link")

        # Set deadman_button to -1 to disable gating and always stream commands.
        self.deadman_button = int(rospy.get_param("~deadman_button", -1))
        # Set toggle_arm_button to -1 to keep one active arm fixed.
        self.toggle_arm_button = int(rospy.get_param("~toggle_arm_button", -1))
        self.home_button = int(rospy.get_param("~home_button", 3))

        self.axis_linear_x = int(rospy.get_param("~axis_linear_x", 1))
        self.axis_linear_y = int(rospy.get_param("~axis_linear_y", 0))
        self.axis_linear_z = int(rospy.get_param("~axis_linear_z", 4))
        self.axis_angular_z = int(rospy.get_param("~axis_angular_z", 3))
        self.axis_trigger_left = int(rospy.get_param("~axis_trigger_left", 2))
        self.axis_trigger_right = int(rospy.get_param("~axis_trigger_right", 5))
        self.button_roll_negative = int(rospy.get_param("~button_roll_negative", 4))
        self.button_roll_positive = int(rospy.get_param("~button_roll_positive", 5))

        self.scale_linear_xy = float(rospy.get_param("~scale_linear_xy", 0.30))
        self.scale_linear_z = float(rospy.get_param("~scale_linear_z", 0.25))
        self.scale_angular_roll = float(rospy.get_param("~scale_angular_roll", 0.70))
        self.scale_angular_pitch = float(rospy.get_param("~scale_angular_pitch", 0.70))
        self.scale_angular_yaw = float(rospy.get_param("~scale_angular_yaw", 0.90))

        self.active_arm = rospy.get_param("~default_active_arm", "right").strip().lower()
        if self.active_arm not in ("right", "left"):
            self.active_arm = "right"

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

        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate_hz), self.timer_cb)

        self.publish_active_arm()
        rospy.loginfo("Xbox mapper ready. Active arm: %s", self.active_arm)

    def publish_active_arm(self):
        msg = String()
        msg.data = self.active_arm
        self.active_arm_pub.publish(msg)

    @staticmethod
    def read_axis(joy_msg, idx):
        if idx < 0 or idx >= len(joy_msg.axes):
            return 0.0
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

        # Toggle active arm on rising edge
        if self.is_rising_edge(msg, self.toggle_arm_button):
            self.active_arm = "left" if self.active_arm == "right" else "right"
            self.publish_active_arm()
            rospy.loginfo("Active arm toggled to: %s", self.active_arm)

        # Home active arm on rising edge
        if self.is_rising_edge(msg, self.home_button):
            home_msg = String()
            home_msg.data = self.active_arm
            self.home_request_pub.publish(home_msg)
            rospy.loginfo("Home requested for %s arm", self.active_arm)

        self.last_joy = copy.deepcopy(msg)
        self.have_joy = True
        self.prev_buttons = list(msg.buttons)

    def make_twist_from_joy(self):
        cmd = TwistStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = self.command_frame

        if not self.have_joy:
            return cmd

        if self.deadman_button >= 0:
            deadman = self.read_button(self.last_joy, self.deadman_button) == 1
            if not deadman:
                return cmd

        roll_negative = self.read_button(self.last_joy, self.button_roll_negative)
        roll_positive = self.read_button(self.last_joy, self.button_roll_positive)

        roll_cmd = float(roll_positive - roll_negative) * self.scale_angular_roll

        cmd.twist.linear.x = self.read_axis(self.last_joy, self.axis_linear_x) * self.scale_linear_xy
        cmd.twist.linear.y = self.read_axis(self.last_joy, self.axis_linear_y) * self.scale_linear_xy
        cmd.twist.linear.z = self.read_axis(self.last_joy, self.axis_linear_z) * self.scale_linear_z

        cmd.twist.angular.x = roll_cmd
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = self.read_axis(self.last_joy, self.axis_angular_z) * self.scale_angular_yaw

        return cmd

    def timer_cb(self, _event):
        active_cmd = self.make_twist_from_joy()
        zero_cmd = TwistStamped()
        zero_cmd.header.stamp = rospy.Time.now()
        zero_cmd.header.frame_id = self.command_frame

        if self.active_arm == "right":
            self.right_twist_pub.publish(active_cmd)
            self.left_twist_pub.publish(zero_cmd)
            self.right_cart_pub.publish(active_cmd)
            self.left_cart_pub.publish(zero_cmd)
        else:
            self.right_twist_pub.publish(zero_cmd)
            self.left_twist_pub.publish(active_cmd)
            self.right_cart_pub.publish(zero_cmd)
            self.left_cart_pub.publish(active_cmd)


def main():
    rospy.init_node("xbox_servo_mapper")
    XboxServoMapper()
    rospy.spin()


if __name__ == "__main__":
    main()
