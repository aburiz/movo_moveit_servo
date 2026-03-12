#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


class FakeKinovaCommandBridge:
    def __init__(self):
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 100.0))
        self.joint_state_topic = rospy.get_param("~joint_state_topic", "/joint_states")

        self.right_joint_names = list(rospy.get_param("~right_joint_names", []))
        self.left_joint_names = list(rospy.get_param("~left_joint_names", []))
        self.other_joint_names = list(rospy.get_param("~other_joint_names", []))

        self.right_home = dict(rospy.get_param("~right_home_positions", {}))
        self.left_home = dict(rospy.get_param("~left_home_positions", {}))

        self.right_cmd = [0.0 for _ in self.right_joint_names]
        self.left_cmd = [0.0 for _ in self.left_joint_names]
        self.active_arm = "right"

        self.joint_positions = {}
        for j in self.right_joint_names + self.left_joint_names + self.other_joint_names:
            self.joint_positions[j] = 0.0

        self.apply_home("right")
        self.apply_home("left")

        self.joint_state_pub = rospy.Publisher(self.joint_state_topic, JointState, queue_size=20)

        self.right_joint_vel_pub = rospy.Publisher(
            "/right_arm_driver/in/joint_velocity", Float64MultiArray, queue_size=20
        )
        self.left_joint_vel_pub = rospy.Publisher(
            "/left_arm_driver/in/joint_velocity", Float64MultiArray, queue_size=20
        )
        self.right_cart_pub = rospy.Publisher(
            "/right_arm_driver/in/cartesian_velocity", TwistStamped, queue_size=20
        )
        self.left_cart_pub = rospy.Publisher(
            "/left_arm_driver/in/cartesian_velocity", TwistStamped, queue_size=20
        )

        rospy.Subscriber(
            "/movo_servo_teleop_demo/right/joint_velocity_cmd",
            Float64MultiArray,
            self.right_joint_vel_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            "/movo_servo_teleop_demo/left/joint_velocity_cmd",
            Float64MultiArray,
            self.left_joint_vel_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            "/movo_servo_teleop_demo/right/cartesian_velocity_cmd",
            TwistStamped,
            self.right_cartesian_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            "/movo_servo_teleop_demo/left/cartesian_velocity_cmd",
            TwistStamped,
            self.left_cartesian_cb,
            queue_size=20,
        )
        rospy.Subscriber("/movo_servo_teleop_demo/home_request", String, self.home_request_cb, queue_size=2)
        rospy.Subscriber("/movo_servo_teleop_demo/active_arm", String, self.active_arm_cb, queue_size=2)

        self.last_tick = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate_hz), self.timer_cb)

        rospy.loginfo("Fake Kinova bridge publishing joint states on %s", self.joint_state_topic)

    def apply_home(self, arm_name):
        if arm_name == "right":
            for joint, value in self.right_home.items():
                self.joint_positions[joint] = float(value)
        elif arm_name == "left":
            for joint, value in self.left_home.items():
                self.joint_positions[joint] = float(value)

    def active_arm_cb(self, msg):
        arm = msg.data.strip().lower()
        if arm in ("right", "left"):
            self.active_arm = arm

    def home_request_cb(self, msg):
        arm = msg.data.strip().lower()
        if arm not in ("right", "left"):
            return
        self.apply_home(arm)
        rospy.loginfo("Home pose applied to %s arm", arm)

    def right_joint_vel_cb(self, msg):
        self.right_cmd = self.resize_command(msg.data, len(self.right_joint_names))

    def left_joint_vel_cb(self, msg):
        self.left_cmd = self.resize_command(msg.data, len(self.left_joint_names))

    @staticmethod
    def resize_command(raw, expected_len):
        cmd = [0.0 for _ in range(expected_len)]
        for i, val in enumerate(raw):
            if i >= expected_len:
                break
            cmd[i] = float(val)
        return cmd

    def right_cartesian_cb(self, msg):
        # Mirror Servo cartesian command to a Kinova-like dummy cartesian topic.
        out = TwistStamped()
        out.header = msg.header
        out.twist = msg.twist
        self.right_cart_pub.publish(out)

    def left_cartesian_cb(self, msg):
        out = TwistStamped()
        out.header = msg.header
        out.twist = msg.twist
        self.left_cart_pub.publish(out)

    @staticmethod
    def clamp_angle(value):
        # Keep mock positions numerically stable for a long run.
        return max(min(value, math.pi), -math.pi)

    def integrate(self, dt):
        for i, joint in enumerate(self.right_joint_names):
            self.joint_positions[joint] = self.clamp_angle(self.joint_positions[joint] + self.right_cmd[i] * dt)
        for i, joint in enumerate(self.left_joint_names):
            self.joint_positions[joint] = self.clamp_angle(self.joint_positions[joint] + self.left_cmd[i] * dt)

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.right_joint_names + self.left_joint_names + self.other_joint_names
        msg.position = [self.joint_positions[j] for j in msg.name]
        msg.velocity = self.right_cmd + self.left_cmd + [0.0 for _ in self.other_joint_names]
        msg.effort = []
        self.joint_state_pub.publish(msg)

    def publish_dummy_joint_velocities(self):
        right_msg = Float64MultiArray()
        right_msg.data = list(self.right_cmd)
        left_msg = Float64MultiArray()
        left_msg.data = list(self.left_cmd)
        self.right_joint_vel_pub.publish(right_msg)
        self.left_joint_vel_pub.publish(left_msg)

        rospy.loginfo_throttle(
            1.0,
            "Dummy Kinova joint velocity | active_arm=%s | right=%s | left=%s",
            self.active_arm,
            [round(v, 3) for v in self.right_cmd],
            [round(v, 3) for v in self.left_cmd],
        )

    def timer_cb(self, _event):
        now = rospy.Time.now()
        dt = (now - self.last_tick).to_sec()
        self.last_tick = now
        if dt <= 0.0:
            return

        self.integrate(dt)
        self.publish_joint_state()
        self.publish_dummy_joint_velocities()


def main():
    rospy.init_node("fake_kinova_command_bridge")
    FakeKinovaCommandBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
