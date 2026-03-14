#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String


class FakeKinovaCommandBridge:
    def __init__(self):
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 100.0))
        self.joint_state_topic = rospy.get_param("~joint_state_topic", "/joint_states")
        self.right_servo_joint_velocity_topic = rospy.get_param(
            "~right_servo_joint_velocity_topic",
            "/movo_servo_teleop_demo/right/joint_velocity_cmd",
        )
        self.left_servo_joint_velocity_topic = rospy.get_param(
            "~left_servo_joint_velocity_topic",
            "/movo_servo_teleop_demo/left/joint_velocity_cmd",
        )
        self.right_cartesian_velocity_topic = rospy.get_param(
            "~right_cartesian_velocity_topic",
            "/movo_servo_teleop_demo/right/cartesian_velocity_cmd",
        )
        self.left_cartesian_velocity_topic = rospy.get_param(
            "~left_cartesian_velocity_topic",
            "/movo_servo_teleop_demo/left/cartesian_velocity_cmd",
        )
        self.right_gripper_velocity_topic = rospy.get_param(
            "~right_gripper_velocity_topic",
            "/movo_servo_teleop_demo/right/gripper_velocity_cmd",
        )
        self.left_gripper_velocity_topic = rospy.get_param(
            "~left_gripper_velocity_topic",
            "/movo_servo_teleop_demo/left/gripper_velocity_cmd",
        )
        self.home_request_topic = rospy.get_param("~home_request_topic", "/movo_servo_teleop_demo/home_request")
        self.active_arm_topic = rospy.get_param("~active_arm_topic", "/movo_servo_teleop_demo/active_arm")
        self.right_dummy_joint_velocity_topic = rospy.get_param(
            "~right_dummy_joint_velocity_topic",
            "/right_arm_driver/in/joint_velocity",
        )
        self.left_dummy_joint_velocity_topic = rospy.get_param(
            "~left_dummy_joint_velocity_topic",
            "/left_arm_driver/in/joint_velocity",
        )
        self.right_dummy_cartesian_velocity_topic = rospy.get_param(
            "~right_dummy_cartesian_velocity_topic",
            "/right_arm_driver/in/cartesian_velocity",
        )
        self.left_dummy_cartesian_velocity_topic = rospy.get_param(
            "~left_dummy_cartesian_velocity_topic",
            "/left_arm_driver/in/cartesian_velocity",
        )
        self.right_dummy_gripper_velocity_topic = rospy.get_param(
            "~right_dummy_gripper_velocity_topic",
            "/right_arm_driver/in/gripper_velocity",
        )
        self.left_dummy_gripper_velocity_topic = rospy.get_param(
            "~left_dummy_gripper_velocity_topic",
            "/left_arm_driver/in/gripper_velocity",
        )

        self.right_joint_names = list(rospy.get_param("~right_joint_names", []))
        self.left_joint_names = list(rospy.get_param("~left_joint_names", []))
        self.other_joint_names = list(rospy.get_param("~other_joint_names", []))

        self.right_home = dict(rospy.get_param("~right_home_positions", {}))
        self.left_home = dict(rospy.get_param("~left_home_positions", {}))
        self.right_home_native_degrees = dict(rospy.get_param("~right_home_native_degrees", {}))
        self.left_home_native_degrees = dict(rospy.get_param("~left_home_native_degrees", {}))

        self.right_gripper_joint = rospy.get_param("~right_gripper_joint", "right_gripper_finger1_joint")
        self.left_gripper_joint = rospy.get_param("~left_gripper_joint", "left_gripper_finger1_joint")
        self.gripper_min_position = float(rospy.get_param("~gripper_min_position", 0.0))
        self.gripper_max_position = float(rospy.get_param("~gripper_max_position", 0.986111027))
        self.right_gripper_home_position = float(rospy.get_param("~right_gripper_home_position", 0.0))
        self.left_gripper_home_position = float(rospy.get_param("~left_gripper_home_position", 0.0))

        self.right_cmd = [0.0 for _ in self.right_joint_names]
        self.left_cmd = [0.0 for _ in self.left_joint_names]
        self.right_gripper_cmd = 0.0
        self.left_gripper_cmd = 0.0
        self.active_arm = "right"

        self.joint_positions = {}
        self.ensure_joint_in_other_names(self.right_gripper_joint)
        self.ensure_joint_in_other_names(self.left_gripper_joint)
        for j in self.right_joint_names + self.left_joint_names + self.other_joint_names:
            self.joint_positions[j] = 0.0

        self.right_home = self.resolve_home_positions(
            "right", self.right_joint_names, self.right_home, self.right_home_native_degrees
        )
        self.left_home = self.resolve_home_positions(
            "left", self.left_joint_names, self.left_home, self.left_home_native_degrees
        )

        self.apply_home("right")
        self.apply_home("left")

        self.joint_state_pub = rospy.Publisher(self.joint_state_topic, JointState, queue_size=20)

        self.right_joint_vel_pub = rospy.Publisher(
            self.right_dummy_joint_velocity_topic, Float64MultiArray, queue_size=20
        )
        self.left_joint_vel_pub = rospy.Publisher(
            self.left_dummy_joint_velocity_topic, Float64MultiArray, queue_size=20
        )
        self.right_cart_pub = rospy.Publisher(
            self.right_dummy_cartesian_velocity_topic, TwistStamped, queue_size=20
        )
        self.left_cart_pub = rospy.Publisher(
            self.left_dummy_cartesian_velocity_topic, TwistStamped, queue_size=20
        )
        self.right_gripper_pub = rospy.Publisher(
            self.right_dummy_gripper_velocity_topic, Float64, queue_size=20
        )
        self.left_gripper_pub = rospy.Publisher(
            self.left_dummy_gripper_velocity_topic, Float64, queue_size=20
        )

        rospy.Subscriber(
            self.right_servo_joint_velocity_topic,
            Float64MultiArray,
            self.right_joint_vel_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.left_servo_joint_velocity_topic,
            Float64MultiArray,
            self.left_joint_vel_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.right_cartesian_velocity_topic,
            TwistStamped,
            self.right_cartesian_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.left_cartesian_velocity_topic,
            TwistStamped,
            self.left_cartesian_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.right_gripper_velocity_topic,
            Float64,
            self.right_gripper_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.left_gripper_velocity_topic,
            Float64,
            self.left_gripper_cb,
            queue_size=20,
        )
        rospy.Subscriber(self.home_request_topic, String, self.home_request_cb, queue_size=2)
        rospy.Subscriber(self.active_arm_topic, String, self.active_arm_cb, queue_size=2)

        self.last_tick = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate_hz), self.timer_cb)

        rospy.loginfo("Fake Kinova bridge publishing joint states on %s", self.joint_state_topic)

    def apply_home(self, arm_name):
        if arm_name == "right":
            for joint, value in self.right_home.items():
                self.joint_positions[joint] = float(value)
            self.joint_positions[self.right_gripper_joint] = self.right_gripper_home_position
        elif arm_name == "left":
            for joint, value in self.left_home.items():
                self.joint_positions[joint] = float(value)
            self.joint_positions[self.left_gripper_joint] = self.left_gripper_home_position

    def ensure_joint_in_other_names(self, joint_name):
        if joint_name and joint_name not in self.other_joint_names:
            self.other_joint_names.append(joint_name)

    @staticmethod
    def wrap_angle(value):
        return (value + math.pi) % (2.0 * math.pi) - math.pi

    def native_degrees_to_joint_positions(self, joint_names, native_degrees):
        # Expected input keys: joint1..jointN in Kinova native actuator angle convention.
        n = len(joint_names)
        ordered_degrees = []
        for idx in range(n):
            key = "joint{}".format(idx + 1)
            if key not in native_degrees:
                return None
            ordered_degrees.append(float(native_degrees[key]))

        if n == 7:
            # Match conversion in movo_joint_interface:
            # kinova_api_wrapper.get_angular_position() + jaco_joint_controller._update_controller_data().
            mapped = [
                math.radians(ordered_degrees[0]),
                math.radians(ordered_degrees[1] - 180.0),
                math.radians(ordered_degrees[2]),
                math.radians(ordered_degrees[3] - 180.0),
                math.radians(ordered_degrees[4]),
                math.radians(ordered_degrees[5] - 180.0),
                math.radians(ordered_degrees[6]),
            ]
            for i in (0, 2, 4, 6):
                mapped[i] = self.wrap_angle(mapped[i])
        elif n == 6:
            mapped = [
                math.radians(ordered_degrees[0]),
                math.radians(ordered_degrees[1] - 180.0),
                math.radians(ordered_degrees[2] - 180.0),
                math.radians(ordered_degrees[3]),
                math.radians(ordered_degrees[4]),
                math.radians(ordered_degrees[5]),
            ]
            for i in (0, 3, 4, 5):
                mapped[i] = self.wrap_angle(mapped[i])
        else:
            mapped = [self.wrap_angle(math.radians(v)) for v in ordered_degrees]

        return {joint_names[i]: mapped[i] for i in range(n)}

    def resolve_home_positions(self, arm_name, joint_names, home_positions, native_degrees):
        if native_degrees:
            converted = self.native_degrees_to_joint_positions(joint_names, native_degrees)
            if converted is not None:
                rospy.loginfo("Using %s_home_native_degrees for home pose conversion", arm_name)
                return converted
            rospy.logwarn(
                "%s_home_native_degrees is incomplete for %d joints. Falling back to %s_home_positions.",
                arm_name,
                len(joint_names),
                arm_name,
            )
        return home_positions

    def active_arm_cb(self, msg):
        arm = msg.data.strip().lower()
        if arm in ("right", "left"):
            self.active_arm = arm

    def home_request_cb(self, msg):
        arm = msg.data.strip().lower()
        if arm == "both":
            self.apply_home("right")
            self.apply_home("left")
            rospy.loginfo("Home pose applied to both arms")
            return
        if arm not in ("right", "left"):
            return
        self.apply_home(arm)
        rospy.loginfo("Home pose applied to %s arm", arm)

    def right_joint_vel_cb(self, msg):
        self.right_cmd = self.resize_command(msg.data, len(self.right_joint_names))

    def left_joint_vel_cb(self, msg):
        self.left_cmd = self.resize_command(msg.data, len(self.left_joint_names))

    def right_gripper_cb(self, msg):
        self.right_gripper_cmd = float(msg.data)

    def left_gripper_cb(self, msg):
        self.left_gripper_cmd = float(msg.data)

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

    def clamp_gripper(self, value):
        return max(min(value, self.gripper_max_position), self.gripper_min_position)

    def integrate(self, dt):
        for i, joint in enumerate(self.right_joint_names):
            self.joint_positions[joint] = self.clamp_angle(self.joint_positions[joint] + self.right_cmd[i] * dt)
        for i, joint in enumerate(self.left_joint_names):
            self.joint_positions[joint] = self.clamp_angle(self.joint_positions[joint] + self.left_cmd[i] * dt)
        self.joint_positions[self.right_gripper_joint] = self.clamp_gripper(
            self.joint_positions[self.right_gripper_joint] + self.right_gripper_cmd * dt
        )
        self.joint_positions[self.left_gripper_joint] = self.clamp_gripper(
            self.joint_positions[self.left_gripper_joint] + self.left_gripper_cmd * dt
        )

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.right_joint_names + self.left_joint_names + self.other_joint_names
        msg.position = [self.joint_positions[j] for j in msg.name]
        other_velocities = []
        for joint_name in self.other_joint_names:
            if joint_name == self.right_gripper_joint:
                other_velocities.append(self.right_gripper_cmd)
            elif joint_name == self.left_gripper_joint:
                other_velocities.append(self.left_gripper_cmd)
            else:
                other_velocities.append(0.0)
        msg.velocity = self.right_cmd + self.left_cmd + other_velocities
        msg.effort = []
        self.joint_state_pub.publish(msg)

    def publish_dummy_joint_velocities(self):
        right_msg = Float64MultiArray()
        right_msg.data = list(self.right_cmd)
        left_msg = Float64MultiArray()
        left_msg.data = list(self.left_cmd)
        self.right_joint_vel_pub.publish(right_msg)
        self.left_joint_vel_pub.publish(left_msg)

        right_gripper_msg = Float64()
        right_gripper_msg.data = self.right_gripper_cmd
        left_gripper_msg = Float64()
        left_gripper_msg.data = self.left_gripper_cmd
        self.right_gripper_pub.publish(right_gripper_msg)
        self.left_gripper_pub.publish(left_gripper_msg)

        rospy.loginfo_throttle(
            1.0,
            "Dummy Kinova cmd | active_arm=%s | right_arm=%s | left_arm=%s | right_gripper=%.3f(%.3f) | left_gripper=%.3f(%.3f)",
            self.active_arm,
            [round(v, 3) for v in self.right_cmd],
            [round(v, 3) for v in self.left_cmd],
            self.right_gripper_cmd,
            self.joint_positions[self.right_gripper_joint],
            self.left_gripper_cmd,
            self.joint_positions[self.left_gripper_joint],
        )

    def timer_cb(self, _event):
        if rospy.is_shutdown():
            return

        now = rospy.Time.now()
        dt = (now - self.last_tick).to_sec()
        self.last_tick = now
        if dt <= 0.0:
            return

        try:
            self.integrate(dt)
            self.publish_joint_state()
            self.publish_dummy_joint_velocities()
        except rospy.ROSException:
            # Expected during node shutdown when topics are being torn down.
            return


def main():
    rospy.init_node("fake_kinova_command_bridge")
    FakeKinovaCommandBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
