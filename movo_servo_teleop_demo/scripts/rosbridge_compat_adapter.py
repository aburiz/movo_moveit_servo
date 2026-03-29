#!/usr/bin/env python3

import math
import threading

import rospy
from geometry_msgs.msg import TwistStamped
from kinova_msgs.msg import PoseVelocity, PoseVelocityWithFingerVelocity, PoseVelocityWithFingers
from kinova_msgs.srv import HomeArm, HomeArmResponse, Start, StartResponse, Stop, StopResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String


class RosbridgeCompatAdapter:
    def __init__(self):
        self.lock = threading.RLock()
        self.config_namespace = rospy.get_param("~config_namespace", "/movo_servo_teleop_demo/real_arms").rstrip("/")
        self.command_frame = str(rospy.get_param("~command_frame", "base_link")).strip() or "base_link"
        self.publish_driver_enabled_on_start = bool(rospy.get_param("~publish_driver_enabled_on_start", True))
        self.publish_estop_clear_on_start = bool(rospy.get_param("~publish_estop_clear_on_start", True))
        self.home_wait_timeout_sec = float(rospy.get_param("~home_wait_timeout_sec", 45.0))
        self.home_position_tolerance_deg = float(rospy.get_param("~home_position_tolerance_deg", 6.0))
        self.legacy_gripper_velocity_scale = float(rospy.get_param("~legacy_gripper_velocity_scale", 5000.0))

        self.driver_enabled_pub = rospy.Publisher(
            "/movo_servo_teleop_demo/driver_enabled", Bool, queue_size=2, latch=True
        )
        self.estop_pub = rospy.Publisher(
            "/movo_servo_teleop_demo/estop_latched", Bool, queue_size=2, latch=True
        )
        self.active_arm_pub = rospy.Publisher(
            "/movo_servo_teleop_demo/active_arm", String, queue_size=2, latch=True
        )
        self.home_request_pub = rospy.Publisher("/movo_servo_teleop_demo/home_request", String, queue_size=4)

        self.arms = {
            "right": self.make_arm_state("right"),
            "left": self.make_arm_state("left"),
        }

        if self.publish_driver_enabled_on_start:
            self.driver_enabled_pub.publish(Bool(data=True))
        if self.publish_estop_clear_on_start:
            self.estop_pub.publish(Bool(data=False))
        self.active_arm_pub.publish(String(data="right"))

        rospy.loginfo(
            "ROS bridge compatibility adapter ready. Public Kinova cartesian topics now feed MoveIt Servo in %s.",
            self.command_frame,
        )
        rospy.loginfo(
            "Bridge compatibility keeps the old ROS1 command surface while routing Cartesian arm motion through Servo."
        )

    def cfg(self, key, default=None):
        return rospy.get_param("{}/{}".format(self.config_namespace, key), default)

    def make_arm_state(self, arm_name):
        public_driver_namespace = str(
            rospy.get_param(
                "~{}/public_driver_namespace".format(arm_name),
                "/{0}_arm/{0}_arm_driver".format(arm_name),
            )
        ).rstrip("/")
        internal_driver_namespace = str(
            rospy.get_param(
                "~{}/internal_driver_namespace".format(arm_name),
                public_driver_namespace + "/internal",
            )
        ).rstrip("/")
        servo_twist_topic = str(
            rospy.get_param(
                "~{}/servo_twist_topic".format(arm_name),
                "/movo_servo_teleop_demo/{}/delta_twist_cmds".format(arm_name),
            )
        ).strip()
        servo_gripper_velocity_topic = str(
            rospy.get_param(
                "~{}/servo_gripper_velocity_topic".format(arm_name),
                "/movo_servo_teleop_demo/{}/gripper_velocity_cmd".format(arm_name),
            )
        ).strip()
        joint_names = list(self.cfg("{0}_arm/joint_names".format(arm_name), []))
        home_joint_angles_deg = self.parse_home_joint_angles_deg(arm_name)

        state = {
            "arm_name": arm_name,
            "public_driver_namespace": public_driver_namespace,
            "internal_driver_namespace": internal_driver_namespace,
            "joint_names": joint_names,
            "home_joint_angles_deg": home_joint_angles_deg,
            "motion_enabled": True,
            "latest_positions_by_name": {},
            "twist_pub": rospy.Publisher(servo_twist_topic, TwistStamped, queue_size=20),
            "gripper_velocity_pub": rospy.Publisher(servo_gripper_velocity_topic, Float64, queue_size=20),
            "start_proxy": rospy.ServiceProxy(internal_driver_namespace + "/in/start", Start),
            "stop_proxy": rospy.ServiceProxy(internal_driver_namespace + "/in/stop", Stop),
        }

        rospy.Subscriber(
            public_driver_namespace + "/in/cartesian_velocity",
            PoseVelocity,
            lambda msg, arm=arm_name: self.kinova_pose_velocity_cb(arm, msg),
            queue_size=20,
        )
        rospy.Subscriber(
            public_driver_namespace + "/in/cartesian_velocity_with_fingers",
            PoseVelocityWithFingers,
            lambda msg, arm=arm_name: self.kinova_pose_velocity_with_fingers_cb(arm, msg),
            queue_size=20,
        )
        rospy.Subscriber(
            public_driver_namespace + "/in/cartesian_velocity_with_finger_velocity",
            PoseVelocityWithFingerVelocity,
            lambda msg, arm=arm_name: self.kinova_pose_velocity_with_finger_velocity_cb(arm, msg),
            queue_size=20,
        )
        rospy.Subscriber(
            public_driver_namespace + "/out/joint_state",
            JointState,
            lambda msg, arm=arm_name: self.feedback_cb(arm, msg),
            queue_size=20,
        )

        rospy.Service(
            public_driver_namespace + "/in/home_arm",
            HomeArm,
            lambda _req, arm=arm_name: self.home_service_cb(arm),
        )
        rospy.Service(
            public_driver_namespace + "/in/start",
            Start,
            lambda _req, arm=arm_name: self.start_service_cb(arm),
        )
        rospy.Service(
            public_driver_namespace + "/in/stop",
            Stop,
            lambda _req, arm=arm_name: self.stop_service_cb(arm),
        )

        return state

    def parse_home_joint_angles_deg(self, arm_name):
        raw_angles = self.cfg("{0}_arm/home_joint_angles_deg".format(arm_name), {})
        ordered = []
        for idx in range(7):
            key = "joint{0}".format(idx + 1)
            if key not in raw_angles:
                return []
            ordered.append(float(raw_angles[key]))
        return ordered

    def feedback_cb(self, arm_name, msg):
        with self.lock:
            latest = self.arms[arm_name]["latest_positions_by_name"]
            for name, position in zip(msg.name, msg.position):
                latest[name] = float(position)

    def build_twist(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, frame_id=""):
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id or self.command_frame
        msg.twist.linear.x = float(linear_x)
        msg.twist.linear.y = float(linear_y)
        msg.twist.linear.z = float(linear_z)
        msg.twist.angular.x = float(angular_x)
        msg.twist.angular.y = float(angular_y)
        msg.twist.angular.z = float(angular_z)
        return msg

    def publish_zero_twist(self, arm_name):
        self.arms[arm_name]["twist_pub"].publish(self.build_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    def publish_twist_if_enabled(self, arm_name, twist_msg):
        with self.lock:
            enabled = self.arms[arm_name]["motion_enabled"]
        if enabled:
            self.arms[arm_name]["twist_pub"].publish(twist_msg)

    def kinova_pose_velocity_cb(self, arm_name, msg):
        twist = self.build_twist(
            msg.twist_linear_x,
            msg.twist_linear_y,
            msg.twist_linear_z,
            msg.twist_angular_x,
            msg.twist_angular_y,
            msg.twist_angular_z,
        )
        self.publish_twist_if_enabled(arm_name, twist)

    def kinova_pose_velocity_with_fingers_cb(self, arm_name, msg):
        self.kinova_pose_velocity_cb(arm_name, msg)
        rospy.logwarn_throttle(
            5.0,
            "%s PoseVelocityWithFingers routes arm motion through Servo, but fingers_closure_percentage is not mapped here. Use the bridged SetFingersPosition action for the real gripper.",
            arm_name,
        )

    def kinova_pose_velocity_with_finger_velocity_cb(self, arm_name, msg):
        self.kinova_pose_velocity_cb(arm_name, msg)
        avg_finger_velocity = (float(msg.finger1) + float(msg.finger2) + float(msg.finger3)) / 3.0
        self.publish_legacy_gripper_velocity(arm_name, avg_finger_velocity)

    def publish_legacy_gripper_velocity(self, arm_name, raw_velocity):
        if self.legacy_gripper_velocity_scale <= 0.0:
            return
        normalized = float(raw_velocity) / self.legacy_gripper_velocity_scale
        normalized = max(min(normalized, 1.0), -1.0)
        self.arms[arm_name]["gripper_velocity_pub"].publish(Float64(data=normalized))

    def call_internal_service(self, arm_name, service_key):
        arm_state = self.arms[arm_name]
        proxy = arm_state[service_key + "_proxy"]
        service_name = arm_state["internal_driver_namespace"] + "/in/" + service_key
        try:
            rospy.wait_for_service(service_name, timeout=0.5)
            proxy()
            return True
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn_throttle(2.0, "%s arm could not call %s: %s", arm_name, service_name, exc)
            return False

    def start_service_cb(self, arm_name):
        with self.lock:
            self.arms[arm_name]["motion_enabled"] = True
        self.call_internal_service(arm_name, "start")
        return StartResponse(start_result="start accepted for {} arm compatibility adapter".format(arm_name))

    def stop_service_cb(self, arm_name):
        with self.lock:
            self.arms[arm_name]["motion_enabled"] = False
        self.publish_zero_twist(arm_name)
        self.call_internal_service(arm_name, "stop")
        return StopResponse(stop_result="stop accepted for {} arm compatibility adapter".format(arm_name))

    def request_home(self, arm_name):
        with self.lock:
            arm_state = self.arms[arm_name]
            previous_enabled = arm_state["motion_enabled"]
            arm_state["motion_enabled"] = False

        self.publish_zero_twist(arm_name)
        self.home_request_pub.publish(String(data=arm_name))

        success = self.wait_for_home_position(arm_name)
        with self.lock:
            self.arms[arm_name]["motion_enabled"] = previous_enabled

        if success:
            return True, "custom home completed for {} arm".format(arm_name)
        return False, "custom home requested for {} arm, but completion could not be confirmed".format(arm_name)

    def wait_for_home_position(self, arm_name):
        arm_state = self.arms[arm_name]
        joint_names = arm_state["joint_names"]
        home_joint_angles_deg = arm_state["home_joint_angles_deg"]
        if len(joint_names) != 7 or len(home_joint_angles_deg) != 7:
            return False

        deadline = rospy.Time.now() + rospy.Duration(self.home_wait_timeout_sec)
        tolerance_rad = math.radians(self.home_position_tolerance_deg)

        while (not rospy.is_shutdown()) and rospy.Time.now() < deadline:
            with self.lock:
                positions = dict(arm_state["latest_positions_by_name"])
            if all(name in positions for name in joint_names):
                close_enough = True
                for joint_name, target_deg in zip(joint_names, home_joint_angles_deg):
                    if abs(float(positions[joint_name]) - math.radians(target_deg)) > tolerance_rad:
                        close_enough = False
                        break
                if close_enough:
                    return True
            rospy.sleep(0.05)
        return False

    def home_service_cb(self, arm_name):
        success, message = self.request_home(arm_name)
        if not success:
            rospy.logwarn("%s", message)
        return HomeArmResponse(homearm_result=message)


if __name__ == "__main__":
    rospy.init_node("rosbridge_compat_adapter")
    RosbridgeCompatAdapter()
    rospy.spin()
