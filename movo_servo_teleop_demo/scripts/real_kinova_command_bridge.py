#!/usr/bin/env python3

import math
import threading

import rospy
import rosservice
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from kinova_msgs.msg import JointVelocity
from kinova_msgs.srv import HomeArm, Start, Stop
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String


class RealKinovaCommandBridge:
    def __init__(self):
        self.lock = threading.RLock()
        self.config_namespace = rospy.get_param("~config_namespace", "/movo_servo_teleop_demo/real_arms").rstrip("/")

        self.publish_rate_hz = float(self.cfg("bridge/publish_rate_hz", 100.0))
        self.command_timeout_sec = float(self.cfg("bridge/command_timeout_sec", 0.15))
        self.feedback_timeout_sec = float(self.cfg("bridge/feedback_timeout_sec", 1.0))
        self.home_lockout_sec = float(self.cfg("bridge/home_lockout_sec", 3.0))
        self.status_publish_rate_hz = float(self.cfg("bridge/status_publish_rate_hz", 5.0))
        self.status_topic = self.cfg("bridge/status_topic", "/movo_servo_teleop_demo/real_arm_status")
        self.driver_enabled_topic = self.cfg("bridge/driver_enabled_topic", "/movo_servo_teleop_demo/driver_enabled")
        self.estop_topic = self.cfg("bridge/estop_topic", "/movo_servo_teleop_demo/estop_latched")
        self.home_request_topic = self.cfg("bridge/home_request_topic", "/movo_servo_teleop_demo/home_request")
        self.active_arm_topic = self.cfg("bridge/active_arm_topic", "/movo_servo_teleop_demo/active_arm")
        self.right_joint_velocity_topic = self.cfg(
            "bridge/right_joint_velocity_topic",
            "/movo_servo_teleop_demo/right/joint_velocity_cmd",
        )
        self.left_joint_velocity_topic = self.cfg(
            "bridge/left_joint_velocity_topic",
            "/movo_servo_teleop_demo/left/joint_velocity_cmd",
        )

        self.local_machine_ip = str(self.cfg("local_machine_ip", "")).strip()
        self.subnet_mask = str(self.cfg("subnet_mask", "255.255.255.0")).strip()
        self.driver_enabled = False
        self.driver_enabled_initialized = False
        self.estop_latched = False
        self.estop_initialized = False
        self.active_arm = "right"

        self.status_pub = rospy.Publisher(self.status_topic, DiagnosticArray, queue_size=10)
        self.arms = {
            "right": self.make_arm_state("right"),
            "left": self.make_arm_state("left"),
        }

        rospy.Subscriber(self.right_joint_velocity_topic, Float64MultiArray, self.right_joint_velocity_cb, queue_size=20)
        rospy.Subscriber(self.left_joint_velocity_topic, Float64MultiArray, self.left_joint_velocity_cb, queue_size=20)
        rospy.Subscriber(self.driver_enabled_topic, Bool, self.driver_enabled_cb, queue_size=2)
        rospy.Subscriber(self.estop_topic, Bool, self.estop_cb, queue_size=2)
        rospy.Subscriber(self.home_request_topic, String, self.home_request_cb, queue_size=4)
        rospy.Subscriber(self.active_arm_topic, String, self.active_arm_cb, queue_size=2)

        self.command_timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate_hz), self.command_timer_cb)
        self.status_timer = rospy.Timer(rospy.Duration(1.0 / self.status_publish_rate_hz), self.status_timer_cb)
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo(
            "Real Kinova bridge ready | local_machine_ip=%s | right=%s -> %s | left=%s -> %s",
            self.local_machine_ip if self.local_machine_ip else "<unset>",
            self.arms["right"]["driver_namespace"],
            self.arms["right"]["target_ip"],
            self.arms["left"]["driver_namespace"],
            self.arms["left"]["target_ip"],
        )
        rospy.loginfo(
            "Servo joint velocities are treated as rad/s and converted to deg/s for kinova_msgs/JointVelocity."
        )

    def cfg(self, key, default=None):
        full_name = "{}/{}".format(self.config_namespace, key)
        return rospy.get_param(full_name, default)

    def make_arm_state(self, arm_name):
        driver_namespace = str(
            self.cfg("{0}_arm/driver_namespace".format(arm_name), "/{0}_arm/{0}_arm_driver".format(arm_name))
        ).rstrip("/")
        target_ip = str(self.cfg("{0}_arm/robot_ip".format(arm_name), "")).strip()
        robot_name = str(self.cfg("{0}_arm/robot_name".format(arm_name), "{0}_arm".format(arm_name))).strip()
        joint_names = list(self.cfg("{0}_arm/joint_names".format(arm_name), []))
        if len(joint_names) != 7:
            rospy.logwarn(
                "%s arm joint_names expected 7 entries for a Jaco2 7-DOF arm, got %d",
                arm_name,
                len(joint_names),
            )

        state = {
            "arm_name": arm_name,
            "robot_name": robot_name,
            "driver_namespace": driver_namespace,
            "target_ip": target_ip,
            "joint_names": joint_names,
            "cmd_rad_s": [0.0] * 7,
            "last_cmd_time": rospy.Time(0),
            "last_feedback_time": rospy.Time(0),
            "feedback_received": False,
            "home_lockout_until": rospy.Time(0),
            "started": False,
            "latest_max_abs_rad_s": 0.0,
            "latest_max_abs_deg_s": 0.0,
            "start_service_name": driver_namespace + "/in/start",
            "stop_service_name": driver_namespace + "/in/stop",
            "home_service_name": driver_namespace + "/in/home_arm",
            "feedback_topic": driver_namespace + "/out/joint_state",
        }
        state["pub"] = rospy.Publisher(driver_namespace + "/in/joint_velocity", JointVelocity, queue_size=1)
        state["start_proxy"] = rospy.ServiceProxy(state["start_service_name"], Start)
        state["stop_proxy"] = rospy.ServiceProxy(state["stop_service_name"], Stop)
        state["home_proxy"] = rospy.ServiceProxy(state["home_service_name"], HomeArm)
        state["feedback_sub"] = rospy.Subscriber(
            state["feedback_topic"],
            JointState,
            lambda msg, arm=arm_name: self.feedback_cb(arm, msg),
            queue_size=10,
        )
        return state

    @staticmethod
    def normalize_arm_name(raw_name):
        arm_name = str(raw_name).strip().lower()
        if arm_name in ("left", "left_arm"):
            return "left"
        if arm_name in ("right", "right_arm"):
            return "right"
        if arm_name == "both":
            return "both"
        return ""

    @staticmethod
    def resize_command(raw_values, expected_len):
        command = [0.0] * expected_len
        for idx, value in enumerate(raw_values):
            if idx >= expected_len:
                break
            command[idx] = float(value)
        return command

    @staticmethod
    def make_joint_velocity_msg(command_deg_s):
        msg = JointVelocity()
        padded = RealKinovaCommandBridge.resize_command(command_deg_s, 7)
        for idx, value in enumerate(padded):
            setattr(msg, "joint{0}".format(idx + 1), float(value))
        return msg

    def feedback_cb(self, arm_name, _msg):
        with self.lock:
            arm_state = self.arms[arm_name]
            arm_state["feedback_received"] = True
            arm_state["last_feedback_time"] = rospy.Time.now()

    def right_joint_velocity_cb(self, msg):
        self.joint_velocity_cb("right", msg)

    def left_joint_velocity_cb(self, msg):
        self.joint_velocity_cb("left", msg)

    def joint_velocity_cb(self, arm_name, msg):
        command = self.resize_command(msg.data, 7)
        if len(msg.data) != 7:
            rospy.logwarn_throttle(
                2.0,
                "%s arm joint velocity command length %d does not match the 7-DOF Kinova interface",
                arm_name,
                len(msg.data),
            )

        with self.lock:
            arm_state = self.arms[arm_name]
            arm_state["cmd_rad_s"] = command
            arm_state["last_cmd_time"] = rospy.Time.now()
            arm_state["latest_max_abs_rad_s"] = max([abs(value) for value in command] or [0.0])

    def driver_enabled_cb(self, msg):
        enabled = bool(msg.data)
        with self.lock:
            previous_known = self.driver_enabled_initialized
            previous_value = self.driver_enabled
            self.driver_enabled = enabled
            self.driver_enabled_initialized = True

        if enabled and ((not previous_known) or (not previous_value)):
            self.call_service_for_all("start")
        elif (not enabled) and previous_known and previous_value:
            self.publish_zero_once()
            self.call_service_for_all("stop")

    def estop_cb(self, msg):
        estop_latched = bool(msg.data)
        with self.lock:
            previous_known = self.estop_initialized
            previous_value = self.estop_latched
            self.estop_latched = estop_latched
            self.estop_initialized = True

        if estop_latched and ((not previous_known) or (not previous_value)):
            self.publish_zero_once()
            self.call_service_for_all("stop")

    def active_arm_cb(self, msg):
        arm_name = self.normalize_arm_name(msg.data)
        if arm_name in ("left", "right"):
            with self.lock:
                self.active_arm = arm_name

    def home_request_cb(self, msg):
        requested = self.normalize_arm_name(msg.data)
        if requested not in ("left", "right", "both"):
            return

        targets = ("left", "right") if requested == "both" else (requested,)
        now = rospy.Time.now()

        with self.lock:
            for arm_name in targets:
                self.arms[arm_name]["home_lockout_until"] = now + rospy.Duration(self.home_lockout_sec)

        self.publish_zero_once()
        for arm_name in targets:
            self.call_service(arm_name, "home")

    def call_service_for_all(self, service_kind):
        for arm_name in ("right", "left"):
            self.call_service(arm_name, service_kind)

    def call_service(self, arm_name, service_kind):
        with self.lock:
            arm_state = self.arms[arm_name]
            service_name = arm_state["{0}_service_name".format(service_kind)]
            service_proxy = arm_state["{0}_proxy".format(service_kind)]

        try:
            rospy.wait_for_service(service_name, timeout=0.5)
        except rospy.ROSException:
            rospy.logwarn("Service unavailable for %s arm: %s", arm_name, service_name)
            return False

        try:
            service_proxy()
            rospy.loginfo("Called %s on %s arm via %s", service_kind, arm_name, service_name)
            if service_kind == "start":
                with self.lock:
                    arm_state["started"] = True
            elif service_kind == "stop":
                with self.lock:
                    arm_state["started"] = False
            return True
        except rospy.ServiceException as exc:
            rospy.logwarn("Service call failed for %s arm (%s): %s", arm_name, service_name, exc)
            return False

    def publish_zero_once(self):
        zero_msg = self.make_joint_velocity_msg([0.0] * 7)
        with self.lock:
            publishers = [self.arms["right"]["pub"], self.arms["left"]["pub"]]
        for publisher in publishers:
            publisher.publish(zero_msg)

    def command_timer_cb(self, _event):
        if rospy.is_shutdown():
            return

        now = rospy.Time.now()
        commands_to_publish = []

        with self.lock:
            enabled = self.driver_enabled and not self.estop_latched
            for arm_name, arm_state in self.arms.items():
                command_age = float("inf")
                if arm_state["last_cmd_time"].to_sec() > 0.0:
                    command_age = (now - arm_state["last_cmd_time"]).to_sec()

                command_is_fresh = command_age <= self.command_timeout_sec
                lockout_active = now < arm_state["home_lockout_until"]
                use_command = enabled and command_is_fresh and not lockout_active

                command_rad_s = list(arm_state["cmd_rad_s"]) if use_command else [0.0] * 7
                command_deg_s = [math.degrees(value) for value in command_rad_s]
                arm_state["latest_max_abs_deg_s"] = max([abs(value) for value in command_deg_s] or [0.0])
                commands_to_publish.append((arm_state["pub"], self.make_joint_velocity_msg(command_deg_s)))

        for publisher, message in commands_to_publish:
            publisher.publish(message)

    def status_timer_cb(self, _event):
        now = rospy.Time.now()
        try:
            service_names = set(rosservice.get_service_list())
        except Exception:
            service_names = set()

        status_array = DiagnosticArray()
        status_array.header.stamp = now

        with self.lock:
            for arm_name, arm_state in self.arms.items():
                service_present = {
                    "start": arm_state["start_service_name"] in service_names,
                    "stop": arm_state["stop_service_name"] in service_names,
                    "home": arm_state["home_service_name"] in service_names,
                }

                if arm_state["feedback_received"]:
                    feedback_age = (now - arm_state["last_feedback_time"]).to_sec()
                else:
                    feedback_age = float("inf")
                connected = arm_state["feedback_received"] and feedback_age <= self.feedback_timeout_sec

                if arm_state["last_cmd_time"].to_sec() > 0.0:
                    command_age = (now - arm_state["last_cmd_time"]).to_sec()
                else:
                    command_age = float("inf")

                status = DiagnosticStatus()
                status.name = "movo_servo_teleop_demo/{0}_arm".format(arm_name)
                status.hardware_id = arm_state["target_ip"] or arm_state["driver_namespace"]

                if self.estop_latched:
                    status.level = DiagnosticStatus.WARN
                    status.message = "ROS estop latched"
                elif not self.driver_enabled:
                    status.level = DiagnosticStatus.WARN
                    status.message = "Waiting for start command"
                elif not all(service_present.values()):
                    status.level = DiagnosticStatus.ERROR
                    status.message = "Driver services missing"
                elif not connected:
                    status.level = DiagnosticStatus.WARN
                    status.message = "No recent feedback from driver"
                else:
                    status.level = DiagnosticStatus.OK
                    status.message = "Streaming joint velocity commands"

                status.values = [
                    KeyValue("arm_name", arm_name),
                    KeyValue("active_arm", self.active_arm),
                    KeyValue("driver_namespace", arm_state["driver_namespace"]),
                    KeyValue("target_ip", arm_state["target_ip"]),
                    KeyValue("local_machine_ip", self.local_machine_ip),
                    KeyValue("subnet_mask", self.subnet_mask),
                    KeyValue("connected", str(bool(connected)).lower()),
                    KeyValue("started", str(bool(arm_state["started"])).lower()),
                    KeyValue("estop", str(bool(self.estop_latched)).lower()),
                    KeyValue("feedback_received", str(bool(arm_state["feedback_received"])).lower()),
                    KeyValue("home_lockout_active", str(bool(now < arm_state["home_lockout_until"])).lower()),
                    KeyValue("start_service_present", str(bool(service_present["start"])).lower()),
                    KeyValue("stop_service_present", str(bool(service_present["stop"])).lower()),
                    KeyValue("home_service_present", str(bool(service_present["home"])).lower()),
                    KeyValue(
                        "last_command_age_sec",
                        "{0:.3f}".format(command_age) if math.isfinite(command_age) else "inf",
                    ),
                    KeyValue(
                        "last_feedback_age_sec",
                        "{0:.3f}".format(feedback_age) if math.isfinite(feedback_age) else "inf",
                    ),
                    KeyValue("max_abs_command_rad_s", "{0:.6f}".format(arm_state["latest_max_abs_rad_s"])),
                    KeyValue("max_abs_command_deg_s", "{0:.6f}".format(arm_state["latest_max_abs_deg_s"])),
                ]
                status_array.status.append(status)

        self.status_pub.publish(status_array)

    def on_shutdown(self):
        self.publish_zero_once()
        self.publish_zero_once()
        self.call_service_for_all("stop")


def main():
    rospy.init_node("real_kinova_command_bridge")
    RealKinovaCommandBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
