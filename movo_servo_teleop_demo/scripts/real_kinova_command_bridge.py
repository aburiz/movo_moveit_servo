#!/usr/bin/env python3

import actionlib
import math
import threading

import rospy
import rosservice
from actionlib_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from kinova_msgs.msg import ArmJointAnglesAction, ArmJointAnglesGoal, JointVelocity
from kinova_msgs.srv import ClearTrajectories, Start, Stop
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String


class RealKinovaCommandBridge:
    def __init__(self):
        self.lock = threading.RLock()
        self.config_namespace = rospy.get_param("~config_namespace", "/movo_servo_teleop_demo/real_arms").rstrip("/")

        self.publish_rate_hz = float(self.cfg("bridge/publish_rate_hz", 100.0))
        self.command_timeout_sec = float(self.cfg("bridge/command_timeout_sec", 0.15))
        self.feedback_timeout_sec = float(self.cfg("bridge/feedback_timeout_sec", 1.0))
        self.home_lockout_sec = float(self.cfg("bridge/home_lockout_sec", 0.0))
        self.home_action_server_timeout_sec = float(self.cfg("bridge/home_action_server_timeout_sec", 1.0))
        self.home_action_result_timeout_sec = float(self.cfg("bridge/home_action_result_timeout_sec", 45.0))
        self.post_home_clear_trajectories = bool(self.cfg("bridge/post_home_clear_trajectories", True))
        self.post_home_clear_delay_sec = float(self.cfg("bridge/post_home_clear_delay_sec", 0.2))
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
        rospy.loginfo("Home requests use configured custom joint-angle goals via the Kinova joint-angle action server.")

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
        home_joint_angles_deg = self.parse_home_joint_angles_deg(arm_name)
        joint_velocity_signs = self.parse_joint_velocity_signs(arm_name)
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
            "joint_velocity_signs": joint_velocity_signs,
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
            "clear_service_name": driver_namespace + "/in/clear_trajectories",
            "home_action_name": driver_namespace + "/joints_action/joint_angles",
            "feedback_topic": driver_namespace + "/out/joint_state",
            "home_joint_angles_deg": home_joint_angles_deg,
            "home_in_progress": False,
            "last_home_result": "idle",
        }
        state["pub"] = rospy.Publisher(driver_namespace + "/in/joint_velocity", JointVelocity, queue_size=1)
        state["start_proxy"] = rospy.ServiceProxy(state["start_service_name"], Start)
        state["stop_proxy"] = rospy.ServiceProxy(state["stop_service_name"], Stop)
        state["clear_proxy"] = rospy.ServiceProxy(state["clear_service_name"], ClearTrajectories)
        state["home_client"] = actionlib.SimpleActionClient(state["home_action_name"], ArmJointAnglesAction)
        state["feedback_sub"] = rospy.Subscriber(
            state["feedback_topic"],
            JointState,
            lambda msg, arm=arm_name: self.feedback_cb(arm, msg),
            queue_size=10,
        )
        return state

    def parse_home_joint_angles_deg(self, arm_name):
        raw_angles = self.cfg("{0}_arm/home_joint_angles_deg".format(arm_name), {})
        ordered_angles = []
        missing = []
        for idx in range(7):
            key = "joint{0}".format(idx + 1)
            if key not in raw_angles:
                missing.append(key)
                continue
            ordered_angles.append(float(raw_angles[key]))
        if missing:
            rospy.logwarn(
                "%s arm custom home pose is missing %s; home requests for this arm will be ignored.",
                arm_name,
                ", ".join(missing),
            )
            return []
        return ordered_angles

    def parse_joint_velocity_signs(self, arm_name):
        raw_signs = list(self.cfg("{0}_arm/joint_velocity_signs".format(arm_name), [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
        if len(raw_signs) != 7:
            rospy.logwarn(
                "%s arm joint_velocity_signs expected 7 entries, got %d. Falling back to actuator-1 inversion only.",
                arm_name,
                len(raw_signs),
            )
            raw_signs = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        return [float(value) for value in raw_signs]

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
        with self.lock:
            if self.estop_latched:
                rospy.logwarn("Ignoring custom home request while estop is latched.")
                return

            for arm_name in targets:
                arm_state = self.arms[arm_name]
                if arm_state["home_in_progress"]:
                    rospy.logwarn("Ignoring %s arm home request because a custom home is already in progress.", arm_name)
                    return
                if len(arm_state["home_joint_angles_deg"]) != 7:
                    rospy.logwarn("Ignoring %s arm home request because no complete custom home pose is configured.", arm_name)
                    return

            for arm_name in targets:
                arm_state = self.arms[arm_name]
                arm_state["home_in_progress"] = True
                arm_state["last_home_result"] = "running"

        self.publish_zero_once()
        for arm_name in targets:
            thread = threading.Thread(target=self.execute_custom_home, args=(arm_name,))
            thread.daemon = True
            thread.start()

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
            elif service_kind == "clear":
                rospy.loginfo("Cleared queued trajectories on %s arm via %s", arm_name, service_name)
            return True
        except rospy.ServiceException as exc:
            rospy.logwarn("Service call failed for %s arm (%s): %s", arm_name, service_name, exc)
            return False

    @staticmethod
    def make_joint_angles_goal(command_deg):
        goal = ArmJointAnglesGoal()
        padded = RealKinovaCommandBridge.resize_command(command_deg, 7)
        for idx, value in enumerate(padded):
            setattr(goal.angles, "joint{0}".format(idx + 1), float(value))
        return goal

    def execute_custom_home(self, arm_name):
        try:
            with self.lock:
                arm_state = self.arms[arm_name]
                target_deg = list(arm_state["home_joint_angles_deg"])
                action_name = arm_state["home_action_name"]
                action_client = arm_state["home_client"]

            if not self.call_service(arm_name, "start"):
                with self.lock:
                    self.arms[arm_name]["last_home_result"] = "start_failed"
                return

            if not action_client.wait_for_server(rospy.Duration(self.home_action_server_timeout_sec)):
                rospy.logwarn("Custom home action unavailable for %s arm: %s", arm_name, action_name)
                with self.lock:
                    self.arms[arm_name]["last_home_result"] = "action_unavailable"
                return

            goal = self.make_joint_angles_goal(target_deg)
            rospy.loginfo("Sending custom home pose to %s arm via %s: %s", arm_name, action_name, target_deg)
            action_client.send_goal(goal)

            finished = action_client.wait_for_result(rospy.Duration(self.home_action_result_timeout_sec))
            if not finished:
                rospy.logwarn("Custom home timed out for %s arm after %.1f s", arm_name, self.home_action_result_timeout_sec)
                action_client.cancel_goal()
                with self.lock:
                    self.arms[arm_name]["last_home_result"] = "timeout"
                return

            goal_state = action_client.get_state()
            if goal_state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Custom home reached for %s arm", arm_name)
                with self.lock:
                    self.arms[arm_name]["last_home_result"] = "succeeded"
            else:
                rospy.logwarn(
                    "Custom home did not succeed for %s arm (action state=%s)",
                    arm_name,
                    str(goal_state),
                )
                with self.lock:
                    self.arms[arm_name]["last_home_result"] = "action_state_{0}".format(goal_state)
        finally:
            self.publish_zero_once()
            if self.post_home_clear_trajectories:
                if self.post_home_clear_delay_sec > 0.0:
                    rospy.sleep(self.post_home_clear_delay_sec)
                self.call_service(arm_name, "clear")
                self.publish_zero_once()
            with self.lock:
                should_restore_stop = not self.driver_enabled
            if should_restore_stop:
                self.call_service(arm_name, "stop")
            with self.lock:
                arm_state = self.arms[arm_name]
                arm_state["home_in_progress"] = False
                if self.home_lockout_sec > 0.0:
                    arm_state["home_lockout_until"] = rospy.Time.now() + rospy.Duration(self.home_lockout_sec)
                else:
                    arm_state["home_lockout_until"] = rospy.Time(0)

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
                lockout_active = arm_state["home_in_progress"] or (now < arm_state["home_lockout_until"])
                use_command = enabled and command_is_fresh and not lockout_active

                command_rad_s = list(arm_state["cmd_rad_s"]) if use_command else [0.0] * 7
                signed_command_rad_s = [
                    arm_state["joint_velocity_signs"][idx] * command_rad_s[idx] for idx in range(len(command_rad_s))
                ]
                command_deg_s = [math.degrees(value) for value in signed_command_rad_s]
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
                    "clear": arm_state["clear_service_name"] in service_names,
                }
                home_action_connected = arm_state["home_client"].wait_for_server(rospy.Duration(0.0))

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

                if arm_state["home_in_progress"]:
                    status.level = DiagnosticStatus.WARN
                    status.message = "Custom home in progress"
                elif self.estop_latched:
                    status.level = DiagnosticStatus.WARN
                    status.message = "ROS estop latched"
                elif not self.driver_enabled:
                    status.level = DiagnosticStatus.WARN
                    status.message = "Waiting for start command"
                elif not service_present["start"] or not service_present["stop"]:
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
                    KeyValue(
                        "home_lockout_active",
                        str(bool(arm_state["home_in_progress"] or (now < arm_state["home_lockout_until"]))).lower(),
                    ),
                    KeyValue("home_in_progress", str(bool(arm_state["home_in_progress"])).lower()),
                    KeyValue("start_service_present", str(bool(service_present["start"])).lower()),
                    KeyValue("stop_service_present", str(bool(service_present["stop"])).lower()),
                    KeyValue("clear_trajectories_service_present", str(bool(service_present["clear"])).lower()),
                    KeyValue("custom_home_action", arm_state["home_action_name"]),
                    KeyValue("custom_home_action_connected", str(bool(home_action_connected)).lower()),
                    KeyValue("custom_home_configured", str(bool(len(arm_state["home_joint_angles_deg"]) == 7)).lower()),
                    KeyValue("last_home_result", arm_state["last_home_result"]),
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
