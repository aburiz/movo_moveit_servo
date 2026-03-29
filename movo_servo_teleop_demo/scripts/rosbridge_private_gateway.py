#!/usr/bin/env python3

import actionlib
import json
import math
import socket
import socketserver
import threading
from xmlrpc.server import SimpleXMLRPCServer

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from kinova_msgs.msg import (
    ArmJointAnglesAction,
    ArmJointAnglesGoal,
    ArmPoseAction,
    ArmPoseGoal,
    FingerPosition,
    JointAngles,
    SetFingersPositionAction,
    SetFingersPositionGoal,
)
from kinova_msgs.srv import ClearTrajectories, Start, Stop
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String


class ThreadedXMLRPCServer(socketserver.ThreadingMixIn, SimpleXMLRPCServer):
    daemon_threads = True
    allow_reuse_address = True


class RosbridgePrivateGateway:
    def __init__(self):
        self.lock = threading.RLock()
        self.config_namespace = rospy.get_param("~config_namespace", "/movo_servo_teleop_demo/real_arms").rstrip("/")
        self.command_frame = str(rospy.get_param("~command_frame", "base_link")).strip() or "base_link"
        self.publish_driver_enabled_on_start = bool(rospy.get_param("~publish_driver_enabled_on_start", True))
        self.publish_estop_clear_on_start = bool(rospy.get_param("~publish_estop_clear_on_start", True))
        self.home_wait_timeout_sec = float(rospy.get_param("~home_wait_timeout_sec", 45.0))
        self.home_position_tolerance_deg = float(rospy.get_param("~home_position_tolerance_deg", 6.0))
        self.command_host = str(rospy.get_param("~private_command_host", "127.0.0.1")).strip() or "127.0.0.1"
        self.command_port = int(rospy.get_param("~private_command_port", 45931))
        self.rpc_host = str(rospy.get_param("~private_rpc_host", "127.0.0.1")).strip() or "127.0.0.1"
        self.rpc_port = int(rospy.get_param("~private_rpc_port", 45932))
        self.rpc_wait_timeout_sec = float(rospy.get_param("~private_rpc_timeout_sec", 2.0))

        self.driver_enabled_pub = rospy.Publisher(
            "/movo_servo_teleop_demo/driver_enabled", Bool, queue_size=2, latch=True
        )
        self.estop_pub = rospy.Publisher(
            "/movo_servo_teleop_demo/estop_latched", Bool, queue_size=2, latch=True
        )
        self.home_request_pub = rospy.Publisher("/movo_servo_teleop_demo/home_request", String, queue_size=4)

        self.arms = {
            "right": self.make_arm_state("right"),
            "left": self.make_arm_state("left"),
        }

        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.command_socket.bind((self.command_host, self.command_port))
        self.command_socket.settimeout(0.5)

        self.rpc_server = ThreadedXMLRPCServer(
            (self.rpc_host, self.rpc_port), allow_none=False, logRequests=False
        )
        self.rpc_server.timeout = 0.5
        self.rpc_server.register_function(self.rpc_ping, "ping")
        self.rpc_server.register_function(self.rpc_start_arm, "start_arm")
        self.rpc_server.register_function(self.rpc_stop_arm, "stop_arm")
        self.rpc_server.register_function(self.rpc_home_arm, "home_arm")
        self.rpc_server.register_function(self.rpc_clear_trajectories, "clear_trajectories")
        self.rpc_server.register_function(self.rpc_send_fingers_goal, "send_fingers_goal")
        self.rpc_server.register_function(self.rpc_send_joint_angles_goal, "send_joint_angles_goal")
        self.rpc_server.register_function(self.rpc_send_pose_goal, "send_pose_goal")
        self.rpc_server.register_function(self.rpc_get_feedback_snapshot, "get_feedback_snapshot")

        self.shutdown_requested = False
        self.command_thread = threading.Thread(target=self.command_loop, name="rosbridge_private_udp")
        self.command_thread.daemon = True
        self.command_thread.start()
        self.rpc_thread = threading.Thread(target=self.rpc_loop, name="rosbridge_private_rpc")
        self.rpc_thread.daemon = True
        self.rpc_thread.start()

        if self.publish_driver_enabled_on_start:
            self.driver_enabled_pub.publish(Bool(data=True))
        if self.publish_estop_clear_on_start:
            self.estop_pub.publish(Bool(data=False))

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo(
            "ROS bridge private gateway ready | private command udp=%s:%d | private rpc=%s:%d",
            self.command_host,
            self.command_port,
            self.rpc_host,
            self.rpc_port,
        )

    def cfg(self, key, default=None):
        return rospy.get_param("{}/{}".format(self.config_namespace, key), default)

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

    def make_arm_state(self, arm_name):
        driver_namespace = str(
            rospy.get_param(
                "~{}/private_driver_namespace".format(arm_name),
                "/{0}_arm/{0}_arm_driver".format(arm_name),
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
            "driver_namespace": driver_namespace,
            "joint_names": joint_names,
            "home_joint_angles_deg": home_joint_angles_deg,
            "motion_enabled": True,
            "latest_positions_by_name": {},
            "latest_joint_state": None,
            "latest_joint_angles": None,
            "latest_finger_position": None,
            "latest_tool_pose": None,
            "twist_pub": rospy.Publisher(servo_twist_topic, TwistStamped, queue_size=20),
            "gripper_velocity_pub": rospy.Publisher(servo_gripper_velocity_topic, Float64, queue_size=20),
            "start_proxy": rospy.ServiceProxy(driver_namespace + "/in/start", Start),
            "stop_proxy": rospy.ServiceProxy(driver_namespace + "/in/stop", Stop),
            "clear_proxy": rospy.ServiceProxy(driver_namespace + "/in/clear_trajectories", ClearTrajectories),
            "fingers_client": actionlib.SimpleActionClient(
                driver_namespace + "/fingers_action/finger_positions", SetFingersPositionAction
            ),
            "joint_angles_client": actionlib.SimpleActionClient(
                driver_namespace + "/joints_action/joint_angles", ArmJointAnglesAction
            ),
            "tool_pose_client": actionlib.SimpleActionClient(
                driver_namespace + "/pose_action/tool_pose", ArmPoseAction
            ),
        }

        rospy.Subscriber(
            driver_namespace + "/out/joint_state",
            JointState,
            lambda msg, arm=arm_name: self.joint_state_cb(arm, msg),
            queue_size=10,
        )
        rospy.Subscriber(
            driver_namespace + "/out/joint_angles",
            JointAngles,
            lambda msg, arm=arm_name: self.joint_angles_cb(arm, msg),
            queue_size=10,
        )
        rospy.Subscriber(
            driver_namespace + "/out/finger_position",
            FingerPosition,
            lambda msg, arm=arm_name: self.finger_position_cb(arm, msg),
            queue_size=10,
        )
        rospy.Subscriber(
            driver_namespace + "/out/tool_pose",
            PoseStamped,
            lambda msg, arm=arm_name: self.tool_pose_cb(arm, msg),
            queue_size=10,
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

    def joint_state_cb(self, arm_name, msg):
        snapshot = {
            "stamp": msg.header.stamp.to_sec(),
            "frame_id": msg.header.frame_id,
            "name": list(msg.name),
            "position": [float(value) for value in msg.position],
            "velocity": [float(value) for value in msg.velocity],
            "effort": [float(value) for value in msg.effort],
        }
        with self.lock:
            arm_state = self.arms[arm_name]
            arm_state["latest_joint_state"] = snapshot
            latest = arm_state["latest_positions_by_name"]
            for name, position in zip(msg.name, msg.position):
                latest[name] = float(position)

    def joint_angles_cb(self, arm_name, msg):
        with self.lock:
            self.arms[arm_name]["latest_joint_angles"] = {
                "joint1": float(msg.joint1),
                "joint2": float(msg.joint2),
                "joint3": float(msg.joint3),
                "joint4": float(msg.joint4),
                "joint5": float(msg.joint5),
                "joint6": float(msg.joint6),
                "joint7": float(msg.joint7),
            }

    def finger_position_cb(self, arm_name, msg):
        with self.lock:
            self.arms[arm_name]["latest_finger_position"] = {
                "finger1": float(msg.finger1),
                "finger2": float(msg.finger2),
                "finger3": float(msg.finger3),
            }

    def tool_pose_cb(self, arm_name, msg):
        with self.lock:
            self.arms[arm_name]["latest_tool_pose"] = {
                "stamp": msg.header.stamp.to_sec(),
                "frame_id": msg.header.frame_id,
                "position": {
                    "x": float(msg.pose.position.x),
                    "y": float(msg.pose.position.y),
                    "z": float(msg.pose.position.z),
                },
                "orientation": {
                    "x": float(msg.pose.orientation.x),
                    "y": float(msg.pose.orientation.y),
                    "z": float(msg.pose.orientation.z),
                    "w": float(msg.pose.orientation.w),
                },
            }

    def build_twist(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.command_frame
        msg.twist.linear.x = float(linear_x)
        msg.twist.linear.y = float(linear_y)
        msg.twist.linear.z = float(linear_z)
        msg.twist.angular.x = float(angular_x)
        msg.twist.angular.y = float(angular_y)
        msg.twist.angular.z = float(angular_z)
        return msg

    def publish_zero_twist(self, arm_name):
        self.arms[arm_name]["twist_pub"].publish(self.build_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    def publish_zero_gripper(self, arm_name):
        self.arms[arm_name]["gripper_velocity_pub"].publish(Float64(data=0.0))

    def command_loop(self):
        while (not rospy.is_shutdown()) and (not self.shutdown_requested):
            try:
                payload, _addr = self.command_socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                return

            try:
                command = json.loads(payload.decode("utf-8"))
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "Ignoring invalid rosbridge private UDP payload: %s", exc)
                continue
            self.handle_command(command)

    def handle_command(self, command):
        arm_name = self.normalize_arm_name(command.get("arm", ""))
        if arm_name not in self.arms:
            return
        if not bool(command.get("motion_enabled", True)):
            return

        with self.lock:
            arm_state = self.arms[arm_name]
            motion_enabled = arm_state["motion_enabled"]
        if not motion_enabled:
            return

        kind = str(command.get("kind", "")).strip().lower()
        if kind != "cartesian_velocity":
            return

        linear = command.get("linear", [0.0, 0.0, 0.0])
        angular = command.get("angular", [0.0, 0.0, 0.0])
        gripper_velocity = command.get("gripper_velocity")
        try:
            twist_msg = self.build_twist(
                linear[0], linear[1], linear[2], angular[0], angular[1], angular[2]
            )
        except Exception:
            return
        arm_state["twist_pub"].publish(twist_msg)
        if gripper_velocity is not None:
            arm_state["gripper_velocity_pub"].publish(Float64(data=float(gripper_velocity)))

    def rpc_loop(self):
        while (not rospy.is_shutdown()) and (not self.shutdown_requested):
            self.rpc_server.handle_request()

    def call_arm_service(self, arm_name, service_kind):
        arm_state = self.arms[arm_name]
        service_name = arm_state["driver_namespace"] + "/in/" + service_kind
        proxy = arm_state[service_kind + "_proxy"]
        try:
            rospy.wait_for_service(service_name, timeout=self.rpc_wait_timeout_sec)
            proxy()
            return True, "called {} on {} arm".format(service_kind, arm_name)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            return False, "{} failed for {} arm: {}".format(service_kind, arm_name, exc)

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

    def execute_home(self, arm_name):
        arm_state = self.arms[arm_name]
        previous_motion_enabled = arm_state["motion_enabled"]
        with self.lock:
            arm_state["motion_enabled"] = False

        self.publish_zero_twist(arm_name)
        self.publish_zero_gripper(arm_name)
        self.home_request_pub.publish(String(data=arm_name))
        success = self.wait_for_home_position(arm_name)

        with self.lock:
            arm_state["motion_enabled"] = previous_motion_enabled
        if success:
            return True, "custom home completed for {} arm".format(arm_name)
        return False, "custom home requested for {} arm, but completion could not be confirmed".format(arm_name)

    def execute_fingers_goal(self, arm_name, finger_values, wait_timeout_sec):
        arm_state = self.arms[arm_name]
        if len(finger_values) != 3:
            return False, "expected 3 finger values", {}
        if not arm_state["fingers_client"].wait_for_server(rospy.Duration(self.rpc_wait_timeout_sec)):
            return False, "fingers action server unavailable for {} arm".format(arm_name), {}

        goal = SetFingersPositionGoal()
        goal.fingers.finger1 = float(finger_values[0])
        goal.fingers.finger2 = float(finger_values[1])
        goal.fingers.finger3 = float(finger_values[2])
        arm_state["fingers_client"].send_goal(goal)
        if not arm_state["fingers_client"].wait_for_result(rospy.Duration(wait_timeout_sec)):
            arm_state["fingers_client"].cancel_goal()
            return False, "timed out waiting for finger goal on {} arm".format(arm_name), {}

        result = arm_state["fingers_client"].get_result()
        if result is None:
            with self.lock:
                result_dict = dict(arm_state["latest_finger_position"] or {})
        else:
            result_dict = {
                "finger1": float(result.fingers.finger1),
                "finger2": float(result.fingers.finger2),
                "finger3": float(result.fingers.finger3),
            }
        return True, "finger goal completed for {} arm".format(arm_name), result_dict

    def execute_joint_angles_goal(self, arm_name, angle_values_deg, wait_timeout_sec):
        arm_state = self.arms[arm_name]
        if len(angle_values_deg) != 7:
            return False, "expected 7 joint angles", {}
        if not arm_state["joint_angles_client"].wait_for_server(rospy.Duration(self.rpc_wait_timeout_sec)):
            return False, "joint-angle action server unavailable for {} arm".format(arm_name), {}

        goal = ArmJointAnglesGoal()
        for index, value in enumerate(angle_values_deg):
            setattr(goal.angles, "joint{}".format(index + 1), float(value))
        arm_state["joint_angles_client"].send_goal(goal)
        if not arm_state["joint_angles_client"].wait_for_result(rospy.Duration(wait_timeout_sec)):
            arm_state["joint_angles_client"].cancel_goal()
            return False, "timed out waiting for joint-angle goal on {} arm".format(arm_name), {}

        result = arm_state["joint_angles_client"].get_result()
        if result is None:
            with self.lock:
                result_dict = dict(arm_state["latest_joint_angles"] or {})
        else:
            result_dict = {
                "joint1": float(result.angles.joint1),
                "joint2": float(result.angles.joint2),
                "joint3": float(result.angles.joint3),
                "joint4": float(result.angles.joint4),
                "joint5": float(result.angles.joint5),
                "joint6": float(result.angles.joint6),
                "joint7": float(result.angles.joint7),
            }
        return True, "joint-angle goal completed for {} arm".format(arm_name), result_dict

    def execute_pose_goal(self, arm_name, pose_dict, wait_timeout_sec):
        arm_state = self.arms[arm_name]
        if not arm_state["tool_pose_client"].wait_for_server(rospy.Duration(self.rpc_wait_timeout_sec)):
            return False, "tool-pose action server unavailable for {} arm".format(arm_name), {}

        goal = ArmPoseGoal()
        goal.pose.header.stamp = rospy.Time.now()
        goal.pose.header.frame_id = str(pose_dict.get("frame_id", self.command_frame))
        position = pose_dict.get("position", {})
        orientation = pose_dict.get("orientation", {})
        goal.pose.pose.position.x = float(position.get("x", 0.0))
        goal.pose.pose.position.y = float(position.get("y", 0.0))
        goal.pose.pose.position.z = float(position.get("z", 0.0))
        goal.pose.pose.orientation.x = float(orientation.get("x", 0.0))
        goal.pose.pose.orientation.y = float(orientation.get("y", 0.0))
        goal.pose.pose.orientation.z = float(orientation.get("z", 0.0))
        goal.pose.pose.orientation.w = float(orientation.get("w", 1.0))
        arm_state["tool_pose_client"].send_goal(goal)
        if not arm_state["tool_pose_client"].wait_for_result(rospy.Duration(wait_timeout_sec)):
            arm_state["tool_pose_client"].cancel_goal()
            return False, "timed out waiting for tool-pose goal on {} arm".format(arm_name), {}

        result = arm_state["tool_pose_client"].get_result()
        if result is None:
            with self.lock:
                result_dict = dict(arm_state["latest_tool_pose"] or {})
        else:
            result_dict = {
                "stamp": result.pose.header.stamp.to_sec(),
                "frame_id": result.pose.header.frame_id,
                "position": {
                    "x": float(result.pose.pose.position.x),
                    "y": float(result.pose.pose.position.y),
                    "z": float(result.pose.pose.position.z),
                },
                "orientation": {
                    "x": float(result.pose.pose.orientation.x),
                    "y": float(result.pose.pose.orientation.y),
                    "z": float(result.pose.pose.orientation.z),
                    "w": float(result.pose.pose.orientation.w),
                },
            }
        return True, "tool-pose goal completed for {} arm".format(arm_name), result_dict

    def make_simple_result(self, ok, message):
        return {"ok": bool(ok), "message": str(message)}

    def rpc_ping(self):
        return {
            "ok": True,
            "message": "private gateway alive",
            "command_host": self.command_host,
            "command_port": int(self.command_port),
            "rpc_host": self.rpc_host,
            "rpc_port": int(self.rpc_port),
        }

    def rpc_start_arm(self, arm_name):
        arm_name = self.normalize_arm_name(arm_name)
        if arm_name not in self.arms:
            return self.make_simple_result(False, "unknown arm")
        with self.lock:
            self.arms[arm_name]["motion_enabled"] = True
        ok, message = self.call_arm_service(arm_name, "start")
        return self.make_simple_result(ok, message)

    def rpc_stop_arm(self, arm_name):
        arm_name = self.normalize_arm_name(arm_name)
        if arm_name not in self.arms:
            return self.make_simple_result(False, "unknown arm")
        with self.lock:
            self.arms[arm_name]["motion_enabled"] = False
        self.publish_zero_twist(arm_name)
        self.publish_zero_gripper(arm_name)
        ok, message = self.call_arm_service(arm_name, "stop")
        return self.make_simple_result(ok, message)

    def rpc_home_arm(self, arm_name):
        arm_name = self.normalize_arm_name(arm_name)
        if arm_name not in self.arms:
            return self.make_simple_result(False, "unknown arm")
        ok, message = self.execute_home(arm_name)
        return self.make_simple_result(ok, message)

    def rpc_clear_trajectories(self, arm_name):
        arm_name = self.normalize_arm_name(arm_name)
        if arm_name not in self.arms:
            return self.make_simple_result(False, "unknown arm")
        ok, message = self.call_arm_service(arm_name, "clear")
        return self.make_simple_result(ok, message)

    def rpc_send_fingers_goal(self, arm_name, finger_values, wait_timeout_sec):
        arm_name = self.normalize_arm_name(arm_name)
        if arm_name not in self.arms:
            return {"ok": False, "message": "unknown arm", "fingers": {}}
        ok, message, result_dict = self.execute_fingers_goal(arm_name, list(finger_values), float(wait_timeout_sec))
        return {"ok": ok, "message": message, "fingers": result_dict}

    def rpc_send_joint_angles_goal(self, arm_name, angle_values_deg, wait_timeout_sec):
        arm_name = self.normalize_arm_name(arm_name)
        if arm_name not in self.arms:
            return {"ok": False, "message": "unknown arm", "angles": {}}
        ok, message, result_dict = self.execute_joint_angles_goal(
            arm_name, list(angle_values_deg), float(wait_timeout_sec)
        )
        return {"ok": ok, "message": message, "angles": result_dict}

    def rpc_send_pose_goal(self, arm_name, pose_dict, wait_timeout_sec):
        arm_name = self.normalize_arm_name(arm_name)
        if arm_name not in self.arms:
            return {"ok": False, "message": "unknown arm", "pose": {}}
        ok, message, result_dict = self.execute_pose_goal(arm_name, dict(pose_dict), float(wait_timeout_sec))
        return {"ok": ok, "message": message, "pose": result_dict}

    def rpc_get_feedback_snapshot(self):
        with self.lock:
            snapshot = {"ok": True, "arms": {}}
            for arm_name, arm_state in self.arms.items():
                snapshot["arms"][arm_name] = {
                    "motion_enabled": bool(arm_state["motion_enabled"]),
                    "joint_state": dict(arm_state["latest_joint_state"] or {}),
                    "joint_angles": dict(arm_state["latest_joint_angles"] or {}),
                    "finger_position": dict(arm_state["latest_finger_position"] or {}),
                    "tool_pose": dict(arm_state["latest_tool_pose"] or {}),
                }
        return snapshot

    def on_shutdown(self):
        self.shutdown_requested = True
        try:
            self.command_socket.close()
        except Exception:
            pass
        try:
            self.rpc_server.server_close()
        except Exception:
            pass
        for arm_name in ("right", "left"):
            self.publish_zero_twist(arm_name)
            self.publish_zero_gripper(arm_name)


if __name__ == "__main__":
    rospy.init_node("rosbridge_private_gateway")
    RosbridgePrivateGateway()
    rospy.spin()
