#!/usr/bin/env python3

import json
import socket
import threading
import xmlrpc.client

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
from kinova_msgs.msg import (
    ArmJointAnglesAction,
    ArmJointAnglesResult,
    ArmPoseAction,
    ArmPoseResult,
    FingerPosition,
    JointAngles,
    PoseVelocity,
    PoseVelocityWithFingerVelocity,
    PoseVelocityWithFingers,
    SetFingersPositionAction,
    SetFingersPositionResult,
)
from kinova_msgs.srv import (
    ClearTrajectories,
    ClearTrajectoriesResponse,
    HomeArm,
    HomeArmResponse,
    Start,
    StartResponse,
    Stop,
    StopResponse,
)
from sensor_msgs.msg import JointState


class TimeoutTransport(xmlrpc.client.Transport):
    def __init__(self, timeout_sec):
        super().__init__()
        self.timeout_sec = timeout_sec

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self.timeout_sec
        return connection


class RosbridgePublicGateway:
    def __init__(self):
        self.lock = threading.RLock()
        self.command_frame = str(rospy.get_param("~command_frame", "base_link")).strip() or "base_link"
        self.legacy_gripper_velocity_scale = float(rospy.get_param("~legacy_gripper_velocity_scale", 5000.0))
        self.feedback_rate_hz = float(rospy.get_param("~public_feedback_rate_hz", 10.0))
        self.private_command_host = str(rospy.get_param("~private_command_host", "127.0.0.1")).strip() or "127.0.0.1"
        self.private_command_port = int(rospy.get_param("~private_command_port", 45931))
        self.private_rpc_host = str(rospy.get_param("~private_rpc_host", "127.0.0.1")).strip() or "127.0.0.1"
        self.private_rpc_port = int(rospy.get_param("~private_rpc_port", 45932))
        self.private_rpc_timeout_sec = float(rospy.get_param("~private_rpc_timeout_sec", 2.0))
        self.private_rpc_uri = "http://{}:{}/".format(self.private_rpc_host, self.private_rpc_port)

        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_target = (self.private_command_host, self.private_command_port)

        self.arms = {
            "right": self.make_arm_state("right"),
            "left": self.make_arm_state("left"),
        }

        self.feedback_timer = rospy.Timer(rospy.Duration(1.0 / self.feedback_rate_hz), self.feedback_timer_cb)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo(
            "ROS bridge public gateway ready | public Kinova surface on this master | private gateway=%s udp=%s:%d",
            self.private_rpc_uri,
            self.private_command_host,
            self.private_command_port,
        )

    @staticmethod
    def normalize_arm_name(raw_name):
        arm_name = str(raw_name).strip().lower()
        if arm_name in ("left", "left_arm"):
            return "left"
        if arm_name in ("right", "right_arm"):
            return "right"
        return ""

    def call_private(self, method_name, *args):
        transport = TimeoutTransport(self.private_rpc_timeout_sec)
        proxy = xmlrpc.client.ServerProxy(
            self.private_rpc_uri, allow_none=False, transport=transport
        )
        try:
            method = getattr(proxy, method_name)
            return method(*args)
        except Exception as exc:
            log_fn = rospy.logwarn_throttle
            if method_name == "get_feedback_snapshot":
                log_fn = rospy.logdebug_throttle
            log_fn(2.0, "Private rosbridge gateway call %s failed: %s", method_name, exc)
            return {"ok": False, "message": str(exc)}

    def make_arm_state(self, arm_name):
        public_driver_namespace = str(
            rospy.get_param(
                "~{}/public_driver_namespace".format(arm_name),
                "/{0}_arm/{0}_arm_driver".format(arm_name),
            )
        ).rstrip("/")

        state = {
            "arm_name": arm_name,
            "public_driver_namespace": public_driver_namespace,
            "joint_state_pub": rospy.Publisher(public_driver_namespace + "/out/joint_state", JointState, queue_size=10),
            "joint_angles_pub": rospy.Publisher(
                public_driver_namespace + "/out/joint_angles", JointAngles, queue_size=10
            ),
            "finger_position_pub": rospy.Publisher(
                public_driver_namespace + "/out/finger_position", FingerPosition, queue_size=10
            ),
            "tool_pose_pub": rospy.Publisher(public_driver_namespace + "/out/tool_pose", PoseStamped, queue_size=10),
        }

        rospy.Subscriber(
            public_driver_namespace + "/in/cartesian_velocity",
            PoseVelocity,
            lambda msg, arm=arm_name: self.pose_velocity_cb(arm, msg),
            queue_size=40,
        )
        rospy.Subscriber(
            public_driver_namespace + "/in/cartesian_velocity_with_fingers",
            PoseVelocityWithFingers,
            lambda msg, arm=arm_name: self.pose_velocity_with_fingers_cb(arm, msg),
            queue_size=40,
        )
        rospy.Subscriber(
            public_driver_namespace + "/in/cartesian_velocity_with_finger_velocity",
            PoseVelocityWithFingerVelocity,
            lambda msg, arm=arm_name: self.pose_velocity_with_finger_velocity_cb(arm, msg),
            queue_size=40,
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
        rospy.Service(
            public_driver_namespace + "/in/home_arm",
            HomeArm,
            lambda _req, arm=arm_name: self.home_service_cb(arm),
        )
        rospy.Service(
            public_driver_namespace + "/in/clear_trajectories",
            ClearTrajectories,
            lambda _req, arm=arm_name: self.clear_service_cb(arm),
        )

        state["fingers_action_server"] = actionlib.SimpleActionServer(
            public_driver_namespace + "/fingers_action/finger_positions",
            SetFingersPositionAction,
            execute_cb=lambda goal, arm=arm_name: self.execute_fingers_goal(arm, goal),
            auto_start=False,
        )
        state["joint_angles_action_server"] = actionlib.SimpleActionServer(
            public_driver_namespace + "/joints_action/joint_angles",
            ArmJointAnglesAction,
            execute_cb=lambda goal, arm=arm_name: self.execute_joint_angles_goal(arm, goal),
            auto_start=False,
        )
        state["tool_pose_action_server"] = actionlib.SimpleActionServer(
            public_driver_namespace + "/pose_action/tool_pose",
            ArmPoseAction,
            execute_cb=lambda goal, arm=arm_name: self.execute_pose_goal(arm, goal),
            auto_start=False,
        )
        state["fingers_action_server"].start()
        state["joint_angles_action_server"].start()
        state["tool_pose_action_server"].start()
        return state

    def send_private_command(self, command):
        try:
            payload = json.dumps(command, separators=(",", ":")).encode("utf-8")
            self.command_socket.sendto(payload, self.command_target)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to forward public rosbridge command: %s", exc)

    def pose_velocity_cb(self, arm_name, msg):
        self.send_private_command(
            {
                "kind": "cartesian_velocity",
                "arm": arm_name,
                "linear": [msg.twist_linear_x, msg.twist_linear_y, msg.twist_linear_z],
                "angular": [msg.twist_angular_x, msg.twist_angular_y, msg.twist_angular_z],
            }
        )

    def pose_velocity_with_fingers_cb(self, arm_name, msg):
        self.pose_velocity_cb(arm_name, msg)
        rospy.logwarn_throttle(
            5.0,
            "%s PoseVelocityWithFingers routes arm motion through Servo, but fingers_closure_percentage is not mapped here. Use the bridged SetFingersPosition action for gripper positioning.",
            arm_name,
        )

    def pose_velocity_with_finger_velocity_cb(self, arm_name, msg):
        normalized = (float(msg.finger1) + float(msg.finger2) + float(msg.finger3)) / 3.0
        if self.legacy_gripper_velocity_scale > 0.0:
            normalized /= self.legacy_gripper_velocity_scale
        normalized = max(min(normalized, 1.0), -1.0)
        self.send_private_command(
            {
                "kind": "cartesian_velocity",
                "arm": arm_name,
                "linear": [msg.twist_linear_x, msg.twist_linear_y, msg.twist_linear_z],
                "angular": [msg.twist_angular_x, msg.twist_angular_y, msg.twist_angular_z],
                "gripper_velocity": normalized,
            }
        )

    def start_service_cb(self, arm_name):
        result = self.call_private("start_arm", arm_name)
        return StartResponse(start_result=str(result.get("message", "")))

    def stop_service_cb(self, arm_name):
        result = self.call_private("stop_arm", arm_name)
        return StopResponse(stop_result=str(result.get("message", "")))

    def home_service_cb(self, arm_name):
        result = self.call_private("home_arm", arm_name)
        return HomeArmResponse(homearm_result=str(result.get("message", "")))

    def clear_service_cb(self, arm_name):
        result = self.call_private("clear_trajectories", arm_name)
        return ClearTrajectoriesResponse(result=str(result.get("message", "")))

    @staticmethod
    def build_joint_angles_result(angles_dict):
        result = ArmJointAnglesResult()
        for index in range(7):
            setattr(result.angles, "joint{}".format(index + 1), float(angles_dict.get("joint{}".format(index + 1), 0.0)))
        return result

    @staticmethod
    def build_fingers_result(fingers_dict):
        result = SetFingersPositionResult()
        result.fingers.finger1 = float(fingers_dict.get("finger1", 0.0))
        result.fingers.finger2 = float(fingers_dict.get("finger2", 0.0))
        result.fingers.finger3 = float(fingers_dict.get("finger3", 0.0))
        return result

    @staticmethod
    def build_pose_result(pose_dict):
        result = ArmPoseResult()
        result.pose.header.stamp = rospy.Time.from_sec(float(pose_dict.get("stamp", 0.0)))
        result.pose.header.frame_id = str(pose_dict.get("frame_id", ""))
        position = pose_dict.get("position", {})
        orientation = pose_dict.get("orientation", {})
        result.pose.pose.position.x = float(position.get("x", 0.0))
        result.pose.pose.position.y = float(position.get("y", 0.0))
        result.pose.pose.position.z = float(position.get("z", 0.0))
        result.pose.pose.orientation.x = float(orientation.get("x", 0.0))
        result.pose.pose.orientation.y = float(orientation.get("y", 0.0))
        result.pose.pose.orientation.z = float(orientation.get("z", 0.0))
        result.pose.pose.orientation.w = float(orientation.get("w", 1.0))
        return result

    def execute_fingers_goal(self, arm_name, goal):
        result = self.call_private(
            "send_fingers_goal",
            arm_name,
            [goal.fingers.finger1, goal.fingers.finger2, goal.fingers.finger3],
            30.0,
        )
        action_server = self.arms[arm_name]["fingers_action_server"]
        fingers_result = self.build_fingers_result(result.get("fingers", {}))
        if result.get("ok", False):
            action_server.set_succeeded(fingers_result, str(result.get("message", "")))
        else:
            action_server.set_aborted(fingers_result, str(result.get("message", "")))

    def execute_joint_angles_goal(self, arm_name, goal):
        result = self.call_private(
            "send_joint_angles_goal",
            arm_name,
            [
                goal.angles.joint1,
                goal.angles.joint2,
                goal.angles.joint3,
                goal.angles.joint4,
                goal.angles.joint5,
                goal.angles.joint6,
                goal.angles.joint7,
            ],
            45.0,
        )
        action_server = self.arms[arm_name]["joint_angles_action_server"]
        joint_result = self.build_joint_angles_result(result.get("angles", {}))
        if result.get("ok", False):
            action_server.set_succeeded(joint_result, str(result.get("message", "")))
        else:
            action_server.set_aborted(joint_result, str(result.get("message", "")))

    def execute_pose_goal(self, arm_name, goal):
        result = self.call_private(
            "send_pose_goal",
            arm_name,
            {
                "frame_id": goal.pose.header.frame_id,
                "position": {
                    "x": goal.pose.pose.position.x,
                    "y": goal.pose.pose.position.y,
                    "z": goal.pose.pose.position.z,
                },
                "orientation": {
                    "x": goal.pose.pose.orientation.x,
                    "y": goal.pose.pose.orientation.y,
                    "z": goal.pose.pose.orientation.z,
                    "w": goal.pose.pose.orientation.w,
                },
            },
            45.0,
        )
        action_server = self.arms[arm_name]["tool_pose_action_server"]
        pose_result = self.build_pose_result(result.get("pose", {}))
        if result.get("ok", False):
            action_server.set_succeeded(pose_result, str(result.get("message", "")))
        else:
            action_server.set_aborted(pose_result, str(result.get("message", "")))

    def publish_joint_state(self, publisher, data):
        if not data:
            return
        msg = JointState()
        msg.header.stamp = rospy.Time.from_sec(float(data.get("stamp", 0.0)))
        msg.header.frame_id = str(data.get("frame_id", ""))
        msg.name = list(data.get("name", []))
        msg.position = [float(value) for value in data.get("position", [])]
        msg.velocity = [float(value) for value in data.get("velocity", [])]
        msg.effort = [float(value) for value in data.get("effort", [])]
        publisher.publish(msg)

    def publish_joint_angles(self, publisher, data):
        if not data:
            return
        msg = JointAngles()
        for index in range(7):
            setattr(msg, "joint{}".format(index + 1), float(data.get("joint{}".format(index + 1), 0.0)))
        publisher.publish(msg)

    def publish_finger_position(self, publisher, data):
        if not data:
            return
        msg = FingerPosition()
        msg.finger1 = float(data.get("finger1", 0.0))
        msg.finger2 = float(data.get("finger2", 0.0))
        msg.finger3 = float(data.get("finger3", 0.0))
        publisher.publish(msg)

    def publish_tool_pose(self, publisher, data):
        if not data:
            return
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.from_sec(float(data.get("stamp", 0.0)))
        msg.header.frame_id = str(data.get("frame_id", self.command_frame))
        position = data.get("position", {})
        orientation = data.get("orientation", {})
        msg.pose.position.x = float(position.get("x", 0.0))
        msg.pose.position.y = float(position.get("y", 0.0))
        msg.pose.position.z = float(position.get("z", 0.0))
        msg.pose.orientation.x = float(orientation.get("x", 0.0))
        msg.pose.orientation.y = float(orientation.get("y", 0.0))
        msg.pose.orientation.z = float(orientation.get("z", 0.0))
        msg.pose.orientation.w = float(orientation.get("w", 1.0))
        publisher.publish(msg)

    def feedback_timer_cb(self, _event):
        snapshot = self.call_private("get_feedback_snapshot")
        arms = snapshot.get("arms", {})
        for arm_name in ("right", "left"):
            if arm_name not in arms:
                continue
            arm_data = arms.get(arm_name, {})
            arm_state = self.arms[arm_name]
            self.publish_joint_state(arm_state["joint_state_pub"], arm_data.get("joint_state", {}))
            self.publish_joint_angles(arm_state["joint_angles_pub"], arm_data.get("joint_angles", {}))
            self.publish_finger_position(arm_state["finger_position_pub"], arm_data.get("finger_position", {}))
            self.publish_tool_pose(arm_state["tool_pose_pub"], arm_data.get("tool_pose", {}))

    def on_shutdown(self):
        try:
            self.command_socket.close()
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("rosbridge_public_gateway")
    RosbridgePublicGateway()
    rospy.spin()
