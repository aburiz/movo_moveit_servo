#!/usr/bin/env python3

import os
import signal
import subprocess
import time
import xmlrpc.client
from urllib.parse import urlparse

import rospy


class PrivateComputeStack:
    def __init__(self):
        self.private_master_uri = str(rospy.get_param("~private_master_uri", "http://127.0.0.1:11312")).strip()
        self.private_ros_ip = str(rospy.get_param("~private_ros_ip", "127.0.0.1")).strip() or "127.0.0.1"
        self.start_private_roscore = bool(rospy.get_param("~start_private_roscore", True))
        self.fail_if_private_master_running = bool(rospy.get_param("~fail_if_private_master_running", True))
        self.private_master_wait_timeout_sec = float(rospy.get_param("~private_master_wait_timeout_sec", 15.0))
        self.private_launch_file = str(rospy.get_param("~private_launch_file", "")).strip()
        self.started_roscore = False
        self.roscore_process = None
        self.launch_process = None

        parsed = urlparse(self.private_master_uri)
        self.private_master_host = parsed.hostname or "127.0.0.1"
        self.private_master_port = parsed.port or 11312

        self.launch_arg_names = [
            "rviz",
            "launch_joy",
            "joy_dev",
            "launch_realsense_d455",
            "realsense_namespace",
            "realsense_clip_distance",
            "octomap_max_range",
            "live_octomap_refresh",
            "live_octomap_refresh_period_sec",
            "live_octomap_refresh_startup_delay_sec",
            "default_active_arm",
            "enable_left_servo",
            "servo_start_delay",
            "load_test_obstacle",
            "local_machine_ip",
            "real_arms_use_urdf",
            "real_arms_feedback_publish_rate",
            "respawn_real_arm_driver",
            "left_arm_ip",
            "right_arm_ip",
            "left_robot_port",
            "right_robot_port",
            "left_local_cmd_port",
            "left_local_broadcast_port",
            "right_local_cmd_port",
            "right_local_broadcast_port",
            "left_arm_serial",
            "right_arm_serial",
        ]

        rospy.on_shutdown(self.on_shutdown)

    def master_available(self):
        try:
            proxy = xmlrpc.client.ServerProxy(self.private_master_uri)
            code, _message, _value = proxy.getPid("/private_compute_stack")
            return int(code) == 1
        except Exception:
            return False

    def wait_for_master(self):
        deadline = time.time() + self.private_master_wait_timeout_sec
        while (not rospy.is_shutdown()) and time.time() < deadline:
            if self.master_available():
                return True
            time.sleep(0.2)
        return self.master_available()

    def private_env(self):
        env = dict(os.environ)
        env["ROS_MASTER_URI"] = self.private_master_uri
        env["ROS_IP"] = self.private_ros_ip
        env["ROS_HOSTNAME"] = self.private_ros_ip
        return env

    def format_roslaunch_arg(self, name, value):
        if isinstance(value, bool):
            rendered = "true" if value else "false"
        else:
            rendered = str(value)
        return "{}:={}".format(name, rendered)

    def build_roslaunch_args(self):
        args = []
        for name in self.launch_arg_names:
            param_name = "~" + name
            if rospy.has_param(param_name):
                args.append(self.format_roslaunch_arg(name, rospy.get_param(param_name)))
        return args

    def start(self):
        if not self.private_launch_file:
            rospy.logfatal("private_compute_stack requires ~private_launch_file")
            return False

        if self.master_available():
            if self.fail_if_private_master_running:
                rospy.logfatal(
                    "Private compute master %s is already running. Stop the existing split-master stack before starting another.",
                    self.private_master_uri,
                )
                return False
        elif self.start_private_roscore:
            self.roscore_process = subprocess.Popen(
                ["roscore", "-p", str(self.private_master_port)],
                env=self.private_env(),
                preexec_fn=os.setsid,
            )
            self.started_roscore = True
            if not self.wait_for_master():
                rospy.logfatal("Timed out waiting for private roscore at %s", self.private_master_uri)
                return False
        else:
            rospy.logfatal("Private master %s is not running and start_private_roscore:=false", self.private_master_uri)
            return False

        command = ["roslaunch", "--wait", self.private_launch_file] + self.build_roslaunch_args()
        rospy.loginfo("Starting private compute stack on %s using %s", self.private_master_uri, self.private_launch_file)
        self.launch_process = subprocess.Popen(
            command,
            env=self.private_env(),
            preexec_fn=os.setsid,
        )
        return True

    def terminate_process(self, process, label, timeout_sec=5.0):
        if process is None:
            return
        if process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            deadline = time.time() + timeout_sec
            while time.time() < deadline:
                if process.poll() is not None:
                    return
                time.sleep(0.1)
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except Exception as exc:
            rospy.logwarn("Failed to stop %s cleanly: %s", label, exc)

    def spin(self):
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            if self.launch_process is not None:
                return_code = self.launch_process.poll()
                if return_code is not None:
                    rospy.logerr("Private compute roslaunch exited with code %s", return_code)
                    rospy.signal_shutdown("private compute stack exited")
                    break
            if self.started_roscore and self.roscore_process is not None:
                return_code = self.roscore_process.poll()
                if return_code is not None:
                    rospy.logerr("Private roscore exited with code %s", return_code)
                    rospy.signal_shutdown("private roscore exited")
                    break
            rate.sleep()

    def on_shutdown(self):
        self.terminate_process(self.launch_process, "private roslaunch")
        if self.started_roscore:
            self.terminate_process(self.roscore_process, "private roscore")


if __name__ == "__main__":
    rospy.init_node("private_compute_stack")
    node = PrivateComputeStack()
    if node.start():
        node.spin()
