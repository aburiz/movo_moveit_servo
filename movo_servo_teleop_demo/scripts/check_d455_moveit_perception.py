#!/usr/bin/env python3

import sys

import rospy
import rosgraph
import tf
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2


class D455MoveItPerceptionCheck:
    def __init__(self):
        self.timeout_sec = float(rospy.get_param("~timeout_sec", 12.0))
        self.camera_namespace = rospy.get_param("~camera_namespace", "d455").strip("/")
        self.mount_parent_frame = rospy.get_param("~mount_parent_frame", "kinect2_link")
        self.camera_base_frame = rospy.get_param("~camera_base_frame", "d455_link")
        self.move_group_namespace = rospy.get_param("~move_group_namespace", "/move_group").rstrip("/")

        self.depth_image_topic = "/" + self.camera_namespace + "/aligned_depth_to_color/image_raw"
        self.depth_camera_info_topic = "/" + self.camera_namespace + "/aligned_depth_to_color/camera_info"
        self.direct_points_topic = "/" + self.camera_namespace + "/depth/color/points"
        self.legacy_points_topics = ["/kinect/sd/points", "/kinect2/sd/points"]

        self.seen_pointcloud_topics = {}
        self.subscribers = []
        for topic_name in [self.direct_points_topic] + self.legacy_points_topics:
            self.seen_pointcloud_topics[topic_name] = False
            self.subscribers.append(
                rospy.Subscriber(topic_name, PointCloud2, self.pointcloud_cb, callback_args=topic_name, queue_size=1)
            )
        self.seen_depth_image = False
        self.seen_depth_camera_info = False
        self.subscribers.append(rospy.Subscriber(self.depth_image_topic, Image, self.depth_image_cb, queue_size=1))
        self.subscribers.append(
            rospy.Subscriber(self.depth_camera_info_topic, CameraInfo, self.depth_camera_info_cb, queue_size=1)
        )

        self.tf_listener = tf.TransformListener()
        self.master = rosgraph.Master(rospy.get_name())

    def pointcloud_cb(self, _msg, topic_name):
        self.seen_pointcloud_topics[topic_name] = True

    def depth_image_cb(self, _msg):
        self.seen_depth_image = True

    def depth_camera_info_cb(self, _msg):
        self.seen_depth_camera_info = True

    def print_result(self, label, ok, detail):
        state = "yes" if ok else "no"
        print(f"{label}: {state}")
        print(f"  {detail}")

    def check_topic_exists(self, topic_name):
        try:
            published_topics = dict(self.master.getPublishedTopics(""))
        except Exception as exc:
            return False, f"Could not query ROS master topics: {exc}"
        if topic_name not in published_topics:
            return False, f"{topic_name} is not currently published"
        return True, f"{topic_name} type = {published_topics[topic_name]}"

    def check_tf(self):
        try:
            self.tf_listener.waitForTransform(
                self.mount_parent_frame, self.camera_base_frame, rospy.Time(0), rospy.Duration(self.timeout_sec)
            )
            return True, f"TF available: {self.mount_parent_frame} -> {self.camera_base_frame}"
        except Exception as exc:
            return False, f"Missing TF {self.mount_parent_frame} -> {self.camera_base_frame}: {exc}"

    def check_moveit_sensor_params(self):
        sensors_param = self.move_group_namespace + "/sensors"
        if not rospy.has_param(sensors_param):
            return False, f"{sensors_param} is not loaded"
        sensors = rospy.get_param(sensors_param, [])
        if not sensors:
            return False, f"{sensors_param} is empty"
        first = sensors[0]
        sensor_plugin = first.get("sensor_plugin", "<missing>")
        point_cloud_topic = first.get("point_cloud_topic")
        image_topic = first.get("image_topic")
        if point_cloud_topic:
            return True, f"{sensors_param}[0] plugin={sensor_plugin} point_cloud_topic = {point_cloud_topic}"
        if image_topic:
            return True, f"{sensors_param}[0] plugin={sensor_plugin} image_topic = {image_topic}"
        return False, f"{sensors_param}[0] does not define point_cloud_topic or image_topic"

    def run(self):
        print(f"Camera namespace: /{self.camera_namespace}")
        print(f"Expected aligned depth image: {self.depth_image_topic}")
        print(f"Expected aligned depth camera info: {self.depth_camera_info_topic}")
        print(f"Expected direct point cloud: {self.direct_points_topic}")
        print(f"Expected legacy relay topics: {', '.join(self.legacy_points_topics)}")
        print(f"Expected mount TF: {self.mount_parent_frame} -> {self.camera_base_frame}")

        all_ok = True

        ok, detail = self.check_topic_exists(self.depth_image_topic)
        self.print_result(f"Aligned depth image advertised [{self.depth_image_topic}]", ok, detail)
        all_ok = all_ok and ok

        ok, detail = self.check_topic_exists(self.depth_camera_info_topic)
        self.print_result(f"Aligned depth camera_info advertised [{self.depth_camera_info_topic}]", ok, detail)
        all_ok = all_ok and ok

        ok, detail = self.check_topic_exists(self.direct_points_topic)
        self.print_result(f"Direct D455 point cloud advertised [{self.direct_points_topic}]", ok, detail)

        for topic_name in self.legacy_points_topics:
            ok, detail = self.check_topic_exists(topic_name)
            self.print_result(f"Legacy relay topic advertised [{topic_name}]", ok, detail)

        direct_message_deadline = rospy.Time.now() + rospy.Duration(self.timeout_sec)
        rate = rospy.Rate(20)
        got_depth_messages = False
        got_camera_info = False
        got_direct_pointcloud_messages = False
        while not rospy.is_shutdown() and rospy.Time.now() < direct_message_deadline:
            got_depth_messages = self.seen_depth_image
            got_camera_info = self.seen_depth_camera_info
            got_direct_pointcloud_messages = self.seen_pointcloud_topics[self.direct_points_topic]
            if got_depth_messages and got_camera_info:
                break
            rate.sleep()

        self.print_result(
            "Aligned depth image messages arriving",
            got_depth_messages,
            "Received Image data on the aligned D455 depth topic"
            if got_depth_messages
            else "Timed out waiting for Image data on the aligned D455 depth topic",
        )
        all_ok = all_ok and got_depth_messages

        self.print_result(
            "Aligned depth camera_info messages arriving",
            got_camera_info,
            "Received CameraInfo data on the aligned D455 depth topic"
            if got_camera_info
            else "Timed out waiting for CameraInfo data on the aligned D455 depth topic",
        )
        all_ok = all_ok and got_camera_info

        self.print_result(
            "Direct D455 point cloud messages arriving",
            got_direct_pointcloud_messages,
            "Received PointCloud2 data on the direct D455 topic"
            if got_direct_pointcloud_messages
            else "No PointCloud2 data seen on the direct D455 topic during the check",
        )

        tf_ok, tf_detail = self.check_tf()
        self.print_result("Camera TF attached", tf_ok, tf_detail)
        all_ok = all_ok and tf_ok

        moveit_ok, moveit_detail = self.check_moveit_sensor_params()
        self.print_result("MoveIt sensor config loaded", moveit_ok, moveit_detail)
        all_ok = all_ok and moveit_ok and (self.depth_image_topic in moveit_detail)

        print("Overall: {}".format("PASS" if all_ok else "FAIL"))
        return 0 if all_ok else 1


def main():
    rospy.init_node("check_d455_moveit_perception")
    checker = D455MoveItPerceptionCheck()
    sys.exit(checker.run())


if __name__ == "__main__":
    main()
