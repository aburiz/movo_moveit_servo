#!/usr/bin/env python3

import sys

import rospy
import rosgraph
import tf
from sensor_msgs.msg import PointCloud2


class D455MoveItPerceptionCheck:
    def __init__(self):
        self.timeout_sec = float(rospy.get_param("~timeout_sec", 12.0))
        self.camera_namespace = rospy.get_param("~camera_namespace", "d455").strip("/")
        self.mount_parent_frame = rospy.get_param("~mount_parent_frame", "kinect2_link")
        self.camera_base_frame = rospy.get_param("~camera_base_frame", "d455_link")
        self.move_group_namespace = rospy.get_param("~move_group_namespace", "/move_group").rstrip("/")

        self.direct_points_topic = "/" + self.camera_namespace + "/depth/color/points"
        self.legacy_points_topics = ["/kinect/sd/points", "/kinect2/sd/points"]

        self.seen_topics = {}
        self.subscribers = []
        for topic_name in [self.direct_points_topic] + self.legacy_points_topics:
            self.seen_topics[topic_name] = False
            self.subscribers.append(
                rospy.Subscriber(topic_name, PointCloud2, self.pointcloud_cb, callback_args=topic_name, queue_size=1)
            )

        self.tf_listener = tf.TransformListener()
        self.master = rosgraph.Master(rospy.get_name())

    def pointcloud_cb(self, _msg, topic_name):
        self.seen_topics[topic_name] = True

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

    def wait_for_messages(self):
        deadline = rospy.Time.now() + rospy.Duration(self.timeout_sec)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if all(self.seen_topics.values()):
                return True
            rate.sleep()
        return False

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
        point_cloud_topic = first.get("point_cloud_topic", "<missing>")
        return True, f"{sensors_param}[0].point_cloud_topic = {point_cloud_topic}"

    def run(self):
        print(f"Camera namespace: /{self.camera_namespace}")
        print(f"Expected direct point cloud: {self.direct_points_topic}")
        print(f"Expected legacy relay topics: {', '.join(self.legacy_points_topics)}")
        print(f"Expected mount TF: {self.mount_parent_frame} -> {self.camera_base_frame}")

        all_ok = True

        ok, detail = self.check_topic_exists(self.direct_points_topic)
        self.print_result(f"Direct D455 point cloud advertised [{self.direct_points_topic}]", ok, detail)
        all_ok = all_ok and ok

        for topic_name in self.legacy_points_topics:
            ok, detail = self.check_topic_exists(topic_name)
            self.print_result(f"Legacy relay topic advertised [{topic_name}]", ok, detail)

        direct_message_deadline = rospy.Time.now() + rospy.Duration(self.timeout_sec)
        rate = rospy.Rate(20)
        got_direct_messages = False
        while not rospy.is_shutdown() and rospy.Time.now() < direct_message_deadline:
            if self.seen_topics[self.direct_points_topic]:
                got_direct_messages = True
                break
            rate.sleep()

        self.print_result(
            "Direct D455 point cloud messages arriving",
            got_direct_messages,
            "Received PointCloud2 data on the direct D455 topic"
            if got_direct_messages
            else "Timed out waiting for PointCloud2 data on the direct D455 topic",
        )
        all_ok = all_ok and got_direct_messages

        tf_ok, tf_detail = self.check_tf()
        self.print_result("Camera TF attached", tf_ok, tf_detail)
        all_ok = all_ok and tf_ok

        moveit_ok, moveit_detail = self.check_moveit_sensor_params()
        self.print_result("MoveIt sensor config loaded", moveit_ok, moveit_detail)
        all_ok = all_ok and moveit_ok and (self.direct_points_topic in moveit_detail)

        print("Overall: {}".format("PASS" if all_ok else "FAIL"))
        return 0 if all_ok else 1


def main():
    rospy.init_node("check_d455_moveit_perception")
    checker = D455MoveItPerceptionCheck()
    sys.exit(checker.run())


if __name__ == "__main__":
    main()
