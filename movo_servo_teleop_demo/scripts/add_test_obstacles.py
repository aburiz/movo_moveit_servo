#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped


def wait_for_scene_update(scene, box_name, timeout=5.0):
    start = rospy.get_time()
    while (rospy.get_time() - start) < timeout and not rospy.is_shutdown():
        known = scene.get_known_object_names()
        if box_name in known:
            return True
        rospy.sleep(0.1)
    return False


def main():
    rospy.init_node("add_test_obstacles")
    moveit_commander.roscpp_initialize([])

    scene = moveit_commander.PlanningSceneInterface(synchronous=True)
    rospy.sleep(1.0)

    planning_frame = rospy.get_param("~planning_frame", "odom")
    box_name = rospy.get_param("~box_name", "teleop_demo_box")
    pose_cfg = rospy.get_param("~box_pose", {})
    size_cfg = rospy.get_param("~box_size", {})

    pose = PoseStamped()
    pose.header.frame_id = planning_frame
    pose.pose.position.x = float(pose_cfg.get("x", 0.65))
    pose.pose.position.y = float(pose_cfg.get("y", -0.25))
    pose.pose.position.z = float(pose_cfg.get("z", 0.95))
    pose.pose.orientation.x = float(pose_cfg.get("qx", 0.0))
    pose.pose.orientation.y = float(pose_cfg.get("qy", 0.0))
    pose.pose.orientation.z = float(pose_cfg.get("qz", 0.0))
    pose.pose.orientation.w = float(pose_cfg.get("qw", 1.0))

    size = (
        float(size_cfg.get("x", 0.25)),
        float(size_cfg.get("y", 0.25)),
        float(size_cfg.get("z", 0.35)),
    )

    scene.remove_world_object(box_name)
    rospy.sleep(0.25)
    scene.add_box(box_name, pose, size=size)

    if wait_for_scene_update(scene, box_name, timeout=5.0):
        rospy.loginfo("Added obstacle '%s' in frame '%s'", box_name, planning_frame)
    else:
        rospy.logwarn("Could not confirm obstacle '%s' in planning scene", box_name)

    rospy.spin()


if __name__ == "__main__":
    main()
