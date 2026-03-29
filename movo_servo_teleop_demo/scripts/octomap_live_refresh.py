#!/usr/bin/env python3

import rospy
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene


class OctomapLiveRefresh:
    def __init__(self):
        self.enabled = rospy.get_param("~enabled", True)
        self.apply_service_name = rospy.get_param("~apply_planning_scene_service", "/apply_planning_scene")
        self.octomap_frame = rospy.get_param("~octomap_frame", "base_link")
        self.startup_delay_sec = float(rospy.get_param("~startup_delay_sec", 8.0))
        self.clear_period_sec = float(rospy.get_param("~clear_period_sec", 0.75))
        self._apply_proxy = None
        self._clear_count = 0

        if not self.enabled:
            rospy.loginfo("Octomap live refresh disabled")
            return

        rospy.loginfo(
            "Octomap live refresh enabled: service=%s frame=%s startup_delay=%.2fs period=%.2fs",
            self.apply_service_name,
            self.octomap_frame,
            self.startup_delay_sec,
            self.clear_period_sec,
        )

        if self.startup_delay_sec > 0.0:
            rospy.Timer(rospy.Duration(self.startup_delay_sec), self._start_timer, oneshot=True)
        else:
            self._start_timer(None)

    def _start_timer(self, _event):
        if rospy.is_shutdown() or not self.enabled:
            return
        self._timer = rospy.Timer(rospy.Duration(self.clear_period_sec), self._clear_octomap)

    def _get_proxy(self):
        if self._apply_proxy is None:
            rospy.wait_for_service(self.apply_service_name, timeout=10.0)
            self._apply_proxy = rospy.ServiceProxy(self.apply_service_name, ApplyPlanningScene)
        return self._apply_proxy

    def _build_empty_octomap_scene(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.octomap.header.frame_id = self.octomap_frame
        scene.world.octomap.octomap.header.frame_id = self.octomap_frame
        scene.world.octomap.origin.orientation.w = 1.0
        return scene

    def _clear_octomap(self, _event):
        try:
            response = self._get_proxy()(self._build_empty_octomap_scene())
            self._clear_count += 1
            if not response.success:
                rospy.logwarn_throttle(5.0, "Octomap live refresh request was rejected")
                return
            if self._clear_count == 1 or self._clear_count % 20 == 0:
                rospy.loginfo("Octomap live refresh cleared the planning-scene octomap")
        except Exception as exc:
            self._apply_proxy = None
            if rospy.is_shutdown():
                return
            rospy.logwarn_throttle(5.0, "Octomap live refresh failed: %s", exc)


def main():
    rospy.init_node("octomap_live_refresh")
    OctomapLiveRefresh()
    rospy.spin()


if __name__ == "__main__":
    main()
