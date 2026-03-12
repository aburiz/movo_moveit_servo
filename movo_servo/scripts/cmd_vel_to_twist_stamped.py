#!/usr/bin/env python3

import copy

import rospy
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelToTwistStamped:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/movo_servo/cmd_vel")
        self.output_topic = rospy.get_param("~output_topic", "/servo_server/delta_twist_cmds")
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.publish_rate = float(rospy.get_param("~publish_rate", 100.0))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.25))

        self.last_twist = Twist()
        self.last_command_time = rospy.Time(0)

        self.pub = rospy.Publisher(self.output_topic, TwistStamped, queue_size=20)
        self.sub = rospy.Subscriber(self.input_topic, Twist, self.twist_cb, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_cb)

        rospy.loginfo("Publishing stamped twists from %s to %s", self.input_topic, self.output_topic)

    def twist_cb(self, msg):
        self.last_twist = copy.deepcopy(msg)
        self.last_command_time = rospy.Time.now()

    def timer_cb(self, _event):
        if self.last_command_time == rospy.Time(0):
            return

        cmd = TwistStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = self.frame_id

        age = (rospy.Time.now() - self.last_command_time).to_sec()
        if age <= self.command_timeout:
            cmd.twist = copy.deepcopy(self.last_twist)
        else:
            cmd.twist = Twist()

        self.pub.publish(cmd)


def main():
    rospy.init_node("cmd_vel_to_twist_stamped")
    CmdVelToTwistStamped()
    rospy.spin()


if __name__ == "__main__":
    main()
