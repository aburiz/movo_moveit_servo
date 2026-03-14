#!/usr/bin/env python3

import sys

import rospy


def main():
    rospy.init_node("real_arm_launch_error")
    message = rospy.get_param(
        "~message",
        "Invalid real-arm launch configuration. Set local_machine_ip:=<CURRENT_PC_IP> when use_real_arms:=true.",
    )
    rospy.logfatal(message)
    print(message, file=sys.stderr)
    raise SystemExit(1)


if __name__ == "__main__":
    main()
