#!/usr/bin/env python3

import argparse
import subprocess

import rospy
import rosservice
from kinova_msgs.msg import JointVelocity
from kinova_msgs.srv import HomeArm, Start, Stop
from sensor_msgs.msg import JointState


def cfg(config_namespace, key, default=None):
    return rospy.get_param("{0}/{1}".format(config_namespace.rstrip("/"), key), default)


def normalize_arm_name(raw_name):
    name = str(raw_name).strip().lower()
    if name in ("left", "left_arm"):
        return "left"
    if name in ("right", "right_arm"):
        return "right"
    raise ValueError("Unsupported arm '{0}'".format(raw_name))


def ping_ip(ip_address):
    try:
        completed = subprocess.run(
            ["ping", "-c", "1", "-W", "1", ip_address],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            text=True,
        )
    except OSError as exc:
        return False, "ping command failed: {0}".format(exc)

    return completed.returncode == 0, completed.stdout.strip() or completed.stderr.strip()


def make_joint_velocity(joint_index, velocity_deg_s):
    msg = JointVelocity()
    values = [0.0] * 7
    values[joint_index] = float(velocity_deg_s)
    for idx, value in enumerate(values):
        setattr(msg, "joint{0}".format(idx + 1), value)
    return msg


def main():
    parser = argparse.ArgumentParser(description="Ping and optionally identify a real MOVO Kinova arm.")
    parser.add_argument("--arm", required=True, choices=["left", "right"], help="Arm to inspect.")
    parser.add_argument("--motion", action="store_true", help="Send a very small identification pulse.")
    parser.add_argument("--timeout", type=float, default=1.0, help="Seconds to wait for feedback/services.")
    parser.add_argument(
        "--motion-joint",
        type=int,
        default=1,
        choices=[1, 2, 3, 4, 5, 6, 7],
        help="1-based joint index for the identification pulse.",
    )
    parser.add_argument("--motion-velocity-deg-s", type=float, default=1.0, help="Identification pulse speed.")
    parser.add_argument("--motion-duration", type=float, default=0.15, help="Identification pulse duration.")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("ping_real_arm", anonymous=True)

    config_namespace = rospy.get_param("~config_namespace", "/movo_servo_teleop_demo/real_arms")
    arm_name = normalize_arm_name(args.arm)
    other_arm = "left" if arm_name == "right" else "right"

    driver_namespace = str(cfg(config_namespace, "{0}_arm/driver_namespace".format(arm_name), "")).rstrip("/")
    other_driver_namespace = str(cfg(config_namespace, "{0}_arm/driver_namespace".format(other_arm), "")).rstrip("/")
    target_ip = str(cfg(config_namespace, "{0}_arm/robot_ip".format(arm_name), "")).strip()
    other_target_ip = str(cfg(config_namespace, "{0}_arm/robot_ip".format(other_arm), "")).strip()
    local_machine_ip = str(cfg(config_namespace, "local_machine_ip", "")).strip()

    start_service_name = driver_namespace + "/in/start"
    stop_service_name = driver_namespace + "/in/stop"
    home_service_name = driver_namespace + "/in/home_arm"
    feedback_topic = driver_namespace + "/out/joint_state"
    joint_velocity_topic = driver_namespace + "/in/joint_velocity"

    print("Requested arm: {0}".format(arm_name))
    print("Target namespace: /{0}_arm".format(arm_name))
    print("Target driver namespace: {0}".format(driver_namespace))
    print("Target IP: {0}".format(target_ip))
    print("Configured local machine IP: {0}".format(local_machine_ip or "<unset>"))
    print(
        "Expected opposite-arm mapping: {0} -> {1} -> {2}".format(
            other_arm, other_driver_namespace or "<unset>", other_target_ip or "<unset>"
        )
    )

    ip_reachable, ping_detail = ping_ip(target_ip) if target_ip else (False, "target_ip not configured")
    print("Robot IP reachable: {0}".format("yes" if ip_reachable else "no"))
    if ping_detail:
        print("Ping detail: {0}".format(ping_detail.splitlines()[-1]))

    try:
        service_list = set(rosservice.get_service_list())
    except Exception as exc:
        service_list = set()
        print("Service query failed: {0}".format(exc))

    service_present = {
        "start": start_service_name in service_list,
        "stop": stop_service_name in service_list,
        "home": home_service_name in service_list,
    }
    print("Start service present: {0}".format("yes" if service_present["start"] else "no"))
    print("Stop service present: {0}".format("yes" if service_present["stop"] else "no"))
    print("Home service present: {0}".format("yes" if service_present["home"] else "no"))
    print("Driver reachable: {0}".format("yes" if any(service_present.values()) else "no"))

    feedback_received = False
    try:
        rospy.wait_for_message(feedback_topic, JointState, timeout=args.timeout)
        feedback_received = True
    except rospy.ROSException:
        feedback_received = False
    print("Feedback received on {0}: {1}".format(feedback_topic, "yes" if feedback_received else "no"))

    if not args.motion:
        print(
            "Non-motion diagnostics complete. If left/right are swapped, compare the requested arm/IP above with the physical arm."
        )
        return

    print("Motion mode requested. This sends a small pulse to {0} joint{1}.".format(arm_name, args.motion_joint))
    print("If the opposite physical arm moves, the namespace/IP/serial mapping is reversed.")

    if not all(service_present.values()):
        raise SystemExit("Motion pulse aborted because start/stop/home services are not all available.")

    start_proxy = rospy.ServiceProxy(start_service_name, Start)
    stop_proxy = rospy.ServiceProxy(stop_service_name, Stop)
    home_proxy = rospy.ServiceProxy(home_service_name, HomeArm)
    _ = home_proxy  # Keep a proxy ready so users can see the service was resolved for this arm.
    publisher = rospy.Publisher(joint_velocity_topic, JointVelocity, queue_size=1)
    rospy.sleep(0.25)

    try:
        start_proxy()
    except rospy.ServiceException as exc:
        raise SystemExit("Failed to call start on {0}: {1}".format(start_service_name, exc))

    pulse_msg = make_joint_velocity(args.motion_joint - 1, args.motion_velocity_deg_s)
    zero_msg = make_joint_velocity(args.motion_joint - 1, 0.0)
    rate = rospy.Rate(100)
    iterations = max(1, int(round(args.motion_duration * 100.0)))

    for _ in range(iterations):
        publisher.publish(pulse_msg)
        rate.sleep()

    for _ in range(5):
        publisher.publish(zero_msg)
        rate.sleep()

    try:
        stop_proxy()
    except rospy.ServiceException as exc:
        raise SystemExit("Identification pulse sent, but stop failed on {0}: {1}".format(stop_service_name, exc))

    print(
        "Identification pulse complete. If the wrong arm moved, swap the left/right namespace, IP, or serial mapping before teleop."
    )


if __name__ == "__main__":
    main()
