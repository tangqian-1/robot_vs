#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import rospy

from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from std_srvs.srv import Empty


DEFAULT_NAMESPACES = ["robot_red", "robot_red2", "robot_blue", "robot_blue2"]


def _parse_namespaces(value):
    if value is None:
        return list(DEFAULT_NAMESPACES)
    if isinstance(value, list):
        return [str(v).strip() for v in value if str(v).strip()]
    text = str(value).strip()
    if not text:
        return list(DEFAULT_NAMESPACES)
    return [item.strip() for item in text.split(",") if item.strip()]


def _topic_for(ns, suffix):
    if suffix.startswith("/"):
        return suffix
    return "/{}/{}".format(ns, suffix)


def main():
    rospy.init_node("stop_all_robots", anonymous=True)

    namespaces = _parse_namespaces(rospy.get_param("~namespaces", None))
    cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "cmd_vel")
    stop_duration = float(rospy.get_param("~stop_duration", 1.0))
    rate_hz = float(rospy.get_param("~rate_hz", 10.0))
    cancel_nav = bool(rospy.get_param("~cancel_nav", True))
    clear_costmaps = bool(rospy.get_param("~clear_costmaps", True))

    if not namespaces:
        rospy.logwarn("No namespaces provided; nothing to stop")
        return

    publishers = {}
    cancel_publishers = {}
    clear_services = {}

    for ns in namespaces:
        cmd_topic = _topic_for(ns, cmd_vel_topic)
        cancel_topic = "/{}/move_base/cancel".format(ns)
        clear_service = "/{}/move_base/clear_costmaps".format(ns)

        publishers[ns] = rospy.Publisher(cmd_topic, Twist, queue_size=10)
        cancel_publishers[ns] = rospy.Publisher(cancel_topic, GoalID, queue_size=1)

        if clear_costmaps:
            try:
                rospy.wait_for_service(clear_service, timeout=0.5)
                clear_services[ns] = rospy.ServiceProxy(clear_service, Empty)
            except rospy.ROSException:
                rospy.logwarn("[%s] clear_costmaps service not available", ns)

    rospy.sleep(0.1)

    if cancel_nav:
        cancel_msg = GoalID()
        for ns, pub in cancel_publishers.items():
            pub.publish(cancel_msg)
            rospy.loginfo("[%s] sent move_base cancel", ns)

    if clear_costmaps:
        for ns, srv in clear_services.items():
            try:
                srv()
                rospy.loginfo("[%s] cleared costmaps", ns)
            except rospy.ServiceException as exc:
                rospy.logwarn("[%s] clear_costmaps failed: %s", ns, exc)

    stop_msg = Twist()
    rate = rospy.Rate(rate_hz)
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec()
        if elapsed >= stop_duration:
            break
        for pub in publishers.values():
            pub.publish(stop_msg)
        rate.sleep()

    for pub in publishers.values():
        pub.publish(stop_msg)

    rospy.loginfo("Stop command sent to %d robots", len(namespaces))


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
