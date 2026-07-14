#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bridge ego_planner traj_server geometry_msgs/Twist cmd_vel into the
vehicle_simulator geometry_msgs/TwistStamped command interface.
"""

import rospy
from geometry_msgs.msg import Twist, TwistStamped


class SimulationCmdVelBridge:
    def __init__(self) -> None:
        rospy.init_node("simulation_cmd_vel_bridge")

        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel")
        self.output_topic = rospy.get_param("~output_topic", "/planning_cmd_vel")
        self.frame_id = rospy.get_param("~frame_id", "vehicle")

        self.publisher = rospy.Publisher(self.output_topic, TwistStamped, queue_size=10)
        self.subscriber = rospy.Subscriber(self.input_topic, Twist, self.callback, queue_size=20)

        rospy.loginfo(
            "[util_3dnav simulation] bridging %s geometry_msgs/Twist -> %s geometry_msgs/TwistStamped",
            self.input_topic,
            self.output_topic,
        )

    def callback(self, msg: Twist) -> None:
        stamped = TwistStamped()
        stamped.header.stamp = rospy.Time.now()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.publisher.publish(stamped)


if __name__ == "__main__":
    try:
        SimulationCmdVelBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
