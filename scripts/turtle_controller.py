#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import turtlewriter.msg
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from random import random as rand

def to_radius(degree):
    return degree * 3.1415926 // 180.0

class ToXyAction(object):

    _feedback = turtlewriter.msg.ToXyFeedback()
    _result = turtlewriter.msg.ToXyResult()

    # msgs
    _vel = Twist()
    _pose = Pose()

    # srvs
    _tele = TeleportAbsolute()

    def __init__(self, name):
        self._r = rospy.Rate(500)

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, turtlewriter.msg.ToXyAction,
                                                execute_cb=self._execute_cb,
                                                auto_start=False)
        self._as.start()

        self._pub_vel = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=1)
        self._srv_tele = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
        self._sub_pose = rospy.Subscriber("turtle1/pose", Pose, self._pose_subscriber_cb)

    def _pose_subscriber_cb(self, pose):
        # rospy.loginfo("Got pose:")
        # rospy.loginfo("  x = {}".format(pose.x))
        # rospy.loginfo("  y = {}".format(pose.y))
        # rospy.loginfo("  theta = {}".format(pose.theta))
        self._pose = pose

    def _execute_cb(self, goal):
        success = True

        # Get feedback from turtlesim/pose
        # TODO

        # Clear previous command
        self._stop_walk()

        # Get goal
        x = goal.x
        y = goal.y

        # Go to (x, y)
        dist, theta = self._get_direction(x, y)
        self._turn_theta(theta)
        success = self._go_dist(dist, 0.8)

        # Walk twice to improve precision
        dist, theta = self._get_direction(x, y)
        self._turn_theta(theta)
        success = self._go_dist(dist, 0.2)

        if success:
            self._result = True
            rospy.loginfo("Action finished!")
            self._as.set_succeeded(self._result)
        else:
            self._result = False
            rospy.loginfo("Action not finished!")
            self._as.set_aborted(self._result)

    def _stop_walk(self):
        self._vel.linear.x = 0
        self._vel.angular.z = 0
        self._pub_vel.publish(self._vel)
        self._r.sleep()

    def _turn_theta(self, theta):
        x = self._pose.x
        y = self._pose.y
        self._srv_tele(x, y, theta)
        self._r.sleep()

    def _go_xy(self, x, y, dt=1.0):
        """
        [deprecated] It's hard to deal with the precision
                     problem when using this method.
        """
        eps = 0.01 / dt

        curr_dx = x - self._pose.x
        curr_dy = y - self._pose.y
        curr_dist = math.sqrt(curr_dx**2 + curr_dy**2)

        speed = curr_dist / dt

        while curr_dist > eps and not rospy.is_shutdown():
            self._vel.linear.x = speed
            self._vel.angular.z = 0
            self._pub_vel.publish(self._vel)
            self._r.sleep()

            curr_dx = x - self._pose.x
            curr_dy = y - self._pose.y
            curr_dist = math.sqrt(curr_dx**2 + curr_dy**2)

            rospy.loginfo("Curr dist = {}".format(curr_dist))

        self._stop_walk();

        if rospy.is_shutdown():
            return False
        else:
            return True

    def _go_dist(self, dist, dt=1.0):
        speed = dist / dt
        time_start = rospy.get_time()

        while (rospy.get_time() - time_start) < dt and not rospy.is_shutdown():
            self._vel.linear.x = speed
            self._vel.angular.z = 0
            self._pub_vel.publish(self._vel)
            self._r.sleep()

        self._stop_walk();

        if rospy.is_shutdown():
            return False
        else:
            return True

    def _random_walk(self):
        self._vel.linear.x = 20 - rand() * 30
        self._vel.angular.z = 20 - rand() * 16
        self._pub.publish(self._vel)
        self._r.sleep()

    def _get_direction(self, x, y):
        curr_pose = self._pose

        dy = y - curr_pose.y
        dx = x - curr_pose.x
        dist = math.sqrt(math.pow(dy, 2) + math.pow(dx, 2))
        theta = math.atan2(dy, dx)

        return dist, theta

if __name__ == '__main__':
    try:
        rospy.init_node("turtle_controller")
        server = ToXyAction("to_xy")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
