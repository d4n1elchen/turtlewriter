#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import actionlib
import turtlewriter.msg
import os
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import SetPen
from std_srvs.srv import Empty
from xml.dom import minidom

rospack = rospkg.RosPack()

rect_points = [
    {'x': 3., 'y': 7.},
    {'x': 7., 'y': 7.},
    {'x': 7., 'y': 3.},
    {'x': 3., 'y': 3.},
    {'x': 3., 'y': 7.}
]

def read_polygon(svg_file):
    doc = minidom.parse(svg_file)  # parseString also exists

    svg = doc.getElementsByTagName('svg')[0]
    svg_height = float(svg.getAttribute('height'))
    svg_width = float(svg.getAttribute('width'))

    polygon = svg.getElementsByTagName('polygon')
    path = svg.getElementsByTagName('path')
    if len(polygon):
        polygon_strings = [poly.getAttribute('points') for poly
                           in polygon]
    elif len(path):
        path_strings = [pth.getAttribute('p') for pth
                        in path]

    doc.unlink()

    x_ratio = 11 / svg_width
    y_ratio = 11 / svg_height

    print(svg_height)
    print(svg_width)

    polygon_list = []

    for polygon in polygon_strings:
        point_list = []
        for p in polygon.split(' '):
            if p == '':
                break
            xy = p.split(',')
            x = float(xy[0]) * x_ratio
            y = (svg_height - float(xy[1])) * y_ratio
            point_list.append({'x': x, 'y': y})

        point_list.append(point_list[0])

        polygon_list.append(point_list)

    return polygon_list

def draw_polygon():
    success = True

    client = actionlib.SimpleActionClient('to_xy', turtlewriter.msg.ToXyAction)

    teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
    set_pen = rospy.ServiceProxy('turtle1/set_pen', SetPen)

    clear = rospy.ServiceProxy('clear', Empty)
    clear()

    client.wait_for_server()

    polygon_list = read_polygon(os.path.join(rospack.get_path('turtlewriter'), 'svg', 'witch_craft.svg'))

    for point_list in polygon_list:
        for index, point in enumerate(point_list):
            x = point['x']
            y = point['y']
            if index == 0:
                set_pen(0, 0, 0, 5, 1)
                rospy.loginfo('First point: ({:.2f}, {:.2f})'.format(x, y))
                teleport(x, y, 0)
            else:
                set_pen(0, 0, 0, 5, 0)
                rospy.loginfo('Move to point: ({:.2f}, {:.2f})'.format(x, y))
                goal = turtlewriter.msg.ToXyGoal(**point)

                client.send_goal_and_wait(goal)

                if not client.get_result:
                    success = False
                    break

    return success

if __name__ == '__main__':
    try:
        rospy.init_node('polygon_drawer')
        result = draw_polygon()
        if result:
            rospy.loginfo('Nice polygon!')
        else:
            rospy.loginfo('Something wrong!')
    except rospy.ROSInterruptException:
        pass
