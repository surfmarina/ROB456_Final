#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from std_msgs.msg import Bool

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import aStar as aS

import numpy as np

class GlobalPlannar(object):
    def __init__(self):
        self.map_data = None
        self.odom_pos = None
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('/recieved_map', Bool, queue_size=10)
        self.mark_pub = rospy.Publisher('/marker', Marker, queue_size=2)

    def map_callback(self, msg):
        rospy.loginfo('In map callback')

        self.map_data = msg.data

        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

        a = aS.aStar([100, 100], [250, 325], img=data)
        aPath = a.path(a.run())

        rospy.loginfo(np.shape(aPath))

        # Astar goes here

        self.pub.publish(True)

    def odom_callback(self, msg):
        # rospy.loginfo('In odom callback' + str(self.odom_pos))
        self.odom_pos = msg.pose.pose

def draw_points(points, pub):
    """
    Plots an array of points [(x, y)...] in rviz

    :param: points iterable of (x, y) pairs. If a numpy array, shape should be (n, 2)
    :return: None
    """
    msg = Marker()
    # Marker header specifies what (and when) it is drawn relative to
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    # uint8 POINTS=8
    msg.type = 8
    # Action, Create/Update
    msg.action = 0
    # Disappear after 1sec. Comment this line out to make them persist indefinitely
    msg.lifetime = rospy.rostime.Duration(1, 0)
    # Set marker visual properties
    msg.color.b = 1.0
    msg.color.a = 1.0
    msg.scale.x = 0.2
    msg.scale.y = 0.2
    # Copy  (x, y) into message and publish
    for (x, y) in points:
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.1 # Places all points 10cm above the ground
        msg.points.append(p)
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('class_example')
    gp = GlobalPlannar()
    rospy.spin()

