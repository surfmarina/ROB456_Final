#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from std_msgs.msg import Bool

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point

import aStar as aS

import numpy as np

class GlobalPlannar(object):
    def __init__(self):
        self.map_data = OccupancyGrid()
        self.map_odom_pos = Pose()
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/map_odom', Pose, self.odom_callback)
        self.pub = rospy.Publisher('/recieved_map', Bool, queue_size=10)

    def map_callback(self, msg):
        rospy.loginfo('In map callback')

        self.map_data = msg.data

        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

        # a* begin, move to seperate node

        x = np.rint(self.map_odom_pos.position.x / msg.info.resolution)
        y = np.rint(self.map_odom_pos.position.y / msg.info.resolution)

        rospy.loginfo("Cell Position: (" + str(x) + "," + str(y) + ")")

        a = aS.aStar([100, 100], [250, 325], img=data)

        # aPath = a.path(a.run())

        # rospy.loginfo(np.shape(aPath))


        self.pub.publish(True)

    def odom_callback(self, msg):
        self.map_odom_pos = msg
        # rospy.loginfo('In odom callback' + str(msg))

if __name__ == '__main__':
    rospy.init_node('class_example')
    gp = GlobalPlannar()
    rospy.spin()

