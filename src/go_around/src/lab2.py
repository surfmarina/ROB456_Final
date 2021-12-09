#!/usr/bin/env python

from math import tanh, atan2
import rospy

# Sensor message types
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# the velocity command message
from geometry_msgs.msg import Twist

GOAL = (2, 1.5)
ODOM = None

def lidar_callback(scan_msg):
    global GOAL, ODOM
    # Let's make a new twist message
    command = Twist()

    
    # Fill in the fields.  Field values are unspecified 
    # until they are actually assigned. The Twist message 
    # holds linear and angular velocities.
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0

    # Lidar properties (unpacked for your ease of use)
    # find current laser angle, max scan length, distance array for all scans, and number of laser scans
    maxAngle = scan_msg.angle_max
    minAngle = scan_msg.angle_min
    angleIncrement = scan_msg.angle_increment

    maxScanLength = scan_msg.range_max
    distances = scan_msg.ranges
    numScans = len(distances)

#    print ODOM
    # Problem 1: move the robot toward the goal

    # ODOM = (x, y, yaw)
    cur_x = ODOM[0]
    cur_y = ODOM[1]
    cur_yaw = ODOM[2]

    end_x = GOAL[0]
    end_y = GOAL[1]
    
    move_x = end_x - cur_x
    move_y = end_y - cur_y
    move_heading = atan2(move_y, move_x) - cur_yaw

    command.linear.x = 0.15
    command.angular.z = move_heading # Positive=LEFT, Negative = RIGHT

#    goal_distance = pow((pow(move_x, 2) + pow(move_y, 2)), 0.5)
    if ((abs(move_x) < 0.15) and (abs(move_y) < 0.15)):
        command.linear.x = 0
#    print(cur_x, cur_y, cur_yaw, move_x, move_y, move_heading)
    print(move_x, move_y, move_heading)
    # End problem 1

    currentLaserTheta = minAngle
    forwardDistance = {} # Index 0-4. Angles from -45 deg to 45 deg. -45, 22, 0, 22, 45
    # for each laser scan
    for i, scan in enumerate(distances):
        # for each laser scan, the angle is currentLaserTheta, the index is i, and the distance is scan
        # Problem 2: avoid obstacles based on laser scan readings
        objLeft = False
        objRight = False
        objWayRight = False
        objWayLeft = False
        objSideLeft = False
        objSideRight = False
        objForward = False

        if (i == 359):
            forwardDistance[0] = float(distances[59]) # 45 deg left
            if (forwardDistance[0] < 0.25): # object .25m ahead on far left 0.35
                objSideLeft = True
            forwardDistance[1] = float(distances[44]) # 45 deg left
            if (forwardDistance[1] < 0.35): # object .25m ahead on far left 0.35
                objWayLeft = True
            forwardDistance[2] = float(distances[21]) # 22 deg left
            if (forwardDistance[2] < 0.27): # object .25m ahead on left 0.27
                objLeft = True
            forwardDistance[3] = float(distances[0]) # Straight ahead
            if (forwardDistance[3] < 0.30): # object .25m ahead in center 0.25
                objForward = True
            forwardDistance[4] = float(distances[359-23]) # 22 deg right
            if (forwardDistance[4] < 0.27): # object .25m ahead on right 0.27
                objRight = True
            forwardDistance[5] = float(distances[359-45]) # 45 deg right
            if (forwardDistance[5] < 0.35): # object .25m ahead on far right 0.35
                objWayRight = True
            forwardDistance[6] = float(distances[359-60]) # 45 deg right
            if (forwardDistance[6] < 0.25): # object .25m ahead on far right 0.35
                objSideRight = True

            if (forwardDistance[3] > forwardDistance[1]):
                turnRight = False
            else:
                turnRight = True
            print(objSideLeft, objWayLeft, objLeft, objForward, objRight, objWayRight, objSideRight)
            modLinearX = command.linear.x
            modAngularZ = command.angular.z

#            objLeft = objLeft + objWayLeft
#            objRight = objRight + objWayRight

            if objForward and turnRight: # Turn right
                if command.linear.x > 0:
                    modLinearX = 0.05
                modAngularZ = modAngularZ - 1.57
                print("Turning sharp right")
            elif objWayLeft and objLeft: # Turn right
                if command.linear.x > 0:
                    modLinearX = 0.05
                modAngularZ = modAngularZ - 1.57
                print("Turning right")
            elif objWayLeft: # Turn right
                if command.linear.x > 0:
                    modLinearX = 0.1
                modAngularZ = modAngularZ - 1
                print("Turning small right")
            elif objForward and not turnRight: # Turn left
                if command.linear.x > 0:
                    modLinearX = 0.05
                modAngularZ = modAngularZ + 1.57
                print("Turning sharp left")
            elif objWayRight and objRight: # Turn left
                if command.linear.x > 0:
                    modLinearX = 0.1
                modAngularZ = modAngularZ + 1.57
                print("Turning left")
            elif objWayRight: # Turn left
                if command.linear.x > 0:
                    modLinearX = 0.15
                modAngularZ = modAngularZ + 1
                print("Turning small left")
            
            if objSideLeft:
                modAngularZ = modAngularZ - 1
            if objSideRight:
                modAngularZ = modAngularZ + 1

            print(modLinearX, modAngularZ)
            command.linear.x = modLinearX
            command.angular.z = modAngularZ

        # End problem 2
        # After this loop is done, we increment the currentLaserTheta
        currentLaserTheta = currentLaserTheta + angleIncrement

    pub.publish(command)


def odom_callback(msg):
    """
    Subscribes to the odom message, unpacks and transforms the relevent information, and places it in the global variable ODOM
    ODOM is structured as follows:
    ODOM = (x, y, yaw)

    :param: msg: Odometry message
    :returns: None
    """
    global ODOM
    position = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    (r, p, yaw) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    ODOM = (position.x, position.y, yaw)


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('lab2', log_level=rospy.DEBUG)

    # subscribe to sensor messages
    lidar_sub = rospy.Subscriber('/scan', LaserScan, lidar_callback)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)

    # publish twist message
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Turn control over to ROS
    rospy.spin()
