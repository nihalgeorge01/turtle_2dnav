#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import math
from turtlesim.msg import Pose
import tf_conversions
import tf2_ros
import numpy

obs_xy = [-100,-100]

def obs_cb(data):
    global obs_xy
    obs_pose = data
    try:
        trans = tfBuffer.lookup_transform('turtle1', 'turtle2',  rospy.Time.now(), rospy.Duration(1.0))
        obs_xy[0] = obs_pose.x - trans.tranforms.transform.translation.x
        obs_xy[1] = obs_pose.y - trans.tranforms.transform.translation.y
    except:
        pass


rospy.init_node('laser_scan_publisher')

scan_pub = rospy.Publisher('scan', LaserScan, queue_size=1)
try:
    obs_sub = rospy.Subscriber('turtle2/pose', Pose, obs_cb)
except:
    pass
tfBuffer = tf2_ros.Buffer()
turtle1_tf2_listener = tf2_ros.TransformListener(tfBuffer)

num_readings = 36
laser_frequency = 40
dist_increment = 0.1
covariance = 0.5

def euclidean_distance(p1, p2):
    dist = ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
    return dist

r = rospy.Rate(10.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    scan = LaserScan()

    scan.header.stamp = current_time
    scan.header.frame_id = 'turtle1'
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = math.pi / num_readings
    scan.time_increment = (1.0 / laser_frequency) / (num_readings)
    scan.range_min = 0.0
    scan.range_max = 15.0

    scan.ranges = []
    scan.intensities = []
    for i in range(0, num_readings):
        slope = (20*i*math.pi)/360
        for j in numpy.arange(0.1, scan.range_max, dist_increment):
            

            curr = [j, slope*j]
            curr_dist = euclidean_distance(curr, obs_xy)
            if curr_dist < covariance:
                scan.ranges.append(curr_dist)
                scan.intensities.append(1)
                break
        else:
            scan.ranges.append(scan.range_max)
            scan.intensities.append(0)
            
            #scan.ranges.append(1.0 * count)  # fake data
            #scan.intensities.append(1)  # fake data

    scan_pub.publish(scan)
    r.sleep()
