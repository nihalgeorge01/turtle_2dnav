#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import LaserScan
import math
from turtlesim.msg import Pose
import tf_conversions
import tf2_ros
import numpy as np

obs_xy = [-100,-100,0]
turt_xy = [0,0,0]
spread = 10
obs_radius = 0.6

def obs_cb(data):
    global obs_xy
    obs_pose = data
    '''
    trans = tfBuffer.lookup_transform('turtle2', 'base_link',  rospy.Time.now(), rospy.Duration(1.0))
    theta = tf_conversions.transformations.euler_from_quaternion([trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w])
    print(theta)
    theta = theta[2]
    '''
    obs_xy[2] = obs_pose.theta
    obs_xy[0] = obs_pose.x
    obs_xy[1] = obs_pose.y
    '''
    obs_xy[0] = trans.transform.translation.x * math.cos(theta) - trans.transform.translation.y * math.sin(theta)
    obs_xy[1] = trans.transform.translation.y * math.sin(theta) + trans.transform.translation.x * math.cos(theta)
    '''
    #print(obs_xy)

def turt_cb(data):
    global turt_xy
    turt_pose = data
    turt_xy[0] = turt_pose.x
    turt_xy[1] = turt_pose.y
    turt_xy[2] = turt_pose.theta

rospy.init_node('laser_scan_publisher')

scan_pub = rospy.Publisher('scan', LaserScan, queue_size=1)
obs_sub = rospy.Subscriber('turtle2/pose', Pose, obs_cb)
turt_sub = rospy.Subscriber('turtle1/pose',Pose,turt_cb)
tfBuffer = tf2_ros.Buffer()
turtle1_tf2_listener = tf2_ros.TransformListener(tfBuffer)

num_readings = 360
laser_frequency = 60
dist_increment = 0.1
covariance = 0.5
rate = rospy.Rate(60)

def euclidean_distance(p1, p2):
    dist = ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
    return dist

def get_point_dist(obs_dist, obs_radius):
    spread = math.asin(min(1.0,obs_radius/obs_dist))
    spread_readings = int(num_readings * (math.degrees(2*spread)/360))

    distances = [100.0 for i in range(spread_readings)]

    for i in range(int(-spread_readings/2)+1, int(spread_readings/2)):
        alpha = abs(i*(2*spread/spread_readings))
        point_dist = obs_radius * math.sin(alpha + math.asin((obs_dist/obs_radius)*math.sin(alpha)))/(math.sin(alpha)+0.000001)
        #distances[int(spread_readings/2)-i] = point_dist
        distances[i] = point_dist
        if point_dist == 0.0:
            distances[int(spread_readings/2)+i] = distances[int(spread_readings/2)+i-1]
            distances[i] = min(distances[:i] + distances[i+1:])
    if 100.0 in distances:
        distances[distances.index(100.0)] = min(distances)
    
    '''
    if len(distances) > 0:
        distances[distances.index(max(distances))] = min(distances)
    '''

    return distances

while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    scan = LaserScan()

    scan.header.stamp = current_time
    scan.header.frame_id = 'base_link'
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = 2.0*math.pi / num_readings
    scan.time_increment = (1.0 / laser_frequency) / (num_readings)
    scan.range_min = 0.0
    scan.range_max = 100.0
    scan.ranges = [100.0 for i in range(num_readings)]
    scan.intensities = [0.0 for i in range(num_readings)]
    
    obs_vec = [obs_xy[0] - turt_xy[0], obs_xy[1] - turt_xy[1], 0]
    
    #print(turt_xy[2])
    
    curr_dist = euclidean_distance([0,0], obs_vec[:2])
    
    turt_vec = [math.cos(turt_xy[2]),math.sin(turt_xy[2])]
    dot = (turt_vec[0] * obs_vec[0]) + (turt_vec[1] * obs_vec[1])
    cross_mag = (turt_vec[0] * obs_vec[1] - turt_vec[1] * obs_vec[0])

    if cross_mag < 0:
        sgn = -1
    else:
        sgn = 1

    obs_vec[2] = sgn * math.acos(dot/curr_dist)

    index = int((math.degrees(obs_vec[2])/(360.0/num_readings))+(180.0))
    distances = get_point_dist(curr_dist,obs_radius)
    
    spread = math.asin(min(1,obs_radius/curr_dist))
    spread_readings = int(num_readings * (2*math.degrees(spread)/360))
    scan.ranges[index-int(spread_readings/2)+1:index+int(spread_readings/2)] = distances

    scan.intensities[index-int(spread_readings/2)+1:index+int(spread_readings/2)] = [1.0 for i in range(spread_readings)]
    #scan.ranges = scan.ranges
    #scan.intensities = scan.intensities
    #print(math.degrees(obs_vec[2]))
    '''
    

    
    #print(index)  
    '''
    scan_pub.publish(scan)
    rate.sleep()
