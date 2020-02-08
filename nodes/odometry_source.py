#!/usr/bin/env python  

import rospy
import math
import tf_conversions
import tf2_ros
import geometry_msgs
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose


def turtle_cb(data):
    global x,y, theta, vx, vy, vth
    x = data.x
    y = data.y
    theta = data.theta
    vx = data.linear_velocity * math.cos(theta)
    vy = data.linear_velocity * math.sin(theta)
    vth = data.angular_velocity

rospy.init_node('odometry')
last_time = rospy.Time.now()
current_time = rospy.Time.now()

if __name__ == '__main__':
    
    odom_broad = tf2_ros.TransformBroadcaster()
    
    global x,y, theta, vx, vy, vth
    
    x = 1.0
    y = 1.0
    theta = 0.0
    vx = 0.0
    vy = 0.0
    vth = 0.0
    turtle_listener = rospy.Subscriber('/turtle1/pose', Pose, turtle_cb)
    odom_pub = rospy.Publisher('odom',Odometry, queue_size = 1)
    rate = rospy.Rate(60)
    
    while not rospy.is_shutdown():
        '''
        try:
            trans = tfBuffer.lookup_transform('turtle1', 'goal', rospy.Time.now(), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        '''

        dt = (current_time - last_time).to_sec()
        delta_x = (vx * math.cos(theta) - vy * math.sin(theta)) * dt;
        delta_y = (vx * math.sin(theta) + vy * math.cos(theta)) * dt;
        delta_theta = vth * dt;

        x += delta_x;
        y += delta_y;
        theta += delta_theta;
        
        current_time = rospy.Time.now()
        
        odom_trans = geometry_msgs.msg.TransformStamped()
        odom_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
        odom_trans.header.stamp = current_time
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"

        odom_trans.transform.translation.x = x
        odom_trans.transform.translation.y = y
        odom_trans.transform.translation.z = 0.0
        
        odom_trans.transform.rotation.x = odom_quat[0]
        odom_trans.transform.rotation.y = odom_quat[1]
        odom_trans.transform.rotation.z = odom_quat[2]
        odom_trans.transform.rotation.w = odom_quat[3]

        odom = Odometry()
        
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        #odom.pose.pose.orientation = odom_quat

        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3] 
        
        odom.header.frame_id = "odom"
        odom.header.stamp = current_time
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
        
        odom_pub.publish(odom)

        last_time = current_time
        rate.sleep()

