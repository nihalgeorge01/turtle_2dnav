#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist

def publish_velocity(data):
    global velocity_publisher
    velocity_publisher.publish(data)

def move2goal():
    rospy.init_node('base_controller', anonymous=True)
    global velocity_publisher
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
    vel_msg = Twist()
    pose_subscriber = rospy.Subscriber('cmd_vel', Twist, publish_velocity)
    rate = rospy.Rate(60)

    rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        move2goal()
    except rospy.ROSInterruptException:
        pass
