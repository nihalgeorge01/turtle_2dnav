#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    msg = Twist()
    #print(data)
    msg.linear.x = int(data.axes[0]*255)
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = int(-1*data.axes[1]*255)

    pub.publish(msg)

rospy.init_node('joycon', anonymous=True)
pub = rospy.Publisher('balls', Twist, queue_size=10)
rospy.Subscriber("joy", Joy, callback)


rate = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
    #rospy.loginfo(hello_str)
    rate.sleep()
rospy.spin()


