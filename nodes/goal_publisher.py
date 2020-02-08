#!/usr/bin/env python
import rospy
import tf_conversions
from geometry_msgs.msg import PoseStamped

goal_theta = 0.0

def goal_publisher():
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.init_node('goal_pub', anonymous=True)
    rate = rospy.Rate(60) 
    global theta
    ct = 0

    while ct <1000 and(not rospy.is_shutdown()):
        goal = PoseStamped();
        ct+=1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "odom"

        goal.pose.position.x = 1.0
        goal.pose.position.y = 5.0
        goal.pose.position.z = 0.0
        
        goal_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, goal_theta) 
        
        goal.pose.orientation.x = goal_quat[0]
        goal.pose.orientation.y = goal_quat[1]
        goal.pose.orientation.z = goal_quat[2]
        goal.pose.orientation.w = goal_quat[3]

        pub.publish(goal)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        goal_publisher()
    except rospy.ROSInterruptException:
        pass
