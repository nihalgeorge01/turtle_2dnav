#!/usr/bin/env python2  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import turtlesim.msg


def handle_turtle_pose(msg, turtlename):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    if turtlename == "turtle1":
        t.child_frame_id = "base_link"
    else:
        t.child_frame_id = turtlename
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0,0,msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == '__main__':
    node_name = "turtle" + str(1) + "_tf2_broadcaster"
    rospy.init_node(node_name)
    turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/%s/pose' % turtlename,
                    turtlesim.msg.Pose,
                    handle_turtle_pose,
                    turtlename)
    rospy.spin()
