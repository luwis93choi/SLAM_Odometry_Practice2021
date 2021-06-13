#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Imu
from gazebo_msgs.msg import LinkStates

import tf2_ros
import geometry_msgs.msg

tf_broadcaster = tf2_ros.TransformBroadcaster()
tf_msgs_sensor2base = geometry_msgs.msg.TransformStamped()
tf_msgs_base2odom = geometry_msgs.msg.TransformStamped()

def imu_subscribe_callback(data):
    rospy.loginfo(rospy.get_caller_id() + '%s', data.orientation)

    tf_msgs_base2odom.header.stamp = rospy.Time.now()
    tf_msgs_base2odom.header.frame_id = 'odom'
    tf_msgs_base2odom.child_frame_id = 'base_link'
    tf_msgs_base2odom.transform.translation.x = 0
    tf_msgs_base2odom.transform.translation.y = 0
    tf_msgs_base2odom.transform.translation.z = 0
    tf_msgs_base2odom.transform.rotation = data.orientation
    tf_broadcaster.sendTransform(tf_msgs_base2odom)
    
def gazebo_model_pose_callback(data):
    rospy.loginfo(rospy.get_caller_id() + '%s', data.orientation)

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    rate = rospy.Rate(30)

    rospy.Subscriber('/imu/data', Imu, imu_subscribe_callback)
    # rospy.Subscriber('/gazebo/link_states', LinkStates, )

    while not rospy.is_shutdown():

        tf_msgs_sensor2base.header.stamp = rospy.Time.now()
        tf_msgs_sensor2base.header.frame_id = 'base_link'
        tf_msgs_sensor2base.child_frame_id = 'lidar_top'
        tf_msgs_sensor2base.transform.translation.x = 0.15
        tf_msgs_sensor2base.transform.translation.y = 0
        tf_msgs_sensor2base.transform.translation.z = 0.2045
        tf_msgs_sensor2base.transform.rotation.x = 0
        tf_msgs_sensor2base.transform.rotation.y = 0
        tf_msgs_sensor2base.transform.rotation.z = 0
        tf_msgs_sensor2base.transform.rotation.w = 1
        tf_broadcaster.sendTransform(tf_msgs_sensor2base)

        rate.sleep()