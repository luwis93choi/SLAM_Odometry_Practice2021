#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Imu
from gazebo_msgs.msg import LinkStates
import geometry_msgs.msg

import tf2_ros

import message_filters

tf_broadcaster = tf2_ros.TransformBroadcaster()
tf_msgs_sensor2base = geometry_msgs.msg.TransformStamped()
tf_msgs_base2odom = geometry_msgs.msg.TransformStamped()

def tf_update(imu_data, gazebo_link_data):

    # rospy.loginfo(rospy.get_caller_id() + '\ntranslation : {}\norientation : {}'.format(gazebo_link_data.pose[1].position, imu_data.orientation))
    
    base_idx = gazebo_link_data.name.index('2WD::base_link')      # Find the index of 2WD base link in Gazebo Link States
    tf_msgs_base2odom.header.stamp = rospy.Time.now()
    tf_msgs_base2odom.header.frame_id = 'odom'
    tf_msgs_base2odom.child_frame_id = 'base_link'
    tf_msgs_base2odom.transform.rotation = imu_data.orientation
    tf_msgs_base2odom.transform.translation = gazebo_link_data.pose[base_idx].position
    tf_broadcaster.sendTransform(tf_msgs_base2odom)

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

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    rate = rospy.Rate(30)

    imu_sub = message_filters.Subscriber('/imu/data', Imu)
    gazebo_link_sub = message_filters.Subscriber('/gazebo/link_states', LinkStates)

    sync_subscriber = message_filters.ApproximateTimeSynchronizer([imu_sub, gazebo_link_sub], 10, 0.1, allow_headerless=True)
    sync_subscriber.registerCallback(tf_update)

    while not rospy.is_shutdown():

        rate.sleep()