#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from aruco_msgs.msg import Marker, MarkerArray

def marker_pose(data):   
    br_ar_tf = tf.TransformBroadcaster()

    # tf_result = tf.transformations.quaternion_from_euler(0,0,0)
    
    for i in range(len(data.markers)):
        # if (data.markers[i].id == 11 or data.markers[i].id == 12):

        # マーカーのTF登録
        br_ar_tf.sendTransform((data.markers[i].pose.pose.position.x, data.markers[i].pose.pose.position.y, data.markers[i].pose.pose.position.z),
                                (data.markers[i].pose.pose.orientation.x, data.markers[i].pose.pose.orientation.y, data.markers[i].pose.pose.orientation.z, data.markers[i].pose.pose.orientation.w),
                                rospy.Time.now(), "ar_marker_" + str(data.markers[i].id), "mover_cam")

def main():
    rospy.init_node('aruco_tf_broadcaster')
    
    while not rospy.is_shutdown():
        rospy.Subscriber('/mover_cam/aruco_marker_publisher/markers', MarkerArray, marker_pose, queue_size=1)

        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass