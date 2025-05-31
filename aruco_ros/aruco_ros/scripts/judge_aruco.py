#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from aruco_msgs.msg import MarkerArray

# cameraフレーム（mover_cam）からbase_frameへのオフセット（仮定）
CAMERA_TO_BASE_Y = -0.5  # [m] カメラの位置がbase_frameより前にあると仮定

def marker_pose(data):
    id1_y = None
    id2_y = None

    for marker in data.markers:
        if marker.id == 1:
            id1_y = marker.pose.pose.position.y + CAMERA_TO_BASE_Y
        elif marker.id == 2:
            id2_y = marker.pose.pose.position.y + CAMERA_TO_BASE_Y

    if id1_y is None and id2_y is None:
        rospy.loginfo("ID 1 と ID 2 のマーカーが見つかりませんでした")
    elif id1_y is None:
        rospy.loginfo("ID 1 のマーカーが見つかりませんでした")
    elif id2_y is None:
        rospy.loginfo("ID 2 のマーカーが見つかりませんでした")
    else:
        if id1_y > id2_y:
            rospy.loginfo(f"ID 1 のマーカーの方が遠くにあります (Y={id1_y:.2f} m > {id2_y:.2f} m)")
        elif id2_y > id1_y:
            rospy.loginfo(f"ID 2 のマーカーの方が遠くにあります (Y={id2_y:.2f} m > {id1_y:.2f} m)")
        else:
            rospy.loginfo(f"ID 1 と ID 2 のマーカーは同じ距離にあります (Y={id1_y:.2f} m)")

def main():
    rospy.init_node('compare_marker_id1_id2')
    rospy.Subscriber('/mover_cam/aruco_marker_publisher/markers', MarkerArray, marker_pose, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
