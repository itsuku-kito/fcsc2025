#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from aruco_msgs.msg import Marker, MarkerArray

"""
最も遠いarucoマーカーの判定をするコード
arucoがtfに登録されるため、登録されたものの中からbase_frame基準で最も遠いIDを表示する
"""

def marker_pose(data):
    listener = tf.TransformListener()
    rospy.sleep(0.1)  # tfバッファに時間を与える（重要）

    max_y = float('-inf')
    farthest_marker = None
    farthest_position = None

    for marker in data.markers:
        marker_frame = "ar_marker_" + str(marker.id)
        try:
            time = rospy.Time(0)
            listener.waitForTransform("base_frame", marker_frame, time, rospy.Duration(0.5))
            (trans, rot) = listener.lookupTransform("base_frame", marker_frame, time)
            y_pos = trans[1]  # y座標

            if y_pos > max_y:
                max_y = y_pos
                farthest_marker = marker.id
                farthest_position = trans

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"{marker_frame} の変換に失敗: {str(e)}")
            continue

    if farthest_marker is not None:
        rospy.loginfo(f"最も遠いマーカー: ID={farthest_marker}, base_frame基準の位置={farthest_position}")

def main():
    rospy.init_node('aruco_tf_far_marker_checker')
    
    rospy.Subscriber('/mover_cam/aruco_marker_publisher/markers', MarkerArray, marker_pose, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
