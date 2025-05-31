#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from aruco_msgs.msg import Marker, MarkerArray

# TF Broadcaster（マーカーのTF登録用）
br_ar_tf = tf.TransformBroadcaster()
# TF Listener（変換用）
listener = tf.TransformListener()

def marker_pose(data):
    rospy.sleep(0.1)  # TFの準備時間

    # マーカーが1つだけの場合
    if len(data.markers) == 1:
        rospy.loginfo("マーカーが1つしか見つかりません")
    elif len(data.markers) == 0:
        rospy.loginfo("マーカーが検出されませんでした")
        return

    max_y = float('-inf')
    farthest_marker = None
    farthest_position = None

    for marker in data.markers:
        marker_id = marker.id
        marker_frame = "ar_marker_" + str(marker_id)

        # まずマーカーのTFを登録する
        pos = marker.pose.pose.position
        ori = marker.pose.pose.orientation
        br_ar_tf.sendTransform(
            (pos.x, pos.y, pos.z),
            (ori.x, ori.y, ori.z, ori.w),
            rospy.Time.now(),
            marker_frame,
            "mover_cam"
        )

    # TF登録が終わってから変換
    for marker in data.markers:
        marker_frame = "ar_marker_" + str(marker.id)
        try:
            time = rospy.Time(0)
            listener.waitForTransform("base_frame", marker_frame, time, rospy.Duration(0.5))
            (trans, rot) = listener.lookupTransform("base_frame", marker_frame, time)
            y_pos = trans[1]  # Y座標

            if y_pos > max_y:
                max_y = y_pos
                farthest_marker = marker.id
                farthest_position = trans

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"{marker_frame} の変換に失敗: {str(e)}")
            continue

    if farthest_marker is not None and len(data.markers) > 1:
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
