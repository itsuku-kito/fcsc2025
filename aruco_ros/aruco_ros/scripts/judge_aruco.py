#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import yaml
import os
from aruco_msgs.msg import MarkerArray
from aruco_ros.srv import JudgeMarker , JudgeMarkerResponse
from std_msgs.msg import String

CAMERA_TO_BASE_Y = -0.5  # カメラ基準とbase_frameのY方向オフセット

# 比較対象のマーカーID（YAMLから取得）
target_id1 = None
target_id2 = None

# 最新のマーカー情報を保持する辞書
latest_markers = {}

def load_ids_from_yaml(file_path):
    global target_id1, target_id2
    if not os.path.isfile(file_path):
        rospy.logerr(f"YAMLファイルが見つかりません: {file_path}")
        return False

    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
        try:
            target_id1 = data["compare_ids"]["id1"]
            target_id2 = data["compare_ids"]["id2"]
            rospy.loginfo(f"比較対象ID: {target_id1}, {target_id2}")
            return True
        except KeyError as e:
            rospy.logerr(f"YAML形式が正しくありません: {e}")
            return False

def marker_pose(data):
    global latest_markers
    latest_markers = {}  # 更新

    for marker in data.markers:
        marker_id = marker.id
        position_y = marker.pose.pose.position.y + CAMERA_TO_BASE_Y
        latest_markers[marker_id] = position_y

def handle_judge_request(req):
    if target_id1 not in latest_markers and target_id2 not in latest_markers:
        rospy.loginfo("どちらのIDも見つかりません")
        return JudgeMarkerResponse(judge_result_ID=-1)

    if target_id1 not in latest_markers:
        rospy.loginfo(f"ID {target_id1} は見つかりません")
        return JudgeMarkerResponse(judge_result_ID=target_id2)

    if target_id2 not in latest_markers:
        rospy.loginfo(f"ID {target_id2} は見つかりません")
        return JudgeMarkerResponse(judge_result_ID=target_id1)

    y1 = latest_markers[target_id1]
    y2 = latest_markers[target_id2]

    if y1 > y2:
        rospy.loginfo(f"遠くにあるマーカー: ID={target_id1}")
        return JudgeMarkerResponse(judge_result_ID=target_id1)
    else:
        rospy.loginfo(f"遠くにあるマーカー: ID={target_id2}")
        return JudgeMarkerResponse(judge_result_ID=target_id2)

def main():
    rospy.init_node('judge_marker_server')

    yaml_path = rospy.get_param("~config_file", "compare_ids.yaml")
    if not load_ids_from_yaml(yaml_path):
        rospy.logerr("YAML読み込みに失敗")
        return

    rospy.Subscriber('/mover_cam/aruco_marker_publisher/markers', MarkerArray, marker_pose, queue_size=1)

    service = rospy.Service('judge_start_1', JudgeMarker, handle_judge_request)
    rospy.loginfo("サービス judge_start_1 を起動しました")
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
