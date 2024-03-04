#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler
import math

def move_to_goal(target_pose_base):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose = target_pose_base

    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
    else:
        return client.get_result()

def calculate_and_move():
    rospy.init_node('calculate_and_move_to_marker')

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    while not rospy.is_shutdown():
        try:
            # 确保所有变换都是最新的
            rospy.sleep(1.0)

            # 从/marker_0到/camera_color_optical_frame的变换
            trans_marker_to_camera = tf_buffer.lookup_transform('camera_color_optical_frame', 'marker_0', rospy.Time(0), rospy.Duration(1.0))
            
            # 从/camera_color_optical_frame到/base_link的变换
            trans_camera_to_base = tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(1.0))
            
            # 通过组合这两个变换，计算/marker_0相对于/base_link的位置
            target_pose_marker = PoseStamped()
            target_pose_marker.header.frame_id = 'marker_0'
            target_pose_marker.pose.orientation.w = 1.0  # 假设面向marker的方向
            target_pose_marker = tf2_geometry_msgs.do_transform_pose(target_pose_marker, trans_marker_to_camera)
            target_pose_base = tf2_geometry_msgs.do_transform_pose(target_pose_marker, trans_camera_to_base)

            # 朝向计算
            yaw = math.atan2(target_pose_base.pose.position.y, target_pose_base.pose.position.x)
            quat = quaternion_from_euler(0, 0, yaw)
            target_pose_base.pose.orientation.x = quat[0]
            target_pose_base.pose.orientation.y = quat[1]
            target_pose_base.pose.orientation.z = quat[2]
            target_pose_base.pose.orientation.w = quat[3]

            # 使用move_base发送目标位置
            move_to_goal(target_pose_base)

            break  # 任务完成后退出循环

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF error: {}".format(e))
            continue

if __name__ == '__main__':
    try:
        calculate_and_move()
    except rospy.ROSInterruptException:
        pass
