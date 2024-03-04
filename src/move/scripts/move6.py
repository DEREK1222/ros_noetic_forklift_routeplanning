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
            # Ensure all transforms are up to date
            rospy.sleep(1.0)

            # Get the transform from /base_link to /marker_0
            trans_base_to_marker = tf_buffer.lookup_transform('base_link', 'marker_0', rospy.Time(0), rospy.Duration(1.0))
            
            # Calculate the yaw angle to face /marker_0
            yaw = math.atan2(trans_base_to_marker.transform.translation.y, trans_base_to_marker.transform.translation.x)
            
            # Convert yaw angle to a quaternion
            quat = quaternion_from_euler(0, 0, yaw)

            # Set the target position to be in front of /marker_0
            distance_in_front_of_marker = 0.30  # Distance to stop in front of the marker
            target_pose_base = PoseStamped()
            target_pose_base.header.frame_id = "base_link"
            target_pose_base.header.stamp = rospy.Time.now()
            target_pose_base.pose.position.x = trans_base_to_marker.transform.translation.x - distance_in_front_of_marker * math.cos(yaw)
            target_pose_base.pose.position.y = trans_base_to_marker.transform.translation.y - distance_in_front_of_marker * math.sin(yaw)
            target_pose_base.pose.position.z = 0  # Assuming the target is on the ground
            target_pose_base.pose.orientation.x = quat[0]
            target_pose_base.pose.orientation.y = quat[1]
            target_pose_base.pose.orientation.z = quat[2]
            target_pose_base.pose.orientation.w = quat[3]

            # Use move_base to send the target position
            result = move_to_goal(target_pose_base)
            if result:
                rospy.loginfo("Goal execution done.")

            break  # Exit the loop once the task is done

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF error: {}".format(e))
            continue

if __name__ == '__main__':
    try:
        calculate_and_move()
    except rospy.ROSInterruptException:
        pass
