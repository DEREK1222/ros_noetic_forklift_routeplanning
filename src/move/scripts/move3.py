#!/usr/bin/env python3
import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class TF2NavNode(object):
    def __init__(self):
        rospy.init_node('tf2_nav_node', anonymous=True)

        self.tf_listener = tf.TransformListener()

        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'map'
        self.target_pose.pose.position.x = 1.0  # 假设目标点
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.orientation.w = 1.0

        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.nav_client.wait_for_server()

        # 设定执行次数限制
        self.execution_count = 0
        self.execution_limit = 1  # 仅执行1次

        # 定时器设置为每2秒执行一次
        rospy.Timer(rospy.Duration(2.0), self.timer_callback, oneshot=False)

    def timer_callback(self, event):
        if self.execution_count < self.execution_limit:
            try:
                (trans,rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))

                self.target_pose.pose.position.x -= trans[0]
                self.target_pose.pose.position.y -= trans[1]
                self.target_pose.pose.position.z = 0.0

                # 在ROS 1中，tf模块直接提供了旋转目标方向的功能
                quaternion = rot
                self.target_pose.pose.orientation.x = quaternion[0]
                self.target_pose.pose.orientation.y = quaternion[1]
                self.target_pose.pose.orientation.z = quaternion[2]
                self.target_pose.pose.orientation.w = quaternion[3]

                self.send_goal(self.target_pose)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.loginfo(f"Could not transform 'base_link' to 'map': {e}")

            self.execution_count += 1  # 增加计数器
        else:
            rospy.signal_shutdown("Execution limit reached.")

    def send_goal(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose = pose

        self.nav_client.send_goal(goal, feedback_cb=self.feedback_callback)
        self.nav_client.wait_for_result()
        result = self.nav_client.get_result()
        rospy.loginfo(f"Result: {result}")

    def feedback_callback(self, feedback):
        # 可以在这里处理反馈信息
        pass

def main():
    try:
        tf2_nav_node = TF2NavNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
