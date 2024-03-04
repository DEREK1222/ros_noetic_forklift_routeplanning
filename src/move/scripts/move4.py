#!/usr/bin/env python3
import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class TF2NavNode(object):
    def __init__(self):
        rospy.init_node('tf2_nav_node', anonymous=True)

        self.tf_listener = tf.TransformListener()

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
                # 使用 base_link 坐标系创建目标姿态，确保目标总在机器人前方
                target_pose = PoseStamped()
                target_pose.header.frame_id = "base_link"
                target_pose.header.stamp = rospy.Time.now()
                target_pose.pose.position.x = 1.0  # 在机器人前方1米
                target_pose.pose.orientation.w = 1.0  # 保持机器人当前的朝向
                
                # 因为目标位置是相对于 base_link 设置的，所以不需要转换到 map 坐标系
                # 发送目标
                self.send_goal(target_pose)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.loginfo("Could not transform 'base_link' to 'map': %s" % str(e))

            self.execution_count += 1  # 增加计数器
        else:
            self.timer.cancel()  # 停止定时器

    def send_goal(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # 这里需要确保目标姿态最终转换到地图坐标系
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 将目标从 base_link 转换到 map 坐标系
        try:
            transformed_pose = self.tf_listener.transformPose("map", pose)
            goal.target_pose.pose = transformed_pose.pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo("Failed to transform pose from 'base_link' to 'map': %s" % str(e))
            return

        self.nav_client.send_goal(goal, feedback_cb=self.feedback_callback)
        self.nav_client.wait_for_result()
        result = self.nav_client.get_result()
        rospy.loginfo("Result: %s" % str(result))

    def feedback_callback(self, feedback):
        # 处理反馈信息
        pass

def main():
    try:
        tf2_nav_node = TF2NavNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
