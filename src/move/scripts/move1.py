#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_to_goal(x, y, w):
    # 初始化action client，用於與move_base服務通信
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # 創建一個新的目標
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"  # 使用base_link作為參考框架
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = w

    # 發送目標給move_base
    client.send_goal(goal)

    # 等待結果
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        # 獲取結果
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('move_base_goal_py')
        # 設置目標點的位置和姿態，這裡以(1.0, 1.0, 1.0)為例
        result = move_to_goal(-1.0, 0.0, 1.0)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
