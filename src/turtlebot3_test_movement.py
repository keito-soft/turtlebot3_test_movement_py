#!/usr/bin/env python3

import rospy

from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped,Pose,Point,Quaternion

class Turtlebot3Movement:
    """
    move_base/statusをSubscribeし、状態が
    - 起動直後
    - 移動完了
    のいずれかの場合、move_base_simple/goal に次のゴールをPublishします。
    """
    def __init__(self):
        rospy.init_node('turtlebot3_movement', anonymous=True)
        self.rate = rospy.Rate(1)

        self._goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self._result_sub = rospy.Subscriber("move_base/status", GoalStatusArray, self._result_callback)

        self._current_status = 0

        self._current_goal_id = 0
        self._goal_poses = [Pose(Point(-2.0, -0.5, 0.0), Quaternion(0, 0, 0, 1)), Pose(Point(2.0, 0.5, 0.0), Quaternion(0, 0, 0, 1)),]
        self._goal_total_number = len(self._goal_poses)

    def _result_callback(self, msg):
        if len(msg.status_list) == 0:
            return

        if self._current_status == msg.status_list[0].status:
            return

        self._current_status = msg.status_list[0].status

        if self._current_status == 3:
            self._set_next_goal()
    
    def control_loop(self):
        if self._current_status == 0:
            self._set_next_goal()
        self.rate.sleep()

    def _set_next_goal(self):
        next_goal_id = (self._current_goal_id + 1) % self._goal_total_number
        next_goal = PoseStamped()
        next_goal.pose = self._goal_poses[next_goal_id]
        next_goal.header.stamp = rospy.Time.now()
        next_goal.header.frame_id = 'map'
        self._goal_pub.publish(next_goal)
        self._current_goal_id = next_goal_id

if __name__ == '__main__':
    try:
        movement = Turtlebot3Movement()

        while not rospy.is_shutdown():
            movement.control_loop()

    except rospy.ROSInterruptException:
        pass