#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction

class sendGoal(object):
  def __init__(self):
    rospy.init_node('send_goal')
    self.client=actionlib.SimpleActionClient('move_base', MoveBaseAction)

    self.mygoal=[]
    self.points_num=rospy.get_param('~points_num',0)
    rospy.loginfo('points num: %d'%self.points_num)
    for i in range(self.points_num):
      self.mygoal.append(rospy.get_param('~goal_points/point_%d'%i, None))

    self.client.wait_for_server()
    self.goal=MoveBaseAction()
    self.goal.action_goal.goal.target_pose.pose.position.x=self.mygoal[0][0]
    self.goal.action_goal.goal.target_pose.pose.position.y=self.mygoal[0][1]
    self.goal.action_goal.goal.target_pose.pose.position.z=self.mygoal[0][2]
    self.goal.action_goal.goal.target_pose.pose.orientation.x=self.mygoal[0][3]
    self.goal.action_goal.goal.target_pose.pose.orientation.y=self.mygoal[0][4]
    self.goal.action_goal.goal.target_pose.pose.orientation.z=self.mygoal[0][5]
    self.goal.action_goal.goal.target_pose.pose.orientation.w=self.mygoal[0][6]


    for next_goal in filter(lambda x:x!=None, self.mygoal):
      self.goal.action_goal.goal.target_pose.header.frame_id='/map'
      self.goal.action_goal.goal.target_pose.header.stamp=rospy.Time.now()
      self.goal.action_goal.goal.target_pose.pose.position.x=next_goal[0]
      self.goal.action_goal.goal.target_pose.pose.position.y=next_goal[1]
      self.goal.action_goal.goal.target_pose.pose.position.z=next_goal[2]
      self.goal.action_goal.goal.target_pose.pose.orientation.x=next_goal[3]
      self.goal.action_goal.goal.target_pose.pose.orientation.y=next_goal[4]
      self.goal.action_goal.goal.target_pose.pose.orientation.z=next_goal[5]
      self.goal.action_goal.goal.target_pose.pose.orientation.w=next_goal[6]

      self.client.send_goal_and_wait(self.goal.action_goal.goal) 
      state=self.client.get_state()
      while state != actionlib.GoalStatus.SUCCEEDED and not rospy.is_shutdown():
        self.client.send_goal_and_wait(self.goal.action_goal.goal) 
        state=self.client.get_state()

      rospy.loginfo('the state is: %s'%state)

if __name__=='__main__':
  s=sendGoal()
