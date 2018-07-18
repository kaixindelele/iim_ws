#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction

import sys
sys.path.append('/home/iim/iim_ws/src/iim_robot')
from robot_arm.nodes import press
from robot_vision.srv import *

from tf import TransformListener

class sendGoal(object):
  def __init__(self):
    self.client=actionlib.SimpleActionClient('move_base', MoveBaseAction)

    self.mygoal=[]
    self.points_num=rospy.get_param('~points_num',0)
    rospy.loginfo('points num: %d'%self.points_num)

    for i in range(self.points_num):
      self.mygoal.append(rospy.get_param('~goal_points/point_%d'%i, None))

    self.base_link=rospy.get_param('~base_link','base_link')
    self.target_link=rospy.get_param('~target_link','arm_shoulder_lift_servo_link')
    self.measure_link=rospy.get_param('~measure_link','base_link')
    feed=rospy.get_param('~feed',0)

    rospy.wait_for_service('get_point_coordinate')
    self.arm_press=press.armPress(self.base_link,self.measure_link,self.target_link)

#    if self.mygoal:
#      self.robotGo(self.mygoal[0])
#
    target=getArmTargetCoordinate('up_key')
#    target=[0.36, -0.06, 1.12]
    if target:
      self.arm_press.pressOnce(target,feed)

  def getArmTargetCoordinate(self,target):
    try:
      get_point_coordinate=rospy.ServiceProxy('get_point_coordinate',vision)
      res=get_point_coordinate(target)
      rospy.loginfo('x: %f, y: %f, z: %f'%(res.x/1000,res.y/1000,res.z/1000))

    except rospy.ServiceException as e:
      res=None
      rospy.loginfo('call service failed: %s'%e)

    listener = TransformListener()
    listener.waitForTransform(measure_link, 'camera_link', rospy.Time(0), rospy.Duration(10))
    camera_in_base,_=listener.lookupTransform(measure_link,'camera_link', rospy.Time(0))
    if res:
      return (res.x+camera_in_base[0],res.y+camera_in_base[1],res.z+camera_in_base[2])
    else:
      return None

  def robotGo(self,target_goal):
    self.client.wait_for_server()
    goal=MoveBaseAction()

    goal.action_goal.goal.target_pose.header.frame_id='/map'
    goal.action_goal.goal.target_pose.header.stamp=rospy.Time.now()
    goal.action_goal.goal.target_pose.pose.position.x=target_goal[0]
    goal.action_goal.goal.target_pose.pose.position.y=target_goal[1]
    goal.action_goal.goal.target_pose.pose.position.z=target_goal[2]
    goal.action_goal.goal.target_pose.pose.orientation.x=target_goal[3]
    goal.action_goal.goal.target_pose.pose.orientation.y=target_goal[4]
    goal.action_goal.goal.target_pose.pose.orientation.z=target_goal[5]
    goal.action_goal.goal.target_pose.pose.orientation.w=target_goal[6]

    self.client.send_goal_and_wait(goal.action_goal.goal) 
    state=self.client.get_state()
    while state != actionlib.GoalStatus.SUCCEEDED and not rospy.is_shutdown():
      self.client.send_goal_and_wait(goal.action_goal.goal) 
      state=self.client.get_state()

    rospy.loginfo('the state is: %s'%state)

if __name__=='__main__':
  rospy.init_node('go_and_press')
  s=sendGoal()
