#! /usr/bin/env python

import roslib
import rospy
import sys
import actionlib
import pandora_control_communications.msg

if __name__ == '__main__':
  rospy.init_node('move_kinect_client_py')
  client = actionlib.SimpleActionClient('move_kinect_action', pandora_control_communications.msg.MoveKinectAction)
  client.wait_for_server()
  goal = pandora_control_communications.msg.MoveKinectGoal()
  goal.command = int(sys.argv[1])
  client.send_goal(goal)
