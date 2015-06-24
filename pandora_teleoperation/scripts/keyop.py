#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from numpy import clip

keyboard_keys = {
  'up' : '\x41',
  'down' : '\x42',
  'right' : '\x43',
  'left' : '\x44',
  'space' : '\x20'}


control_bindings = {
  '\x41' : ( 0.2 , 0.0),
  '\x42' : (-0.2 , 0.0),
  '\x43' : ( 0.0 ,-0.2),
  '\x44' : ( 0.0 , 0.2),
  '\x20' : ( 0.0 , 0.0)}

class Keyop:

  def __init__(self):
    self.settings = termios.tcgetattr(sys.stdin)
    self.pub = rospy.Publisher('cmd_vel', Twist)
    self.lin_vel = 0
    self.ang_vel = 0
    rospy.loginfo("Keyboard Teleoperation Node Initialized")
    rospy.loginfo("Use the arrow keys to adjust linear and angular speed or space to brake")
    rospy.loginfo("Press 'ctr-c' or 'q' to exit")
    rospy.loginfo(
      "\033[33;1mLinear Speed: %s - Angular Speed: %s \033[0m \x1b[1A\x1b[1M", self.lin_vel, self.ang_vel)

  def get_key(self):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    return key

  def key_loop(self):
    try:
      while 1:
        key = self.get_key()
        if key in control_bindings.keys():
          # rospy.loginfo("key: %s", hex(ord(key)))
          if key == keyboard_keys['space']:
            self.lin_vel = 0.0
            self.ang_vel = 0.0
          else:
            self.lin_vel = self.lin_vel + control_bindings[key][0]
            self.ang_vel = self.ang_vel + control_bindings[key][1]
        elif key == '\x03' or key == '\x71':
          rospy.loginfo("Exiting...")
          break
        else:
          continue

        self.lin_vel = clip(self.lin_vel, -1.0, 1.0)
        self.ang_vel = clip(self.ang_vel, -1.0, 1.0)

        rospy.loginfo(
          "\033[33;1mLinear Speed: %s - Angular Speed: %s \033[0m \x1b[1A\x1b[1M", self.lin_vel, self.ang_vel)
        msg = Twist()
        msg.linear.x = self.lin_vel; msg.linear.y = 0; msg.linear.z = 0
        msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = self.ang_vel
        self.pub.publish(msg)

    except:
      print e

    finally:
      msg = Twist()
      msg.linear.x = 0; msg.linear.y = 0; msg.linear.z = 0
      msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = 0
      self.pub.publish(msg)

    # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == "__main__":
  rospy.init_node('keyop_node')
  keyop = Keyop()
  keyop.key_loop()
