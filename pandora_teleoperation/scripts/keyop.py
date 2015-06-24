#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import thread
from time import sleep
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
    self.msg = Twist()
    self.pub = rospy.Publisher('cmd_vel', Twist)
    self.lock = thread.allocate_lock()
    thread.start_new_thread(self.publish_twist, ())
    self.lin_vel = 0
    self.ang_vel = 0
    self.key_loop()

  def get_key(self):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    return key

  def publish_twist(self):
    rate = rospy.Rate(5)
    while 1:
      self.lock.acquire()
      self.pub.publish(self.msg)
      self.lock.release()
      rate.sleep()

  def key_loop(self):
    rospy.loginfo("Keyboard Teleoperation Node Initialized")
    rospy.loginfo("Use the arrow keys to adjust linear and angular speed or space to brake")
    rospy.loginfo("Press 'ctr-c' or 'q' to exit")
    rospy.loginfo(
        "\033[33;1mLinear Speed: %s - Angular Speed: %s\033[0m\x1b[1A\x1b[1M",
        self.lin_vel, self.ang_vel)

    self.settings = termios.tcgetattr(sys.stdin)

    try:
      while 1:
        try:
          key = self.get_key()
        except KeyboardInterrupt:
          raise

        if key in control_bindings.keys():
          # rospy.loginfo("key: %s", hex(ord(key)))
          if key == keyboard_keys['space']:
            self.lin_vel = 0.0
            self.ang_vel = 0.0
          else:
            self.lin_vel = self.lin_vel + control_bindings[key][0]
            self.ang_vel = self.ang_vel + control_bindings[key][1]
        # elif key == '\x03' or key == '\x71':
          # break
        else:
          continue

        self.lin_vel = clip(self.lin_vel, -1.0, 1.0)
        self.ang_vel = clip(self.ang_vel, -1.0, 1.0)

        rospy.loginfo(
          "\033[33;1mLinear Speed: %s - Angular Speed: %s \033[0m \x1b[1A\x1b[1M", self.lin_vel, self.ang_vel)
        self.msg.linear.x = self.lin_vel; self.msg.linear.y = 0; self.msg.linear.z = 0
        self.msg.angular.x = 0; self.msg.angular.y = 0; self.msg.angular.z = self.ang_vel

    except KeyboardInterrupt:
      rospy.loginfo("Keyboard Interrupt caught")

    finally:
      rospy.loginfo("Halting motors and Exiting...")
      self.msg.linear.x = 0; self.msg.linear.y = 0; self.msg.linear.z = 0
      self.msg.angular.x = 0; self.msg.angular.y = 0; self.msg.angular.z = 0
      self.pub.publish(self.msg)

    # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == "__main__":
  rospy.init_node('keyop_node')
  keyop = Keyop()
