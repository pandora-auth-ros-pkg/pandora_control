#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sys, select, termios, tty
import thread
from numpy import clip

control_keys = {
  'up' : '\x41',
  'down' : '\x42',
  'right' : '\x43',
  'left' : '\x44',
  'space' : '\x20'}
control_bindings = {
  '\x41' : ( 0.1 , 0.0),
  '\x42' : (-0.1 , 0.0),
  '\x43' : ( 0.0 ,-0.1),
  '\x44' : ( 0.0 , 0.1),
  '\x20' : ( 0.0 , 0,0)}

mode_keys = {
  'k' : '\6B',  # kinect - xtion pan n' tilt mode key
  'l' : '\6C',  # linear actuator mode key
  'm' : '\6D',  # motors mode key
  'p' : '\70'}  # pi-cam pan n' tilt mode key
modes = {
  'k' : 'xtion',
  'l' : 'lac',
  'm' : 'motors',
  'p' : 'picam'}


class Keyop:

  def __init__(self):
    self.lin_vel = 0       # motors linear velocity
    self.ang_vel = 0       # motors angular velocity
    self.lac_position = 0  # linear actuator vertical position
    self.xtion_yaw = 0     # xtion yaw
    self.xtion_pitch = 0   # xtion pitch
    self.picam_yaw = 0     # picam yaw
    self.picam_pitch = 0   # picam pitch

    self.motors_msg = Twist()         # motors velocity msg
    self.lac_msg = Float64()          # linear actuator msg
    self.xtion_yaw_msg = Float64()    # xtion yaw msg
    self.xtion_pitch_msg = Float64()  # xtion pitch msg
    self.picam_yaw_msg = Float64()    # picam yaw msg
    self.picam_pitch_msg = Float64()  # picam pitch msg

    self.motors_pub = rospy.Publisher('cmd_vel', Twist)
    self.lac_pub = rospy.Publisher('linear_motor_controller/command', Float64)
    self.xtion_yaw_pub = rospy.Publisher('kinect_yaw_controller/command', Float64)
    self.xtion_pitch_pub = rospy.Publisher('kinect_pitch_controller/command', Float64)
    self.picam_yaw_pub = rospy.Publisher('camera_effector/pan_command', Float64)
    self.picam_pitch_pub = rospy.Publisher('camera_effector/tilt_command', Float64)

    self.mode = 'motors'  # initialize mode to motors mode

    self.lock = thread.allocate_lock()
    thread.start_new_thread(self.publish_commands, ())
    self.key_loop()


  def publish_commands(self):
    rate = rospy.Rate(5)
    while 1:
      self.lock.acquire()
      if self.mode == 'motors':
        self.motors_msg.linear.x = self.lin_vel
        self.motors_msg.angular.z = self.ang_vel
        self.motors_pub.publish(self.motors_msg)
      elif self.mode == 'lac':
        self.lac_msg.data = self.lac_position
        self.lac_pub.publish(self.lac_msg)
      elif self.mode == 'xtion':
        self.xtion_yaw_msg.data = self.xtion_yaw
        self.xtion_pitch_msg.data = self.xtion_pitch
        self.xtion_yaw_pub.publish(self.xtion_yaw_msg)
        self.xtion_pitch_pub.publish(self.xtion_pitch_msg)
      elif self.mode == 'picam':
        self.picam_yaw_msg.data = self.picam_yaw
        self.picam_pitch_msg.data = self.picam_pitch
        self.picam_yaw_pub.publish(self.picam_yaw_pub)
        self.picam_pitch_pub.publish(self.picam_pitch_pub)
      elif self.mode == 'quit':
        self.lock.release()
        break
      self.lock.release()
      rate.sleep()

  def print_state(self, erase):
    if erase:
      rospy.loginfo("\x1b[3A")

    rospy.loginfo("\033[32;1mTeleop Mode: %s\033[0m", self.mode)

    rospy.loginfo("\x1b[1M\x1b[1A")

    if self.mode == 'motors':
      rospy.loginfo("\033[33;1mMotors: linear_speed: %s - angular_speed: %s\033[0m", self.lin_vel, self.ang_vel)
    elif self.mode == 'lac':
      rospy.loginfo("\033[33;1mLinear Actuator: Position: %s\033[0m", self.lac_position)
    elif self.mode == 'xtion':
      rospy.loginfo("\033[33;1mXtion: pitch: %s - yaw: %s\033[0m", self.xtion_pitch, self.xtion_yaw)
    elif self.mode == 'picam':
      rospy.loginfo("\033[33;1mPicam: pitch: %s - yaw: %s\033[0m", self.picam_pitch, self.picam_yaw)

  def get_key(self):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    return key


  def key_loop(self):
    rospy.loginfo("Use the arrow keys to adjust speed or position and space to halt/reset")
    rospy.loginfo("Press 'ctr-c' or 'q' to exit")
    self.print_state(0)
    self.settings = termios.tcgetattr(sys.stdin)

    while 1:
      key = self.get_key()
      if key in control_bindings.keys():
        if self.mode == 'motors':
          if key == control_keys['space']:
            self.lin_vel = 0.0
            self.ang_vel = 0.0
          else:
            self.lin_vel = self.lin_vel + control_bindings[key][0] * 2
            self.ang_vel = self.ang_vel + control_bindings[key][1] * 2
          self.lin_vel = clip(self.lin_vel, -1.0, 1.0)
          self.ang_vel = clip(self.ang_vel, -1.0, 1.0)
        elif self.mode == 'lac':
          if key == control_keys['space']:
            self.lac_position = 0
          else:
            self.lac_position = self.lac_position + control_bindings[key][0] * 10
          self.lac_position = clip(self.lac_position, 0, 14)
        elif self.mode == 'xtion':
          if key == control_keys['space']:
            self.xtion_pitch = 0
            self.xtion_yaw = 0
          else:
            self.xtion_pitch = self.xtion_pitch + control_bindings[key][0] / 2
            self.xtion_yaw = self.xtion_yaw + control_bindings[key][1]
          self.xtion_pitch = clip(self.xtion_pitch, -0.45, 0.75)
          self.xtion_yaw = clip(self.xtion_yaw, -0.7, 0.7)
        elif self.mode == "picam":
          if key == control_keys['space']:
            self.picam_pitch = 0
            self.picam_yaw = 0
          else:
            self.picam_pitch = self.picam_pitch + control_bindings[key][0]
            self.picam_yaw = self.picam_yaw + control_bindings[key][1]
          self.picam_yaw = clip(self.picam_yaw, -0.7, 0.7)
          self.picam_pitch = clip(self.picam_pitch, -0.45, 0.75)

        self.print_state(1)

      elif key in mode_keys:
        self.lock.acquire()
        self.lin_vel = 0; self.ang_vel = 0
        self.motors_msg.linear.x = 0; self.motors_msg.angular.z = 0
        self.motors_pub.publish(self.motors_msg)
        self.mode = modes[key]
        self.lock.release()
        self.print_state(1)
        continue
      elif key == '\x03' or key == '\x71':  # ctr-c or q
        break
      else:
        continue

    self.finalize()

  def finalize(self):
    rospy.loginfo('Halting motors and exiting...')
    self.lock.acquire()
    self.settings = termios.tcgetattr(sys.stdin)
    self.mode = 'quit'
    self.motors_msg.linear.x = 0; self.motors_msg.angular.z = 0
    self.motors_pub.publish(self.motors_msg)
    self.lock.release()
    sys.exit()

if __name__ == "__main__":
  rospy.init_node('keyop_node')
  rospy.loginfo("Keyboard Teleoperation Node Initialized")
  keyop = Keyop()
