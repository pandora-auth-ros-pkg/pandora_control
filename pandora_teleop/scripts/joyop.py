#!/usr/bin/env python

'''
File Description  Joystick Teleoperation Script, that controls the robot motors
                  velocity, linear actuator position, xtion and picam pan n' tilts

Script Usage      Control the motors velocity with the right_stick
                  Control the linear actuator position with button_1 +  left_stick
                  Control the xtion pan n' tilt with button_2 + left_stick
                  Control the picam pan n' tilt with button_3 + left_stick

Author            George Kouros
'''

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from numpy import clip
from subprocess import call
from sys import argv

class Joyop:

  def __init__(self, motors_lin_vel_scale=0.5, motors_ang_vel_scale=0.8):
    self.motors_lin_vel_scale = motors_lin_vel_scale
    self.motors_ang_vel_scale = motors_ang_vel_scale
    self.lac_scale = 14.0
    self.xtion_yaw_range = [-0.7, 0.7]
    self.xtion_pitch_range = [-0.45, -0.75]
    self.picam_yaw_range = [-0.7, 0.7]
    self.picam_pitch_range = [-0.45, -0.75]

    self.motors_lin_vel = 0
    self.motors_ang_vel = 0
    self.angular_ang_vel = 0
    self.lac_position = 0
    self.xtion_yaw = 0
    self.xtion_pitch = 0
    self.picam_yaw = 0
    self.picam_pitch = 0

    self.motors_vel_pub = rospy.Publisher('/cmd_vel', Twist)
    self.lac_position_pub = rospy.Publisher('/linear_elevator_controller/command', Float64)
    self.xtion_yaw_pub = rospy.Publisher('/kinect_yaw_controller/command', Float64)
    self.xtion_pitch_pub = rospy.Publisher('/kinect_pitch_controller/command', Float64)
    self.picam_yaw_pub = rospy.Publisher('/camera_effector/pan_command', Float64)
    self.picam_pitch_pub = rospy.Publisher('/camera_effector/tilt_command', Float64)
    joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

    rospy.Timer(rospy.Duration(0.1), self.launch_joy_node, oneshot=True)
    rospy.sleep(rospy.Duration(3))
    rospy.Timer(rospy.Duration(0.1), self.pub_callback, oneshot=False)
    rospy.loginfo("\n\n\n\n")
    rospy.spin()

  def launch_joy_node(self, event):
    call(["rosrun", "joy", "joy_node"])  # launch joystick node

  def print_state(self):
    rospy.loginfo("\x1b[1M\x1b[5A\x1b[5M")  # erase previous prints
    rospy.loginfo("\033[33;1m[motors] lin_vel: %s - ang_vel: %s\033[0m", self.motors_lin_vel, self.motors_ang_vel)
    rospy.loginfo("\033[33;1m[linear_actuator] position: %s\033[0m", self.lac_position)
    rospy.loginfo("\033[33;1m[xtion] yaw: %s - pitch: %s\033[0m", self.xtion_yaw, self.xtion_pitch)
    rospy.loginfo("\033[33;1m[picam] yaw: %s - pitch: %s\033[0m", self.picam_yaw, self.picam_pitch)

  def joy_callback(self, joy_msg):
    self.motors_lin_vel = joy_msg.axes[2] * self.motors_lin_vel_scale
    self.motors_ang_vel = joy_msg.axes[3] * self.motors_ang_vel_scale

    if joy_msg.buttons[0] == 1:
      self.lac_position = self.lac_scale * joy_msg.axes[1] * (joy_msg.axes[1] > 0)
    if joy_msg.buttons[1] == 1:
      self.xtion_yaw = clip(joy_msg.axes[0], self.xtion_yaw_range[0], self.xtion_yaw_range[1])
      self.xtion_pitch = clip(joy_msg.axes[1], self.xtion_pitch_range[0], self.xtion_pitch_range[1])
    if joy_msg.buttons[2] == 1:
      self.picam_yaw = clip(joy_msg.axes[0], self.picam_yaw_range[0], self.picam_yaw_range[1])
      self.picam_pitch = clip(joy_msg.axes[1], self.picam_pitch_range[0], self.picam_pitch_range[1])


  def pub_callback(self, event):
    motors_vel_msg = Twist()
    lac_position_msg = Float64()
    xtion_yaw_msg = Float64()
    xtion_pitch_msg = Float64()
    picam_yaw_msg = Float64()
    picam_pitch_msg = Float64()

    motors_vel_msg.linear.x = self.motors_lin_vel
    motors_vel_msg.angular.z = self.motors_ang_vel
    lac_position_msg.data = self.lac_position
    xtion_yaw_msg.data = self.xtion_yaw
    xtion_pitch_msg.data = self.xtion_pitch
    picam_yaw_msg.data = self.picam_yaw
    picam_pitch_msg.data = self.picam_pitch

    self.motors_vel_pub.publish(motors_vel_msg)
    self.lac_position_pub.publish(lac_position_msg)
    self.xtion_yaw_pub.publish(xtion_yaw_msg)
    self.xtion_pitch_pub.publish(xtion_pitch_msg)
    self.picam_yaw_pub.publish(picam_yaw_msg)
    self.picam_pitch_pub.publish(picam_pitch_msg)

    self.print_state()


if __name__ == "__main__":
  rospy.init_node('joyop_node')
  rospy.loginfo("Joyop Teleoperation Node Initialized")
  rospy.loginfo("RS[motors], 1+LS[lac], 2+LS[xtion], 3+LS[picam]")

  args = argv[1:]
  if len(args) == 1:
    rospy.loginfo("Setting max linear velocity to %s and angular velocity to %s", args[0], args[1])
    joyop = Joyop(args[0], args[1])
  else:
    rospy.loginfo("Using default max linear(0.5m/s) and angular velocity(0.8m/s)")
    joyop = Joyop()

