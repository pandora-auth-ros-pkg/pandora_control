/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author:  Evangelos Apostolidis
* Author:  Chris Zalidis
*********************************************************************/

#include <pandora_end_effector_planner/sensor_orientation_planner.h>

namespace pandora_control
{
  SensorOrientationActionServer::SensorOrientationActionServer(
    std::string actionName,
    ros::NodeHandle nodeHandle)
  :
    actionServer_(
      nodeHandle,
      actionName,
      boost::bind(&SensorOrientationActionServer::callback, this, _1), false),
    actionName_(actionName),
    nodeHandle_(nodeHandle)
  {
    // get params from param server
    if (getPlannerParams())
    {
      if (timeStep_ <= 0) {
        ROS_WARN_STREAM("[" << actionName_ << "] Wrong time step value: "
          << timeStep_ << ", updating as fast as possible!");
        timeStep_ = 0.01;
      }

      kinect_pitch_publisher =
        nodeHandle_.advertise<std_msgs::Float64>(
          pitchCommandTopic_,
          5, true);

      kinect_yaw_publisher =
        nodeHandle_.advertise<std_msgs::Float64>(
          yawCommandTopic_,
          5, true);

      std_msgs::Float64 targetPosition;
      targetPosition.data = 0;
      kinect_pitch_publisher.publish(targetPosition);
      kinect_yaw_publisher.publish(targetPosition);
      position_ = CENTER;

      actionServer_.start();
    }
  }

  SensorOrientationActionServer::~SensorOrientationActionServer(void)
  {
  }

  void SensorOrientationActionServer::callback(
      const pandora_end_effector_planner::MoveSensorGoalConstPtr& goal)
  {
    command_ = goal->command;

    if (command_ == pandora_end_effector_planner::MoveSensorGoal::CENTER )
    {
      if (position_ != CENTER)
      {
        std_msgs::Float64 pitchTargetPosition, yawTargetPosition;
        pitchTargetPosition.data = 0;
        yawTargetPosition.data = 0;
        kinect_pitch_publisher.publish(pitchTargetPosition);
        kinect_yaw_publisher.publish(yawTargetPosition);
        position_ = CENTER;
      }
      ROS_DEBUG("%s: Succeeded", actionName_.c_str());
      // set the action state to succeeded
      actionServer_.setSucceeded();
      return;
    }
    else if (command_ == pandora_end_effector_planner::MoveSensorGoal::MOVE)
    {
      ros::Rate rate(1/timeStep_);

      std_msgs::Float64 pitchTargetPosition, yawTargetPosition;

      while (ros::ok())
      {
        if (actionServer_.isPreemptRequested() || !ros::ok())
        {
          ROS_DEBUG("%s: Preempted", actionName_.c_str());
          // set the action state to preempted
          actionServer_.setPreempted();
          return;
        }

        switch (position_)
        {
          case CENTER:
            pitchTargetPosition.data = 0;
            yawTargetPosition.data = maxYaw_;
            position_ = HIGH_LEFT;
            break;
          case HIGH_LEFT:
            pitchTargetPosition.data = maxPitch_;
            yawTargetPosition.data = maxYaw_;
            position_ = LOW_LEFT;
            break;
          case LOW_LEFT:
            pitchTargetPosition.data = maxPitch_;
            yawTargetPosition.data = 0;
            position_ = LOW_CENTER;
            break;
          case LOW_CENTER:
            pitchTargetPosition.data = maxPitch_;
            yawTargetPosition.data = -maxYaw_;
            position_ = LOW_RIGHT;
            break;
          case LOW_RIGHT:
            pitchTargetPosition.data = 0;
            yawTargetPosition.data = -maxYaw_;
            position_ = HIGH_RIGHT;
            break;
          case HIGH_RIGHT:
            pitchTargetPosition.data = 0;
            yawTargetPosition.data = 0;
            position_ = CENTER;
            break;
        }
        kinect_pitch_publisher.publish(pitchTargetPosition);
        kinect_yaw_publisher.publish(yawTargetPosition);
        rate.sleep();
      }
    }
    else if (command_ == pandora_end_effector_planner::MoveSensorGoal::POINT)
    {
    }
    else if (command_ == pandora_end_effector_planner::MoveSensorGoal::STOP)
    {
      ROS_DEBUG("%s: Succeeded", actionName_.c_str());
      // set the action state to succeeded
      actionServer_.setSucceeded();
      return;
    }
    else
    {
      ROS_DEBUG("%s: Aborted, there is no such command", actionName_.c_str());
      // set the action state to aborted
      actionServer_.setAborted();
      return;
    }
  }

  bool SensorOrientationActionServer::getPlannerParams()
  {
    nodeHandle_.param(actionName_ + "/max_pitch", maxPitch_, 0.4);
    nodeHandle_.param(actionName_ + "/max_yaw", maxYaw_, 0.7);
    nodeHandle_.param(actionName_ + "/time_step", timeStep_, 1.0);

    if (nodeHandle_.getParam(actionName_ + "/pitch_command_topic",
      pitchCommandTopic_))
    {
      ROS_INFO_STREAM("Got param pitch_command_topic: " << pitchCommandTopic_);
    }
    else
    {
      ROS_FATAL("Failed to get param pitch_command_topic shuting down");
      return false;
    }

    if (nodeHandle_.getParam(actionName_ + "/yaw_command_topic",
      yawCommandTopic_))
    {
      ROS_INFO_STREAM("Got param yaw_command_topic: " << yawCommandTopic_);
    }
    else
    {
      ROS_FATAL("Failed to get param yaw_command_topic shuting down");
      return false;
    }
    return true;
  }
}  // namespace pandora_control

int main(int argc, char **argv)
{
  if (argc != 4)
  {
    ROS_FATAL_STREAM("No arguement passed. Action name is required");
    return 1;
  }

  ros::init(argc, argv, argv[1]);
  ros::NodeHandle nodeHandle;
  std::string actionName = argv[1];

  pandora_control::SensorOrientationActionServer
    sensorOrientationActionServer(
      actionName,
      nodeHandle);

  ros::spin();
}
