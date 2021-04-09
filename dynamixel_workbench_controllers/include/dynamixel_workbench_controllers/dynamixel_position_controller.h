/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */
/* Modified by: Douglas Smith */

#ifndef DYNAMIXEL_POSITION_CONTROL_H
#define DYNAMIXEL_POSITION_CONTROL_H

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStatePosList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <mutex>
#include <std_msgs/Float64.h>

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

// #define DEBUG

typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

using StateMsg = dynamixel_workbench_msgs::DynamixelStatePos;
using StateListMsg = dynamixel_workbench_msgs::DynamixelStatePosList;
using JointTractoryActionServer = actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>;

class DynamixelPositionController
{

private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher dynamixel_state_list_pub_;
  ros::Publisher joint_states_pub_;

  // ROS Topic Subscriber
  ros::Subscriber trajectory_sub_;
  ros::Subscriber move_sub_; //For testing

  // ROS Service Server
  ros::ServiceServer dynamixel_command_server_;

  // ROS Service Client

  // ROS Action Server
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;

  std::map<std::string, uint32_t> dynamixel_;
  std::map<std::string, const ControlItem *> control_items_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
  StateListMsg dynamixel_state_list_;

  double position_tol_;

  trajectory_msgs::JointTrajectory::Ptr jnt_tra_msg_;
  trajectory_msgs::JointTrajectory::ConstPtr last_jnt_tra_msg_;

  double read_period_;
  double write_period_;
  double pub_period_;

  bool is_moving_;
  bool init_done_;

  std::mutex state_mtx_;
  std::mutex traj_mtx_;

public:
  DynamixelPositionController();
  ~DynamixelPositionController();

  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool getDynamixelsInfo(const std::string yaml_file);
  bool loadDynamixels(void);
  bool initDynamixels(void);
  bool initControlItems(void);
  bool initSDKHandlers(void);
  bool getPresentPosition(std::vector<std::string> dxl_name);

  double getReadPeriod() { return read_period_; }
  double getWritePeriod() { return write_period_; }
  double getPublishPeriod() { return pub_period_; }

  void initPublisher(void);
  void initSubscriber(void);

  void initServer();

  void readCallback(const ros::TimerEvent &);
  void writeCallback(const ros::TimerEvent &);
  void publishCallback(const ros::TimerEvent &);

  void trajectoryMsgCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
  bool dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                   dynamixel_workbench_msgs::DynamixelCommand::Response &res);

  void moveCallback(const std_msgs::Float64::ConstPtr &msg);

  bool dynsAtSetPositions();
  bool dynsAtSetPositions(const std::vector<StateMsg *> states);
  //void updateSetPositions(const std::vector<uint8_t> &id_array, const std::vector<int32_t> &dynamixel_position);

  bool writeSetVals(StateMsg *state, double pos, double vel, int remaining_pos);

  StateMsg *getJointState(const std::string &name);

  JointTractoryActionServer::GoalConstPtr goal_;
  void goalCB();
  void cancelCB();
};

#endif //DYNAMIXEL_POSITION_CONTROL_H