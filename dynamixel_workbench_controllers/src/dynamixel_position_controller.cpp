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

#include "dynamixel_workbench_controllers/dynamixel_position_controller.h"
#include <std_msgs/Float64.h>

double toRad = M_PI / 180.0;
double toDeg = 180.0 / M_PI;

DynamixelPositionController::DynamixelPositionController()
    : node_handle_(""),
      priv_node_handle_("~"),
      is_moving_(false)
{

  read_period_ = priv_node_handle_.param<double>("dxl_read_period", 0.010f);
  write_period_ = priv_node_handle_.param<double>("dxl_write_period", 0.010f);
  pub_period_ = priv_node_handle_.param<double>("publish_period", 0.010f);

  position_tol_ = 0.1 * toRad;

  dxl_wb_ = new DynamixelWorkbench;

  //jnt_tra_msg_.reset(new trajectory_msgs::JointTrajectory());
}

DynamixelPositionController::~DynamixelPositionController() {}

bool DynamixelPositionController::initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
  bool result = false;
  const char *log;

  result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  return result;
}

bool DynamixelPositionController::getDynamixelsInfo(const std::string yaml_file)
{
  YAML::Node dynamixel;
  dynamixel = YAML::LoadFile(yaml_file.c_str());

  if (dynamixel == NULL)
    return false;

  for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++)
  {
    std::string name = it_file->first.as<std::string>();
    if (name.size() == 0)
    {
      continue;
    }

    YAML::Node item = dynamixel[name];
    for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
    {
      std::string item_name = it_item->first.as<std::string>();
      int32_t value = it_item->second.as<int32_t>();

      if (item_name == "ID")
        dynamixel_[name] = value;

      ItemValue item_value = {item_name, value};
      std::pair<std::string, ItemValue> info(name, item_value);

      dynamixel_info_.push_back(info);
    }
  }

  return true;
}

bool DynamixelPositionController::loadDynamixels(void)
{
  bool result = false;
  const char *log;

  for (auto const &dxl : dynamixel_)
  {
    uint16_t model_number = 0;
    result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    }
    else
    {
      dynamixel_workbench_msgs::DynamixelState state;
      state.name = dxl.first;
      state.id = dxl.second;
      dynamixel_state_list_.dynamixel_state.push_back(state);

      ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
    }
  }
  return result;
}

bool DynamixelPositionController::initDynamixels(void)
{
  const char *log;

  for (auto const &dxl : dynamixel_)
  {
    dxl_wb_->torqueOff((uint8_t)dxl.second);

    for (auto const &info : dynamixel_info_)
    {
      if (dxl.first == info.first)
      {
        if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
        {
          bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
          if (result == false)
          {
            ROS_ERROR("%s", log);
            ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
            return false;
          }
        }
      }
    }

    dxl_wb_->torqueOn((uint8_t)dxl.second);
  }

  return true;
}

bool DynamixelPositionController::initControlItems(void)
{
  bool result = false;
  const char *log = NULL;

  auto it = dynamixel_.begin();

  const ControlItem *goal_position = dxl_wb_->getItemInfo(it->second, "Goal_Position");
  if (goal_position == NULL)
    return false;

  const ControlItem *goal_velocity = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
  if (goal_velocity == NULL)
    goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
  if (goal_velocity == NULL)
    return false;

  const ControlItem *present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
  if (present_position == NULL)
    return false;

  const ControlItem *present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
  if (present_velocity == NULL)
    present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
  if (present_velocity == NULL)
    return false;

  const ControlItem *present_current = dxl_wb_->getItemInfo(it->second, "Present_Current");
  if (present_current == NULL)
    present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
  if (present_current == NULL)
    return false;

  control_items_["Goal_Position"] = goal_position;
  control_items_["Goal_Velocity"] = goal_velocity;

  control_items_["Present_Position"] = present_position;
  control_items_["Present_Velocity"] = present_velocity;
  control_items_["Present_Current"] = present_current;

  return true;
}

bool DynamixelPositionController::initSDKHandlers(void)
{
  bool result = false;
  const char *log = NULL;

  auto it = dynamixel_.begin();

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

    /* 
      As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
    */
    // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
    uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length + 2;

    result = dxl_wb_->addSyncReadHandler(start_address,
                                         read_length,
                                         &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }
  }

  return result;
}

void DynamixelPositionController::initPublisher()
{
  dynamixel_state_list_pub_ = priv_node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 100);
  joint_states_pub_ = priv_node_handle_.advertise<sensor_msgs::JointState>("joint_states", 100);
}

void DynamixelPositionController::initSubscriber()
{
  trajectory_sub_ = priv_node_handle_.subscribe("joint_trajectory", 100, &DynamixelPositionController::trajectoryMsgCallback, this);
  move_sub_ = priv_node_handle_.subscribe("move_me", 100, &DynamixelPositionController::moveCallback, this);
}

void DynamixelPositionController::moveCallback(const std_msgs::Float64::ConstPtr &msg)
{

  dxl_wb_->torqueOn(2);
  auto &states = dynamixel_state_list_.dynamixel_state;
  for (auto &state : states)
  {
    state.set_velocity = dxl_wb_->convertVelocity2Value(state.id, 0.2);
    state.set_position = dxl_wb_->convertRadian2Value(state.id, msg->data * toRad);
  }
}

void DynamixelPositionController::initServer()
{
  dynamixel_command_server_ = priv_node_handle_.advertiseService("dynamixel_command", &DynamixelPositionController::dynamixelCommandMsgCallback, this);
}

void DynamixelPositionController::readCallback(const ros::TimerEvent &)
{
  auto &states = dynamixel_state_list_.dynamixel_state;

  int32_t get_current[states.size()];
  int32_t get_velocity[states.size()];
  int32_t get_position[states.size()];

  uint8_t id_array[states.size()];
  uint8_t id_cnt = 0;
  bool result = false;

  const char *log = NULL;
  static bool init_done = false;

  for (auto &state : dynamixel_state_list_.dynamixel_state)
  {
    id_array[id_cnt++] = state.id;
  }

#ifndef DEBUG
  if (true)
  {
#endif
    if (dxl_wb_->getProtocolVersion() == 2.0f)
    {
      result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                 id_array,
                                 dynamixel_.size(),
                                 &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }

      result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                        id_array,
                                        id_cnt,
                                        control_items_["Present_Current"]->address,
                                        control_items_["Present_Current"]->data_length,
                                        get_current,
                                        &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }

      result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                        id_array,
                                        id_cnt,
                                        control_items_["Present_Velocity"]->address,
                                        control_items_["Present_Velocity"]->data_length,
                                        get_velocity,
                                        &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }

      result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                        id_array,
                                        id_cnt,
                                        control_items_["Present_Position"]->address,
                                        control_items_["Present_Position"]->data_length,
                                        get_position,
                                        &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }

      for (uint8_t index = 0; index < id_cnt; index++)
      {
        states[index].present_current = get_current[index];
        states[index].present_velocity = get_velocity[index];
        states[index].present_position = get_position[index];
      }
    }
    else if (dxl_wb_->getProtocolVersion() == 1.0f)
    {
      uint16_t length_of_data = control_items_["Present_Position"]->data_length +
                                control_items_["Present_Velocity"]->data_length +
                                control_items_["Present_Current"]->data_length;
      uint32_t get_all_data[length_of_data];
      uint8_t dxl_cnt = 0;
      for (auto const &dxl : dynamixel_)
      {
        result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                       control_items_["Present_Position"]->address,
                                       length_of_data,
                                       get_all_data,
                                       &log);
        if (result == false)
        {
          ROS_ERROR("%s", log);
        }

        states[dxl_cnt].present_current = DXL_MAKEWORD(get_all_data[4], get_all_data[5]);
        states[dxl_cnt].present_velocity = DXL_MAKEWORD(get_all_data[2], get_all_data[3]);
        states[dxl_cnt].present_position = DXL_MAKEWORD(get_all_data[0], get_all_data[1]);

        dxl_cnt++;
      }
    }

    //First run, fetch the goal position
    if (!init_done)
    {
      init_done = true;
      for (auto &state : states)
      {
        int32_t temp_data;
        result = dxl_wb_->itemRead(state.id, "Goal_Position", &temp_data, &log);
        if (result == false)
          ROS_ERROR("%s", log);
        else
          state.set_position = temp_data;

        result = dxl_wb_->itemRead(state.id, "Goal_Velocity", &temp_data, &log);
        if (result == false)
          ROS_ERROR("%s", log);
        else
          state.set_velocity = temp_data;
      }
    }
#ifndef DEBUG
  }
#endif
}

void DynamixelPositionController::publishCallback(const ros::TimerEvent &)
{
  dynamixel_state_list_pub_.publish(dynamixel_state_list_);
}

void DynamixelPositionController::writeCallback(const ros::TimerEvent &)
{
  auto &states = dynamixel_state_list_.dynamixel_state;

  bool result = false;
  const char *log = NULL;

  std::vector<uint8_t> id_array;

  std::vector<int32_t> dynamixel_position;
  std::vector<int32_t> dynamixel_velocity;

  static uint32_t point_cnt = 0;
  static uint32_t position_cnt = 0;

  if (!jnt_tra_msg_.joint_names.empty() && !jnt_tra_msg_.points.empty())
  {
    auto pt = jnt_tra_msg_.points.front();
    auto &state = this->getJointState(jnt_tra_msg_.joint_names.front());
    state.set_position = dxl_wb_->convertRadian2Value(state.id, pt.positions[0]);
    state.set_velocity = dxl_wb_->convertVelocity2Value(state.id, pt.velocities[0]);
  }

  if (!jnt_tra_msg_.points.empty() && this->dynsAtSetPositions())
  {
    traj_mtx_.lock();
    jnt_tra_msg_.points.erase(jnt_tra_msg_.points.begin());
    traj_mtx_.unlock();

    ROS_INFO_STREAM("Done with a move");

    if (!jnt_tra_msg_.joint_names.empty() && !jnt_tra_msg_.points.empty())
    {
      auto pt = jnt_tra_msg_.points.front();
      auto &state = this->getJointState(jnt_tra_msg_.joint_names.front());
      state.set_position = dxl_wb_->convertRadian2Value(state.id, pt.positions[0]);
      state.set_velocity = dxl_wb_->convertVelocity2Value(state.id, pt.velocities[0]);
    }
  }

  for (auto &state : states)
  {
    id_array.push_back(state.id);
    dynamixel_position.push_back(state.set_position);
    dynamixel_velocity.push_back(state.set_velocity);
  }

  result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, id_array.data(), id_array.size(), dynamixel_velocity.data(), 1, &log);
  if (result == false)
    ROS_ERROR("%s", log);

  result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, id_array.data(), id_array.size(), dynamixel_position.data(), 1, &log);
  if (result == false)
    ROS_ERROR("%s", log);
}

bool DynamixelPositionController::dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                                              dynamixel_workbench_msgs::DynamixelCommand::Response &res)
{
  bool result = false;
  const char *log;

  uint8_t id = req.id;
  std::string item_name = req.addr_name;
  int32_t value = req.value;

  result = dxl_wb_->itemWrite(id, item_name.c_str(), value, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[ID : %d]", value, item_name.c_str(), id);
  }

  res.comm_result = result;

  return true;
}

void DynamixelPositionController::trajectoryMsgCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
  this->traj_mtx_.lock();
  last_jnt_tra_msg_ = msg;
  jnt_tra_msg_ = *msg;
  this->traj_mtx_.unlock();
}

dynamixel_workbench_msgs::DynamixelState &DynamixelPositionController::getJointState(const std::string &name)
{
  for (auto &state : dynamixel_state_list_.dynamixel_state)
  {
    if (state.name.compare(name) == 0)
      return state;
  }
}

bool DynamixelPositionController::dynsAtSetPositions()
{
  for (auto &&state : dynamixel_state_list_.dynamixel_state)
  {
    auto tol = dxl_wb_->convertRadian2Value(state.id, position_tol_);
    if (abs(state.present_position - state.set_position) > tol)
      return false;
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_workbench_controllers");
  ros::NodeHandle node_handle("");

  std::string port_name = "/dev/ttyUSB0";
  uint32_t baud_rate = 57600;

  if (argc < 2)
  {
    ROS_ERROR("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
    return 0;
  }
  else
  {
    port_name = argv[1];
    baud_rate = atoi(argv[2]);
  }

  DynamixelPositionController dynamixel_controller;

  bool result = false;

  std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");

  result = dynamixel_controller.initWorkbench(port_name, baud_rate);
  if (result == false)
  {
    ROS_ERROR("Please check USB port name");
    return 0;
  }

  result = dynamixel_controller.getDynamixelsInfo(yaml_file);
  if (result == false)
  {
    ROS_ERROR("Please check YAML file");
    return 0;
  }

  result = dynamixel_controller.loadDynamixels();
  if (result == false)
  {
    ROS_ERROR("Please check Dynamixel ID or BaudRate");
    return 0;
  }

  result = dynamixel_controller.initDynamixels();
  if (result == false)
  {
    ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
    return 0;
  }

  result = dynamixel_controller.initControlItems();
  if (result == false)
  {
    ROS_ERROR("Please check control items");
    return 0;
  }

  result = dynamixel_controller.initSDKHandlers();
  if (result == false)
  {
    ROS_ERROR("Failed to set Dynamixel SDK Handler");
    return 0;
  }

  dynamixel_controller.initPublisher();
  dynamixel_controller.initSubscriber();
  dynamixel_controller.initServer();

  ros::Timer read_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getReadPeriod()), &DynamixelPositionController::readCallback, &dynamixel_controller);
  ros::Timer write_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getWritePeriod()), &DynamixelPositionController::writeCallback, &dynamixel_controller);
  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getPublishPeriod()), &DynamixelPositionController::publishCallback, &dynamixel_controller);

  ros::spin();

  return 0;
}