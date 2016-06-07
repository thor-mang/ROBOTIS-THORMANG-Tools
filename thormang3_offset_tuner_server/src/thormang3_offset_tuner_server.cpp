/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * Thormang3OffsetTunerServer.cpp
 *
 *  Created on: 2016. 2. 15.
 *      Author: HJSONG
 */

#include "thormang3_offset_tuner_server/thormang3_offset_tuner_server.h"

#define OFFSET_ROSPARAM_KEY "offset"
#define OFFSET_INIT_POS_ROSPARAM_KEY "init_pose_for_offset_tuner"

using namespace thormang3;

OffsetTunerServer::OffsetTunerServer()
  : controller_(0),
    init_file_(""),
    robot_file_(""),
    offset_file_("")
{
}

OffsetTunerServer::~OffsetTunerServer()
{
}

void OffsetTunerServer::setCtrlModule(std::string module)
{
  robotis_controller_msgs::JointCtrlModule control_msg;

  std::map<std::string, robotis_framework::DynamixelState *>::iterator joint_iter;
  ros::NodeHandle nh;
  ros::Publisher set_ctrl_module_pub = nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_ctrl_module", 1);

  BaseModule* base_module = BaseModule::getInstance();

  for (joint_iter = base_module->result_.begin(); joint_iter != base_module->result_.end(); ++joint_iter)
  {
    control_msg.joint_name.push_back(joint_iter->first);
    control_msg.module_name.push_back(module);
  }

  set_ctrl_module_pub.publish(control_msg);
}

bool OffsetTunerServer::initialize()
{
  controller_ = robotis_framework::RobotisController::getInstance();

  ros::NodeHandle nh;

  //Get File Path
  nh.param<std::string>("offset_file_path", offset_file_, "");
  nh.param<std::string>("robot_file_path", robot_file_, "");
  nh.param<std::string>("init_file_path", init_file_, "");

  if ((offset_file_ == "") || (robot_file_ == ""))
  {
    ROS_ERROR("Failed to get file path");
    return -1;
  }
  //
  //Controller Initialize with robot file info
  if (controller_->initialize(robot_file_, init_file_) == false)
  {
    ROS_ERROR("ROBOTIS Controller Initialize Fail!");
    return -1;
  }
  //controller_->LoadOffset(offset_file_);
  controller_->addMotionModule((MotionModule*) BaseModule::getInstance());

  //Initialize RobotOffsetData
  for (std::map<std::string, robotis_framework::Dynamixel *>::iterator robot_it = controller_->robot_->dxls_.begin();
       robot_it != controller_->robot_->dxls_.end(); robot_it++)
  {
    std::string joint_name = robot_it->first;
    robot_offset_data_[joint_name] = new JointOffsetData(0, 0);
    robot_torque_enable_data_[joint_name] = true;
  }

  //Get and Initialize Offset Data
  std::map<std::string, double> _offset_data;
  //bool result = nh.getParam(OFFSET_ROSPARAM_KEY, _offset_data);
//	if(result == true)
//	{
  //ROS_INFO("Succeed to get offset");

  //offset data initialize
  //referring to  robotis file info the data will be initialized

//
//
//
//		for(std::map<std::string, double>::iterator _it = _offset_data.begin(); _it != _offset_data.end(); _it++)
//	    {
//	        std::string _joint_name = _it->first;
//	        double 		_joint_offset = _it->second;
//
//	        RobotOffsetData[_joint_name]		= new JointOffsetData(_joint_offset, 0);
//	        RobotTorqueEnableData[_joint_name]	= true;
//	    }
//	}
//	else {
//		ROS_ERROR("Failed to get offset data");
//		return false;
//	}

  //Load Offset.yaml
  YAML::Node offset_yaml_node = YAML::LoadFile(offset_file_.c_str());

  //Get Offset Data and Init_Pose for Offset Tuning
  YAML::Node offset_data_node = offset_yaml_node[OFFSET_ROSPARAM_KEY];
  YAML::Node offset_init_pose_node = offset_yaml_node[OFFSET_INIT_POS_ROSPARAM_KEY];

  //Initialize Offset Data in RobotOffsetData
  for (YAML::const_iterator node_it = offset_data_node.begin(); node_it != offset_data_node.end(); node_it++)
  {
    std::string joint_name = node_it->first.as<std::string>();
    double offset = node_it->second.as<double>();

    std::map<std::string, JointOffsetData*>::iterator robot_offset_data_it = robot_offset_data_.find(joint_name);
    if (robot_offset_data_it != robot_offset_data_.end())
      robot_offset_data_[joint_name]->joint_offset_rad_ = offset;
  }

  //Initialize Init Pose for Offset Tuning in RobotOffsetData
  for (YAML::const_iterator ode_it = offset_init_pose_node.begin(); ode_it != offset_init_pose_node.end(); ode_it++)
  {
    std::string joint_name = ode_it->first.as<std::string>();
    double offset_init_pose = ode_it->second.as<double>();

    std::map<std::string, JointOffsetData*>::iterator robot_offset_data_it = robot_offset_data_.find(joint_name);
    if (robot_offset_data_it != robot_offset_data_.end())
      robot_offset_data_[joint_name]->joint_init_pos_rad_ = offset_init_pose;
  }

//	//Get Initial Pose for Offset Tunning
//	std::map<std::string, double> _init;
//	result = _nh.getParam(OFFSET_INIT_POS_ROSPARAM_KEY, _init);
//	if(result == true)
//	{
//		std::map<std::string, JointOffsetData*>::iterator _robot_offset_data_it; // iterator for RobotOffsetData
//		ROS_INFO("Succeed to get offset init pose");
//	    for(std::map<std::string, double>::iterator _it = _init.begin(); _it != _init.end(); _it++) {
//	        std::string _joint_name = _it->first;
//	        double 		_joint_init = _it->second;
//
//	        //check the joint name exist at the offset data
//	        _robot_offset_data_it = RobotOffsetData.find(_joint_name);
//
//	        if(_robot_offset_data_it != RobotOffsetData.end())
//	        	RobotOffsetData[_joint_name]->joint_init_pos_rad = _joint_init;
//	        else
//	        	continue;
//	    }
//	}
//	else {
//		ROS_ERROR("Failed to get offset init pose");
//		return false;
//	}

  ROS_INFO(" ");
  ROS_INFO(" ");
  //For Data Check, Print All Available Data
  int i = 0;
  ROS_INFO_STREAM("num" << " | " << "joint_name" << " | " << "InitPose" << ", " << "OffsetAngleRad");
  for (std::map<std::string, JointOffsetData*>::iterator it = robot_offset_data_.begin(); it != robot_offset_data_.end(); it++)
  {
    std::string       joint_name = it->first;
    JointOffsetData  *joint_data = it->second;

    ROS_INFO_STREAM(i << " | " << joint_name << " : " << joint_data->joint_init_pos_rad_ << ", "
                    << joint_data->joint_offset_rad_);
    i++;
  }

  send_tra_sub_             = nh.subscribe("/robotis/base/send_tra", 10, &OffsetTunerServer::stringMsgsCallBack, this);
  joint_offset_data_sub_    = nh.subscribe("/robotis/offset_tuner/joint_offset_data", 10,
                                           &OffsetTunerServer::jointOffsetDataCallback, this);
  joint_torque_enable_sub_  = nh.subscribe("/robotis/offset_tuner/torque_enable", 10,
                                           &OffsetTunerServer::jointTorqueOnOffCallback, this);
  command_sub_              = nh.subscribe("/robotis/offset_tuner/command", 5, &OffsetTunerServer::commandCallback, this);
  offset_data_server_       = nh.advertiseService("robotis/offset_tuner/get_present_joint_offset_data",
                                                  &OffsetTunerServer::getPresentJointOffsetDataServiceCallback, this);

  return true;
}

void OffsetTunerServer::moveToInitPose()
{

  controller_->startTimer();
  BaseModule *base_module = BaseModule::getInstance();

//	Eigen::MatrixXd _offset_init_pose = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1 , 1 );

//    for(std::map<std::string, JointOffsetData*>::iterator _it = RobotOffsetData.begin(); _it != RobotOffsetData.end(); _it++)
//    {
//        std::string 		_joint_name = _it->first;
//        JointOffsetData* 	_joint_data = _it->second;
//
//
//        if ( controller_->robot->dxls.find(_joint_name) == controller_->robot->dxls.end() ) {
//          continue;
//        }
//        else {
//            int id = (int)(controller_->robot->dxls[_joint_name]->id);
//            //if(id > MAX_JOINT_ID) {
//            if(id > MAX_JOINT_ID) {
//            	ROS_ERROR("Invalid JointID");
//            	return;
//            }
//            else {
//            	offset_init_pose.coeffRef(id, 0) = _joint_data->joint_init_pos_rad;
//            }
//        }
//    }

  //make map, key : joint_name, value : joint_init_pos_rad;
  std::map<std::string, double> offset_init_pose;
  for (std::map<std::string, JointOffsetData*>::iterator it = robot_offset_data_.begin();
       it != robot_offset_data_.end(); it++)
  {
    std::string       joint_name = it->first;
    JointOffsetData  *joint_data = it->second;

    if (controller_->robot_->dxls_.find(joint_name) == controller_->robot_->dxls_.end())
      continue;
    else
      offset_init_pose[joint_name] = joint_data->joint_init_pos_rad_ + joint_data->joint_offset_rad_;
  }

  usleep(80 * 1000);
  base_module->PoseGenProc(offset_init_pose);
  usleep(10 * 000);

  while (base_module->isRunning())
    usleep(50 * 1000);

  controller_->stopTimer();
  while (controller_->isTimerRunning())
    usleep(10 * 1000);

  if (controller_->isTimerRunning())
  {
    ROS_INFO("Timer Running");
  }

  setCtrlModule("none");
}

void OffsetTunerServer::stringMsgsCallBack(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM(msg->data);
}

void OffsetTunerServer::commandCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "save")
  {
//		YAML::Node  _baseNode = YAML::LoadFile(offset_file_); // gets the root node
//	    for(std::map<std::string, JointOffsetData*>::iterator _it = RobotOffsetData.begin(); _it != RobotOffsetData.end(); _it++)
//	    {
//	        std::string 		_joint_name = _it->first;
//	        JointOffsetData* 	_joint_data = _it->second;
//
//	        _baseNode["offset"][_joint_name] 					 = _joint_data->joint_offset_rad; // edit one of the nodes
//	        _baseNode["init_pose_for_offset_tuner"][_joint_name] = _joint_data->joint_init_pos_rad; // edit one of the nodes
//	    }
//		std::ofstream fout(offset_file_.c_str());
//		fout << _baseNode; // dump it back into the file

    YAML::Emitter out;
    std::map<std::string, double> offset;
    std::map<std::string, double> init_pose;
    for (std::map<std::string, JointOffsetData*>::iterator it = robot_offset_data_.begin();
         it != robot_offset_data_.end(); it++)
    {
      std::string       joint_name = it->first;
      JointOffsetData  *joint_data = it->second;

      offset[joint_name]    = joint_data->joint_offset_rad_; // edit one of the nodes
      init_pose[joint_name] = joint_data->joint_init_pos_rad_; // edit one of the nodes
    }

    out << YAML::BeginMap;
    out << YAML::Key << "offset" << YAML::Value << offset;
    out << YAML::Key << "init_pose_for_offset_tuner" << YAML::Value << init_pose;
    out << YAML::EndMap;
    std::ofstream fout(offset_file_.c_str());
    fout << out.c_str(); // dump it back into the file

  }
  else if (msg->data == "ini_pose")
    moveToInitPose();
  else
    ROS_INFO_STREAM("Invalid Command : " << msg->data);
}

void OffsetTunerServer::jointOffsetDataCallback(const thormang3_offset_tuner_msgs::JointOffsetData::ConstPtr &msg)
{
  if (controller_->isTimerRunning())
  {
    ROS_ERROR("Timer is running now");
    return;
  }

  //goal position
  ROS_INFO_STREAM(msg->joint_name << " " << msg->goal_value << " " << msg->offset_value
                  << " " << msg->p_gain << " " << msg->i_gain << " " << msg->d_gain);

  std::map<std::string, JointOffsetData*>::iterator it;
  it = robot_offset_data_.find(msg->joint_name);
  if (it == robot_offset_data_.end())
  {
    ROS_ERROR("Invalid Joint Name");
    return;
  }

  if (robot_torque_enable_data_[msg->joint_name] == false)
  {
    ROS_ERROR_STREAM(msg->joint_name << "is turned off the torque");
    return;
  }

  double  goal_pose_rad   = msg->offset_value + msg->goal_value;
  int32_t goal_pose_value = controller_->robot_->dxls_[msg->joint_name]->convertRadian2Value(goal_pose_rad);
  uint8_t dxl_error       = 0;
  int32_t comm_result     = COMM_SUCCESS;

  comm_result = controller_->writeCtrlItem(msg->joint_name,
                                           controller_->robot_->dxls_[msg->joint_name]->goal_position_item_->item_name_,
                                           goal_pose_value, &dxl_error);
  if (comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("Failed to write goal position");
    return;
  }
  else
  {
    robot_offset_data_[msg->joint_name]->joint_init_pos_rad_  = msg->goal_value;
    robot_offset_data_[msg->joint_name]->joint_offset_rad_    = msg->offset_value;
  }

  if (dxl_error != 0)
  {
    ROS_ERROR_STREAM("goal_pos_set : " << msg->joint_name << "  has error " << (int) dxl_error);
  }

  robot_offset_data_[msg->joint_name]->p_gain_ = msg->p_gain;
  robot_offset_data_[msg->joint_name]->i_gain_ = msg->i_gain;
  robot_offset_data_[msg->joint_name]->d_gain_ = msg->d_gain;

//	uint16_t goal_pose_addr = controller_->robot->dxls[msg->joint_name]->goal_position_address;
//	uint8_t _error = 0;
//	int comm_result = COMM_SUCCESS;
//	//length ??
//	int id = controller_->robot->dxls[msg->joint_name]->id;
//	ROS_INFO_STREAM("ID : " << id <<" " << goal_pose_value << " " << (uint32_t)goal_pose_value);
//	comm_result = controller_->Write4Byte(msg->joint_name, goal_pose_addr, goal_pose_value, &_error);
//
//	if(comm_result != COMM_SUCCESS)	{
//		ROS_ERROR("Failed to write goal position");
//		return;
//	}
//	else {
//		RobotOffsetData[msg->joint_name]->joint_init_pos_rad = msg->goal_value;
//		RobotOffsetData[msg->joint_name]->joint_offset_rad   = msg->offset_value;
//	}
//
//	if(_error != 0 )	{
//		fprintf(stderr, "%c %d\n", _error, _error);
//		ROS_ERROR_STREAM("goal_pos_set : " << msg->joint_name << "  has error "<< (int)_error);
//	}
//
//	uint16_t pgain_addr = controller_->robot->dxls[msg->joint_name]->position_p_gain_address;
//	comm_result = controller_->Write2Byte(msg->joint_name, pgain_addr, msg->p_gain, &_error);
//	if(comm_result != COMM_SUCCESS)	{
//		ROS_ERROR("Failed to write p gain");
//		return;
//	}
//	else
//		RobotOffsetData[msg->joint_name]->p_gain = msg->p_gain;
//
//	if(_error != 0 )	{
//		ROS_ERROR_STREAM("pgain_set : " << msg->joint_name << "  has error "<< (int)_error);
//	}
//
//	uint16_t igain_addr = controller_->robot->dxls[msg->joint_name]->position_i_gain_address;
//	if(igain_addr != 0 ) {
//		uint8_t length = controller_->robot->dxls[msg->joint_name]->ctrl_table["position_I_gain"]->data_length;
//		if(length == 1)
//			comm_result = controller_->Write1Byte(msg->joint_name, igain_addr, (uint8_t)(msg->i_gain), &_error);
//		else if(length == 2)
//			comm_result = controller_->Write2Byte(msg->joint_name, igain_addr, (uint16_t)(msg->i_gain), &_error);
//		else if(length == 4)
//			comm_result = controller_->Write4Byte(msg->joint_name, igain_addr, (uint32_t)(msg->i_gain), &_error);
//		else {
//			return;
//		}
//
//		if(comm_result != COMM_SUCCESS)	{
//			ROS_ERROR("Failed to write i gain");
//			return;
//		}
//		else
//			RobotOffsetData[msg->joint_name]->i_gain = msg->i_gain;
//
//		if(_error != 0 )	{
//			ROS_ERROR_STREAM("igain_set : " << msg->joint_name << "  has error "<< (int)_error);
//		}
//	}
//
//
//
//	uint16_t dgain_addr = controller_->robot->dxls[msg->joint_name]->position_d_gain_address;
//	if(dgain_addr != 0 ) {
//		comm_result = controller_->Write2Byte(msg->joint_name, dgain_addr, msg->d_gain, &_error);
//		if(comm_result != COMM_SUCCESS)	{
//			ROS_ERROR("Failed to write d gain");
//			return;
//		}
//		else
//			RobotOffsetData[msg->joint_name]->d_gain = msg->d_gain;
//
//		if(_error != 0 )	{
//			ROS_ERROR_STREAM("dgain_set : " << msg->joint_name << "  has error "<< (int)_error);
//		}
//	}
}

void OffsetTunerServer::jointTorqueOnOffCallback(
    const thormang3_offset_tuner_msgs::JointTorqueOnOffArray::ConstPtr& msg)
{
  for (unsigned int i = 0; i < msg->torque_enable_data.size(); i++)
  {
    std::string joint_name = msg->torque_enable_data[i].joint_name;
    bool torque_enable = msg->torque_enable_data[i].torque_enable;
    ROS_INFO_STREAM(i << " " << joint_name << torque_enable);

    std::map<std::string, JointOffsetData*>::iterator it = robot_offset_data_.find(joint_name);
    if (it == robot_offset_data_.end())
    {
      ROS_ERROR("Invalid Joint Name");
      continue;
    }
    else
    {
      int32_t comm_result = COMM_SUCCESS;
      uint8_t dxl_error = 0;
      uint8_t torque_enable_value = 0;

      if (torque_enable)
        torque_enable_value = 1;
      else
        torque_enable_value = 0;

      comm_result = controller_->writeCtrlItem(joint_name,
                                               controller_->robot_->dxls_[joint_name]->torque_enable_item_->item_name_,
                                               torque_enable_value, &dxl_error);
      if (comm_result != COMM_SUCCESS)
      {
        ROS_ERROR("Failed to write goal position");
      }
      else
      {
        robot_torque_enable_data_[joint_name] = torque_enable;
      }

      if (dxl_error != 0)
      {
        ROS_ERROR_STREAM("goal_pos_set : " << joint_name << "  has error " << (int) dxl_error);
      }
    }
  }

//	for(unsigned int _i = 0;  _i < msg->torque_enable_data.size(); _i++)
//	{
//		std::string _joint_name = msg->torque_enable_data[_i].joint_name;
//
//		std::map<std::string, JointOffsetData*>::iterator _it;
//		for(_it = RobotOffsetData.begin(); _it != RobotOffsetData.end(); _it++)
//		{
//			if(_it->first == _joint_name)
//				break;
//		}
//
//		if(_it == RobotOffsetData.end())
//		{
//			ROS_ERROR_STREAM("Invalid Joint Name : " << _joint_name);
//			return;
//		}

//		uint16_t _torque_enable_addr = controller_->robot->dxls[msg->torque_enable_data[_i].joint_name]->torque_enable_address;
//		uint8_t  _torque_enable = 0;
//		if(msg->torque_enable_data[_i].torque_enable == true)
//			_torque_enable = 1;
//
//		uint8_t _error = 0;
//		int comm_result = controller_->Write1Byte(msg->torque_enable_data[_i].joint_name, _torque_enable_addr, _torque_enable, &_error);
//		if((comm_result != COMM_SUCCESS))
//		{
//			ROS_ERROR("Failed to write torque enable [result : %d]", comm_result);
//			return;
//		}
//
//		if(_error != 0 )	{
//			ROS_ERROR_STREAM( msg->torque_enable_data[_i].joint_name << "  has error "<< (int)_error);
//		}
//
//		if(RobotTorqueEnableData[msg->torque_enable_data[_i].joint_name] == false)
//		{
//			uint32_t present_pos_value = 0;
//			uint8_t  error = 0;
//			int 	 comm_result = COMM_SUCCESS;
//			int comm_count = 0;
//			int max_comm_count = 3;
//			for(comm_count = 0; comm_count < max_comm_count; comm_count++) {
//				comm_result = controller_->Read4Byte(_joint_name, controller_->robot->dxls[_joint_name]->present_position_address, &present_pos_value, &error);
//				if(comm_result != COMM_SUCCESS)
//				{
//					ROS_ERROR("Failed to read present pos");
//					return;
//				}
//				else {
//					RobotOffsetData[msg->torque_enable_data[_i].joint_name]->joint_offset_rad = controller_->robot->dxls[_joint_name]->ConvertValue2Radian(present_pos_value);
//					break;
//				}
//			}
//
//			if(comm_count == max_comm_count)
//			{
//				ROS_ERROR("All Comm Fail");
//			}
//		}
//
//		RobotTorqueEnableData[msg->torque_enable_data[_i].joint_name] = msg->torque_enable_data[_i].torque_enable;
//	}
}

bool OffsetTunerServer::getPresentJointOffsetDataServiceCallback(
                        thormang3_offset_tuner_msgs::GetPresentJointOffsetData::Request &req,
                        thormang3_offset_tuner_msgs::GetPresentJointOffsetData::Response &res)
{

  ROS_INFO("GetPresentJointOffsetDataService Called");

  for (std::map<std::string, JointOffsetData*>::iterator it = robot_offset_data_.begin();
       it != robot_offset_data_.end(); it++)
  {
    std::string       joint_name = it->first;
    JointOffsetData  *joint_data = it->second;

    thormang3_offset_tuner_msgs::JointOffsetPositionData joint_offset_pos;

    int32_t present_pos_value = 0;
    uint8_t dxl_error         = 0;
    int     comm_result       = COMM_SUCCESS;

    comm_result = controller_->readCtrlItem(joint_name,
                                            controller_->robot_->dxls_[joint_name]->present_position_item_->item_name_,
                                            (uint32_t*) &present_pos_value,
                                            &dxl_error);
    if (comm_result != COMM_SUCCESS)
    {
      ROS_ERROR("Failed to read present pos");
      return false;
    }
    else
    {
      if (dxl_error != 0)
      {
        ROS_ERROR_STREAM(joint_name << "  has error " << (int) dxl_error);
      }

      joint_offset_pos.joint_name     = joint_name;
      joint_offset_pos.goal_value     = joint_data->joint_init_pos_rad_;
      joint_offset_pos.offset_value   = joint_data->joint_offset_rad_;
      joint_offset_pos.present_value  = controller_->robot_->dxls_[joint_name]->convertValue2Radian(present_pos_value);
      joint_offset_pos.p_gain         = joint_data->p_gain_;
      joint_offset_pos.i_gain         = joint_data->i_gain_;
      joint_offset_pos.d_gain         = joint_data->d_gain_;

      res.present_data_array.push_back(joint_offset_pos);
    }
  }
  return true;
}

