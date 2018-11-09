/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/open_manipulator_control_gui/qnode.hpp"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace open_manipulator_control_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"open_manipulator_control_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  chain_joint_states_sub_ = n.subscribe("open_manipulator/joint_states", 10, &QNode::jointStatesCallback, this);
  chain_kinematics_pose_sub_ = n.subscribe("open_manipulator/kinematics_pose", 10, &QNode::kinematicsPoseCallback, this);
  ar_marker_pose_sub = n.subscribe("/ar_pose_marker", 10, &QNode::arMarkerPoseMsgCallback, this);

  goal_joint_space_path_client_ = n.serviceClient<open_manipulator_msgs::SetJointPosition>("open_manipulator/goal_joint_space_path");
  goal_task_space_path_client_ = n.serviceClient<open_manipulator_msgs::SetKinematicsPose>("open_manipulator/goal_task_space_path");
  goal_tool_control_client_ = n.serviceClient<open_manipulator_msgs::SetJointPosition>("open_manipulator/goal_tool_control");

  start();
	return true;
}

void QNode::run() {
  ros::Rate loop_rate(10);
  int count = 0;
	while ( ros::ok() ) {

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
}


void QNode::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for(int i = 0; i < msg->name.size(); i ++)
  {
    if(!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle = temp_angle;
}

void QNode::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position = temp_position;
  std::cout << "id000000" << ar_marker_pose.id <<"ar_marker subscribe " << msg->pose.position.x <<","<< msg->pose.position.y <<","<< msg->pose.position.z <<","<< std::endl;
  
}

ar_track_alvar_msgs::AlvarMarker ar_marker_pose;


void QNode::arMarkerPoseMsgCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
    if (msg->markers.size() == 0)
          return;

    std::cout << "ar_marker subscribe " << std::endl;

    ar_marker_pose = msg->markers[0];

        //std::cout.precision(2);

        std::cout << "id111111" << ar_marker_pose.id <<"ar_marker subscribe " << ar_marker_pose.pose.pose.position.x <<","<< ar_marker_pose.pose.pose.position.y <<","<< ar_marker_pose.pose.pose.position.z <<","<< std::endl;
  

    if (ar_marker_pose.id != 1)
          return;
  
  open_manipulator_msgs::SetKinematicsPose srv;
  double secs =ros::Time::now().toSec();

  srv.request.kinematics_pose.pose.position.x = ar_marker_pose.pose.pose.position.y;
  srv.request.kinematics_pose.pose.position.y = -ar_marker_pose.pose.pose.position.x;
  //srv.request.kinematics_pose.pose.position.z = ar_marker_pose.pose.pose.position.z/10+0.04;
  srv.request.kinematics_pose.pose.position.z = 0.2;
  srv.request.path_time = 1;

  if(goal_task_space_path_client_.call(srv))
  {
     std::cout << "11111111111 "<< srv.response.isPlanned << std::endl;
     return ;
     
  }
  std::cout << "a2222222222222 " << std::endl;




}

std::vector<double> QNode::getPresentJointAngle()
{
  return present_joint_angle;
}
std::vector<double> QNode::getPresentGripperAngle()
{
  return present_gripper_angle;
}
std::vector<double> QNode::getPresentKinematicsPose()
{
  return present_kinematic_position;
}

bool QNode::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_client_.call(srv))
  {
    return srv.response.isPlanned;
  }
  return false;
}

bool QNode::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.position = joint_angle;

  if(goal_tool_control_client_.call(srv))
  {
    return srv.response.isPlanned;
  }
  return false;
}

bool QNode::setTaskSpacePath(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;

  if(goal_task_space_path_client_.call(srv))
  {
    return srv.response.isPlanned;
  }
  return false;
}



}  // namespace open_manipulator_control_gui
