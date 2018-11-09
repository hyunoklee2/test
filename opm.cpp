bool OM_EXAMPLE::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.position = joint_angle;

  if(goal_tool_control_client_.call(srv))
  {
    return srv.response.isPlanned;
  }
  return false;
}

void OM_EXAMPLE::on_btn_gripper_open_clicked(void)
{
  std::vector<double> joint_angle;
  joint_angle.push_back(-1.0);

  if(!qnode.setToolControl(joint_angle))
  {
    //writeLog("[ERR!!] Failed to send service");
    return;
  }

  //writeLog("Send gripper open");
}

void OM_EXAMPLE::on_btn_gripper_close_clicked(void)
{
  std::vector<double> joint_angle;
  joint_angle.push_back(0.5);
  if(!qnode.setToolControl(joint_angle))
  {
    //writeLog("[ERR!!] Failed to send service");
    return;
  }

  //writeLog("Send gripper close");
}


