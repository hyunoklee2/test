int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_example");

  OM_EXAMPLE om_example_;
  om_example_.goalPose = om_example_.getPresentKinematicsPose();

  std::vector<std::string> joint_name;
  std::vector<double> joint_angle;
  double path_time = 2.0;
  joint_name.push_back("joint1"); joint_angle.push_back(0.0);
  joint_name.push_back("joint2"); joint_angle.push_back(0.0);
  joint_name.push_back("joint3"); joint_angle.push_back(0.0);
  joint_name.push_back("joint4"); joint_angle.push_back(0.0);
  setJointSpacePath(joint_name, joint_angle, path_time);

  sleep(2);

  ros::spinOnce();
  
  while(true){
        om_example_.goalPose = om_example_.getPresentKinematicsPose();
        sleep(2);
	om_example_.setGoal();
  	om_example_.setTaskSpacePath(om_example_.goalPose, 3.0);
  }  

  return 0;

}
