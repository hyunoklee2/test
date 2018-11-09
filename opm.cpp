void QNode::arMarkerPoseMsgCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{    
    if (msg->markers.size() == 0)
      return;

    
    ar_marker_pose = msg->markers[0];   
    std::cout.precision(2); 
    std::cout << "id" << ar_marker_pose.id <<"ar_marker subscribe " << ar_marker_pose.pose.pose.position.x * 1000 <<","<< ar_marker_pose.pose.pose.position.y * 1000 <<","<< ar_marker_pose.pose.pose.position.z * 1000 <<","<< std::endl;

    
      open_manipulator_msgs::SetKinematicsPose srv;
  /*srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;*/
  //double secs =ros::Time::now().toSec();

  srv.request.kinematics_pose.pose.position.x = ar_marker_pose.pose.pose.position.x;
  srv.request.kinematics_pose.pose.position.y = ar_marker_pose.pose.pose.position.y;
  srv.request.kinematics_pose.pose.position.z = ar_marker_pose.pose.pose.position.z;
  srv.request.path_time = secs;

  if(goal_task_space_path_client_.call(srv))
  {
    return srv.response.isPlanned;
  }
  return false;


}
