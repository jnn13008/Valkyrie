void callback(const openpose_ros_msgs::OpenPoseHumanListConstPtr& second)
{
  if (first_msg == 0){
    //save first message

    //save transform
    past = ros::Time::now();

    //send msg to move arm horizontally
    sensor_msgs::TimeReference time;
    time.header.stamp = past;
    first_msg = 1;
    pub_move.publish(time);
  }
  else {

    :/publish
    pub_pose.publish(msg);

  }
}

private:
  ros::NodeHandle nh;
  ros::Publisher pub_pose, pub_move;
  ros::Subscriber sub_pose;
  tf::TransformListener listener;
  ros::Time past, now;
  openpose_ros_msgs::OpenPoseHumanList first, second;
  int first_msg;
  //FileStorage fs("logitechc110.yml", FileStorage::READ);
  Mat projection_matrix;


};//End of class msg_filter

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "triangulation");

  //Create an object of class msg_filt that will take care of everything
  triangulation_class Class;

  ros::spin();

  return 0;
}
