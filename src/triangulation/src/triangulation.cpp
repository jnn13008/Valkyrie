#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/TimeReference.h>
#include <openpose_ros_msgs/BoundingBox.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/PointWithProb.h>
#include <triangulation/PointWithProb.h>
#include <triangulation/Pose.h>

class triangulation_class
{
public:
  triangulation_class()
  {
    //Topic you want to publish
    pub_pose = nh.advertise<triangulation::Pose>("filtered/Image_rect_color", 1);
    pub_move = nh.advertise<sensor_msgs::TimeReference>("trigger/StereoMovement", 1);

    //Topic you want to subscribe
    sub_pose = nh.subscribe("openpose/human", 1, &triangulation_class::callback, this);
    first_msg = 0;
    focal_length =0.724; //correct this number with camera calibration
    
    
    
  }

void callback(const openpose_ros_msgs::OpenPoseHumanListConstPtr& second)
{
  if (first_msg == 0){
    //save first message
    for(int i = 0; i<25; i++) {
      first.human_list[0].body_key_points_with_prob[i].x = second->human_list[0].body_key_points_with_prob[i].x;
      first.human_list[0].body_key_points_with_prob[i].y = second->human_list[0].body_key_points_with_prob[i].y;
      first.human_list[0].body_key_points_with_prob[i].prob = second->human_list[0].body_key_points_with_prob[i].prob;
    }
    //save transform
    past = ros::Time::now();

    //send msg to move arm horizontally
    sensor_msgs::TimeReference time;
    time.header.stamp = past;
    first_msg = 1;
    pub_move.publish(time);
  }
  else {
    triangulation::Pose pose;
    now = ros::Time::now();

    //get pose
    tf::StampedTransform transform_now;
    tf::StampedTransform transform_past;

    try{
      listener.lookupTransform("end_effector_frame", "static_frame", now, transform_now);
      listener.lookupTransform("end_effector_frame", "static_frame", past, transform_past);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    float camera_distance = abs(transform_now.getOrigin().getX()-transform_past.getOrigin().getX());

    //fill message
    triangulation::Pose msg;
    msg.header.stamp == now;
    for(int i = 0; i < 25; i++){

      msg.body_key_points_with_prob[i].prob = second->human_list[0].body_key_points_with_prob[i].prob*first.human_list[0].body_key_points_with_prob[i].prob;
      //triangulate
      msg.body_key_points_with_prob[i].z = (focal_length*camera_distance)/abs(first.human_list[0].body_key_points_with_prob[i].x - second->human_list[0].body_key_points_with_prob[i].x);
      //x and y coords
      msg.body_key_points_with_prob[i].x = msg.body_key_points_with_prob[i].z*(second->human_list[0].body_key_points_with_prob[i].x/focal_length);
      msg.body_key_points_with_prob[i].y = msg.body_key_points_with_prob[i].z*(second->human_list[0].body_key_points_with_prob[i].y/focal_length);    

      }
    //publish
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
  float focal_length;
  
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
