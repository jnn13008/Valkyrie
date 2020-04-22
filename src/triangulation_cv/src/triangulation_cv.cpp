#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/TransformStamped.h"
#include <sensor_msgs/TimeReference.h>
#include <triangulation/PointWithProb.h>
#include <triangulation/Pose.h>
#include <openpose_ros_msgs/BoundingBox.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/PointWithProb.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp> //4.1
#include <opencv2/core/persistence.hpp>// 4.1
#include "opencv2/opencv.hpp"

using namespace cv;

class triangulation_class
{
public:
  triangulation_class()
  {
    //Topic you want to publish
    pub_pose = nh.advertise<triangulation::Pose>("three_d_pose", 1);
    pub_move = nh.advertise<sensor_msgs::TimeReference>("trigger/StereoMovement", 1);

    //Topic you want to subscribe
    sub_pose = nh.subscribe("openpose/human", 1, &triangulation_class::callback, this);
    first_msg = 0;
    FileStorage fs("logitechc110.yml", FileStorage::READ);
    fs.open("logitechc110.yml", FileStorage::READ);
    fs["projection_matrix"] >> projection_matrix;
    //Mat C = (Mat_<double>(3,3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    //Mat cameraMatrix = (Mat_<double>(3,3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
    fs.release();
  }

void callback(const openpose_ros_msgs::OpenPoseHumanListConstPtr& second)
{
  if (first_msg == 0){
    //save first message
    for(int i = 0; i < 25; i++) {
      first.human_list[0].body_key_points_with_prob[i].x = second->human_list[0].body_key_points_with_prob[i].x;
      first.human_list[0].body_key_points_with_prob[i].y = second->human_list[0].body_key_points_with_prob[i].y;
      //first.human_list[0].body_key_points_with_prob[i].prob = second->human_list[0].body_key_points_with_prob[i].prob;
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

    float camera_translation_x = abs(transform_now.getOrigin().getX()-transform_past.getOrigin().getX());
    float camera_translation_y = abs(transform_now.getOrigin().getY()-transform_past.getOrigin().getY());

    //fill projection matrix
    Mat projection_matrix_translation;
    projection_matrix.copyTo(projection_matrix_translation);

    projection_matrix_translation.at<float>(3,0) = camera_translation_x*projection_matrix.at<float>(0,0);
    projection_matrix_translation.at<float>(3,1) = camera_translation_y*projection_matrix.at<float>(1,1);

    //fill point arrays for opencv triangulation
    Mat points_perspective_one = Mat_<float>::zeros(2, 25);
    Mat points_perspective_two = Mat_<float>::zeros(2, 25);
    for(int i = 0; i < 25; i++) {
      points_perspective_one.at<float>(0,i) = first.human_list[0].body_key_points_with_prob[i].x;
      points_perspective_one.at<float>(1,i) = first.human_list[0].body_key_points_with_prob[i].y;
      points_perspective_two.at<float>(0,i) = second->human_list[0].body_key_points_with_prob[i].x;
      points_perspective_two.at<float>(1,i) = second->human_list[0].body_key_points_with_prob[i].y;
    }
    //create outpput array for triangulation
    Mat triangulation_result = Mat_<float>::zeros(4, 25);

    //triangulate
    triangulatePoints(projection_matrix, projection_matrix_translation, points_perspective_one, points_perspective_two, triangulation_result);

    //fill message
    triangulation::Pose msg;
    msg.header.stamp = now;

    for(int i = 0; i < 25; i++) {
      msg.body_key_points_with_prob[i].x = triangulation_result.at<float>(0,i);
      msg.body_key_points_with_prob[i].y = triangulation_result.at<float>(1,i);
      msg.body_key_points_with_prob[i].z = triangulation_result.at<float>(2,i);
      // probability? triangulation_result[3,i]
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
