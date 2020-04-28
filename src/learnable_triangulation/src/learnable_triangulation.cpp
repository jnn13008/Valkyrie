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
    sub_pose = nh.subscribe("/filtered/Image_rect_color", 1, &triangulation_class::callback, this);
    first_msg = 0;
    FileStorage fs("logitechc110.yml", FileStorage::READ);
    fs.open("logitechc110.yml", FileStorage::READ);
    fs["projection_matrix"] >> projection_matrix;
    fs.release();
  }

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
