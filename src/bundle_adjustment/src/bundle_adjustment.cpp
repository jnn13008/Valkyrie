#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <triangulation/Pose.h>
#include <triangulation/PointWithProb.h>

class BundleAdjustment
{
public:
  BundleAdjustment()
  {
    //Topic you want to publish
    pub = nh.advertise<triangulation::Pose>("/published_topic", 1);

    //Topic you want to subscribe
    sub = nh.subscribe("/subscribed_topic", 1, &BundleAdjustment::callback, this);
  }

  void callback(const triangulation::Pose& input)
  {
    triangulation::Pose output;
    //.... do something with the input and generate the output...
    
    pub.publish(output);
  }

private:
  ros::NodeHandle nh; 
  ros::Publisher pub;
  ros::Subscriber sub;

};//End of class BundleAdjustment

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "Bundle_Adjustment");

  //Create an object of class BundleAdjustment that will take care of everything
  BundleAdjustment Class;

  ros::spin();

  return 0;
}
