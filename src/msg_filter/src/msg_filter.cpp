#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/TimeReference.h>

using namespace sensor_msgs;
using namespace message_filters;

class msg_filter
{
public:
  msg_filter()
  {
    //Topic you want to publish
    pub = nh.advertise<Image>("filtered/Image_rect_color", 1);

    //Topic you want to subscribe
    image_sub.subscribe(nh, "usb_cam/image_rect_color", 1);
 		time_sub.subscribe(nh, "trigger", 1);

		//Filter
		//Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, time_sub);
		//sync.registerCallback(boost::bind(&msg_filter::callback, this, _1, _2));
		sync.reset(new Sync(MySyncPolicy(10), image_sub, time_sub));  
		sync->registerCallback(boost::bind(&msg_filter::callback, this, _1, _2));
  }

  void callback(const ImageConstPtr& image, const TimeReferenceConstPtr& time)
  {
  	ROS_INFO("in callback");
    pub.publish(image);
  }

private:
  ros::NodeHandle nh; 
  ros::Publisher pub;
  Subscriber<Image> image_sub;
	Subscriber<TimeReference> time_sub;
	typedef sync_policies::ApproximateTime<Image, TimeReference> MySyncPolicy;
  typedef Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync;
};//End of class msg_filter

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "msg_filter");

  //Create an object of class msg_filt that will take care of everything
  msg_filter Class;

  ros::spin();

  return 0;
}
