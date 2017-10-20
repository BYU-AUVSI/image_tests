#include "ros/ros.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transport_profiler");

    ros::NodeHandle nh;

    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread("/home/ephraimkunz/Desktop/test.jpg",CV_LOAD_IMAGE_COLOR);

    cv_image.encoding = "bgr8";
    //ROS_WARN_STREAM("cv_image: " << cv_image.image);
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/image_raw", 1);
    ros::Rate loop_rate(100000);

    while (nh.ok()) 
    {
        pub.publish(ros_image);
        loop_rate.sleep();
    }
}