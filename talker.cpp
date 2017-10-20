#include "ros/ros.h"
#include "ros/package.h"
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

cv::Mat getImageForSize(int image_size) {
    cv::Mat image;
    std::string path;

    switch(image_size) {
        case 1:
            path = ros::package::getPath("image_transport_profiler") + "/size1.jpg";
            break;

        case 16:
            path = ros::package::getPath("image_transport_profiler") + "/size16.jpg";        
            break;

        case 24:
            path = ros::package::getPath("image_transport_profiler") + "/size24.jpg";        
        default:
            break;
    }

    image = cv::imread(path,CV_LOAD_IMAGE_COLOR);            
    return image;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transport_profiler");
    ros::NodeHandle nh;

    // Get parameters
    int publish_frequency, image_size, publisher_queue_size;
    if(!ros::param::get("publisher_frequency", publish_frequency))
        ROS_WARN_STREAM("Publisher frequency not set");
    if(!ros::param::get("image_size", image_size))
        ROS_WARN_STREAM("Image size not set");
    if(!ros::param::get("publisher_queue_size", publisher_queue_size))
        ROS_WARN_STREAM("Publisher queue size not set");


    cv_bridge::CvImage cv_image;
    cv_image.image = getImageForSize(image_size);

    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/image_raw", publisher_queue_size);
    
    ros::Rate rate(publish_frequency);

    while (nh.ok()) 
    {
        pub.publish(ros_image);
        rate.sleep();
    }
}