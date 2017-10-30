#include "ros/ros.h"
#include "ros/package.h"
#include <image_transport/image_transport.h>
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
    image_transport::ImageTransport it(nh);
    ROS_INFO_STREAM("Starting compressed talker node");

    // Get parameters
    int publish_frequency, image_size, publisher_queue_size;
    bool use_compression;
    if(!ros::param::get("publisher_frequency", publish_frequency))
        ROS_WARN_STREAM("Publisher frequency not set");
    if(!ros::param::get("image_size", image_size))
        ROS_WARN_STREAM("Image size not set");
    if(!ros::param::get("publisher_queue_size", publisher_queue_size))
        ROS_WARN_STREAM("Publisher queue size not set");
    if(!ros::param::get("use_compression", use_compression))
        ROS_WARN_STREAM("Use compression not set");

    ROS_INFO_STREAM("Getting image");
    cv_bridge::CvImage cv_image;
    cv_image.image = getImageForSize(image_size);

    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);

    // We will use a republisher node to get this into /image_raw since we have a python stamper.
    image_transport::Publisher pub = it.advertise("/image_raw_compressed", publisher_queue_size);
    
    ros::Rate rate(publish_frequency);

    ROS_INFO_STREAM("Starting publish loop");    
    while (nh.ok()) 
    {
        pub.publish(ros_image);
        rate.sleep();
    }
}