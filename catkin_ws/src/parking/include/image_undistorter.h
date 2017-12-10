//
// Created by nilsiism on 10.12.17.
//

#ifndef IMAGE_UNDIST_IMAGE_UNDISTORTER_H
#define IMAGE_UNDIST_IMAGE_UNDISTORTER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <string>

class ImageUndistorter
{
public:
    ImageUndistorter();
    ~ImageUndistorter();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

protected:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    image_transport::Subscriber distorted_image_sub_;
    image_transport::Publisher undistorted_image_pub_;

private:
    cv::Mat intrinsic_;
    cv::Mat distCoeffs_;
};

#endif //IMAGE_UNDIST_IMAGE_UNDISTORTER_H
