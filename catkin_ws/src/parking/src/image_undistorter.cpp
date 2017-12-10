//
// Created by nilsiism on 10.12.17.
//

#include "image_undistorter.h"


ImageUndistorter::ImageUndistorter()
        : it_(nh_)
{
    distorted_image_sub_ = it_.subscribe("/schumi/camera_node/image/raw", 1, &ImageUndistorter::imageCallback, this);
    undistorted_image_pub_ = it_.advertise("/schumi/undistorted_image", 1);

    intrinsic_ = (cv::Mat_<double>(3,3)<<353.32880109224254, 0.0, 340.6187776422325, 0.0, 350.03231511108174, 256.42403699758046, 0.0, 0.0, 1.0);
    distCoeffs_ = (cv::Mat_<double>(5,1) <<  -0.3120820098153013, 0.07611370059003515, -0.0043552007678532715, -0.00045294362665816284, 0.0);
}

ImageUndistorter::~ImageUndistorter()
{

}

void ImageUndistorter::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr_undistorted;

    try
    {
        cv_ptr_undistorted = cv_bridge::toCvCopy(msg);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat undistorted_img = cv_ptr_undistorted->image;
    cv::undistort(undistorted_img, undistorted_img, intrinsic_, distCoeffs_);
    undistorted_image_pub_.publish(cv_ptr_undistorted->toImageMsg());
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_undistorter");

    ImageUndistorter iu;

    ros::spin();

    return 0;
}
