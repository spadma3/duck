#include "ros/ros.h" // main ROS include
#include "std_msgs/Float32.h" // number message datatype
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
//#include "duckietown_msgs/Pixel.h"
#include "duckietown_msgs/WheelsCmd.h"
#include "duckietown_msgs/Vector2D.h"
#include <std_srvs/Empty.h>
#include <cmath> // needed for nan
#include <stdint.h>

// GTSAM includes
// #include <gtsam/slam/BetweenFactor.h>
// #include <gtsam/slam/PriorFactor.h>
// #include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <fstream>

// TODO: WheelsCmd should be WheelsCmdStamped (otherwise how do you integrate odometry?)
// simple class to contain the node's variables and code
class slam_node
{
public:
slam_node(); // constructor

// class variables_
private:
	ros::NodeHandle nh_; // interface to this node
	ros::Subscriber odometryTopic_; // interface to wheel odometry
  ros::Subscriber landmarkTopic_; // interface to landmark measurements
	ros::Publisher estimatedPoses_; // interface to the trajectory topic publication
	ros::Publisher estimatedLandmarks_; // interface to the landmarks topic pub

// TODO: these three variables should be computed/given by calibration
  double radius_l_; // radius of the left wheel
  double radius_r_; // radius of the right wheel
  double baseline_lr_; //distance between the center of the two wheels
  geometry_msgs::Pose2D odometricPose_; // 2D pose obtained by integrating odometry over time

	ros::Timer timer_; // the timer object
	double running_sum_; // the running sum since start
	double moving_average_period_; // how often to sample the moving average
	double moving_average_sum_; // sum since the last moving average update
	int moving_average_count_; // number of samples since the last update

	// callback function declarations
	void odometryMeasurementCallback(duckietown_msgs::WheelsCmd::ConstPtr const& msg);
  void landmarkMeasurementCallback(std_msgs::Float32::ConstPtr const& msg);
	void timerCallback(ros::TimerEvent const& event);
};

// program entry point
int main(int argc, char *argv[])
{
	// initialize the ROS client API, giving the default node name
	ros::init(argc, argv, "slam_node");
	slam_node node;
	// enter the ROS main loop
	ros::spin();
	return 0;
}

// class constructor; subscribe to topics and advertise intent to publish
slam_node::slam_node() :
radius_l_(1), radius_r_(1), baseline_lr_(1),
running_sum_(0), moving_average_period_(30), moving_average_sum_(0),
moving_average_count_(0){

	// subscribe to the number stream topic
  odometryTopic_ = nh_.subscribe("/ferrari/joy_mapper/wheels_cmd", 1, &slam_node::odometryMeasurementCallback, this);
	landmarkTopic_ = nh_.subscribe("number_stream", 1, &slam_node::landmarkMeasurementCallback, this);

  //gtsam::nonlinearFactorGraph graph;
  
	// advertise that we'll publish on the corresponding topic
	estimatedPoses_ = nh_.advertise<geometry_msgs::Pose2D>("odometricPose", 1);
	estimatedLandmarks_ = nh_.advertise<std_msgs::Float32>("moving_average", 1);

	// get moving average period from parameter server (or use default value if not present)
	ros::NodeHandle private_nh("~");
	private_nh.param("moving_average_period", moving_average_period_,	moving_average_period_);
	if (moving_average_period_ < 0.5)
	moving_average_period_ = 0.5;

	// create the Timer with period moving_average_period_
	timer_ = nh_.createTimer(ros::Duration(moving_average_period_), &slam_node::timerCallback, this);
	ROS_INFO("Created timer with period of %f seconds", moving_average_period_);
}

// this callback is executed every time an odometry measurement is received
void slam_node::odometryMeasurementCallback(duckietown_msgs::WheelsCmd::ConstPtr const& msg){

  double linVel = (radius_r_* msg->vel_right + radius_l_* msg->vel_left) / 2;
  double omega = (radius_r_* msg->vel_right - radius_l_* msg->vel_left) / baseline_lr_; 

  // DEBUG 
  //std_msgs::Float32 linVel_msg;
  //linVel_msg.data = linVel;
  //estimatedPoses_.publish(linVel_msg);  

  // TODO: this should be an actual deltaT
  double deltaT = 1;   

  // odometricPose_
  double theta_tm1 = odometricPose_.theta;
  double theta_t = theta_tm1 + omega * deltaT;
  
  if (fabs(omega) <= 0.0001){ 
    // straight line
    odometricPose_.x = odometricPose_.x + sin(theta_tm1) * linVel;
    odometricPose_.y = odometricPose_.y + cos(theta_tm1) * linVel;
  }else{
    double v_w_ratio = linVel / omega;
    odometricPose_.x = odometricPose_.x - v_w_ratio * sin(theta_tm1) + v_w_ratio * sin(theta_t);
    odometricPose_.y = odometricPose_.y + v_w_ratio * cos(theta_tm1) - v_w_ratio * sin(theta_t);
  }
  odometricPose_.theta = theta_t;
  geometry_msgs::Pose2D odometricPose_msg;   
  odometricPose_msg.x = odometricPose_.x; 
  odometricPose_msg.y = odometricPose_.y; 
  odometricPose_msg.theta = odometricPose_.theta; 
  estimatedPoses_.publish(odometricPose_msg);   
}

// the callback function for the number stream topic subscription
void slam_node::landmarkMeasurementCallback(std_msgs::Float32::ConstPtr const& msg){
	// add the data to the running sums
	running_sum_ = running_sum_ + msg->data;
	moving_average_sum_ = moving_average_sum_ + msg->data;
	// increment the moving average counter
	moving_average_count_++;
	// create a message containing the running total
	std_msgs::Float32 sum_msg;
	sum_msg.data = running_sum_;
	// publish the running sum message
	estimatedLandmarks_.publish(sum_msg);						
}

// the callback function for the timer event
void slam_node::timerCallback(ros::TimerEvent const& event){
	// create the message containing the moving average
	std_msgs::Float32 moving_average_msg;
	if (moving_average_count_ > 0)
	moving_average_msg.data = moving_average_sum_ / moving_average_count_;
	else
	moving_average_msg.data = nan("");
	// publish the moving average
	estimatedLandmarks_.publish(moving_average_msg);
	// reset the moving average
	moving_average_sum_ = 0;
	moving_average_count_ = 0;
}