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
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
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
  ros::Subscriber sub_motionModel_;
  ros::Subscriber sub_odometryOdometryCB_;
  ros::Subscriber sub_landmarkMeasurementCB_;

	ros::Publisher pub_motionModel_;
	ros::Publisher pub_numbers_; 

  // TODO: these three variables should be computed/given by calibration
  double radius_l_; // radius of the left wheel
  double radius_r_; // radius of the right wheel
  double baseline_lr_; //distance between the center of the two wheels
  geometry_msgs::Pose2D odometricPose_; // 2D pose obtained by integrating odometry till time t
  gtsam::Pose2 odometricPose_tm1_; // 2D pose obtained by integrating odometry till time t-1
  noiseModel::Diagonal::shared_ptr odomNoise_ = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));

	ros::Timer timer_; // the timer object
	double running_sum_; // the running sum since start
	double moving_average_period_; // how often to sample the moving average
	double moving_average_sum_; // sum since the last moving average update
	int moving_average_count_; // number of samples since the last update

	// callback function declarations
  // TODO: move motion model to suitable node
  void motionModelCallback(duckietown_msgs::WheelsCmd::ConstPtr const& msg);
	void odometryMeasurementCallback(geometry_msgs::Pose2D::ConstPtr const& msg);
  void landmarkMeasurementCallback(std_msgs::Float32::ConstPtr const& msg);
	void timerCallback(ros::TimerEvent const& event);
};

// program entry point
int main(int argc, char *argv[])
{
	// initialize the ROS client API, giving the default node name
	ros::init(argc, argv, "slam_node");
	slam_node node;
	ros::spin(); // enter the ROS main loop
	return 0;
}

// class constructor; subscribe to topics and advertise intent to publish
slam_node::slam_node() :
radius_l_(1), radius_r_(1), baseline_lr_(1),
running_sum_(0), moving_average_period_(30), moving_average_sum_(0),
moving_average_count_(0){

	// subscribe to the number stream topic
  sub_motionModel_           = nh_.subscribe("/ferrari/joy_mapper/wheels_cmd", 1, &slam_node::motionModelCallback, this);
  sub_odometryOdometryCB_    = nh_.subscribe("odometricPose", 1, &slam_node::odometryMeasurementCallback, this);
	sub_landmarkMeasurementCB_ = nh_.subscribe("number_stream", 1, &slam_node::landmarkMeasurementCallback, this);
  
	// advertise that we'll publish on the corresponding topic
	pub_motionModel_ =     nh_.advertise<geometry_msgs::Pose2D>("odometricPose", 1);
	pub_numbers_ = nh_.advertise<std_msgs::Float32>("moving_average", 1);

  gtsam::NonlinearFactorGraph::shared_ptr graph;

	// get moving average period from parameter server (or use default value if not present)
	ros::NodeHandle private_nh("~");
	private_nh.param("moving_average_period", moving_average_period_,	moving_average_period_);
	if (moving_average_period_ < 0.5)
	moving_average_period_ = 0.5;

	// create the Timer with period moving_average_period_
	timer_ = nh_.createTimer(ros::Duration(moving_average_period_), &slam_node::timerCallback, this);
	ROS_INFO("Created timer with period of %f seconds", moving_average_period_);
}

///////////////////////////////////////////////////////////////////////////////////////////
// this callback is executed every time an odometry measurement is received
void slam_node::motionModelCallback(duckietown_msgs::WheelsCmd::ConstPtr const& msg){

  // TODO: Convertion from motor duty to motor rotation rate (currently a naive multiplication)
  // TODO: these must be computed automatically from calibration
  double k_d_r = 1; 
  double k_d_l = 1; 

  // Convertion from motor duty to motor rotation rate (currently a naive multiplication)
  double w_r =  k_d_r * msg->vel_right;
  double w_l =  k_d_l * msg->vel_left;

  // compute linear and angular velocity of the platform
  double v = (radius_r_ * w_r + radius_l_* w_l) / 2;
  double omega =  (radius_r_ * w_r - radius_l_* w_l) / baseline_lr_; 
 
  // TODO: this should be an actual deltaT
  double deltaT = 1;   

  // odometricPose_
  double theta_tm1 = odometricPose_.theta; // orientation at time t
  double theta_t = theta_tm1 + omega * deltaT; // orientation at time t+1
  
  if (fabs(omega) <= 0.0001){
    // straight line
    odometricPose_.x = odometricPose_.x + sin(theta_tm1) * v;
    odometricPose_.y = odometricPose_.y + cos(theta_tm1) * v;
  }else{
    // arc of circle, see "Probabilitic robotics"
    double v_w_ratio = v / omega;
    odometricPose_.x = odometricPose_.x - v_w_ratio * sin(theta_tm1) + v_w_ratio * sin(theta_t);
    odometricPose_.y = odometricPose_.y + v_w_ratio * cos(theta_tm1) - v_w_ratio * sin(theta_t);
  }
  odometricPose_.theta = theta_t;
  geometry_msgs::Pose2D odometricPose_msg;   
  odometricPose_msg.x = odometricPose_.x; 
  odometricPose_msg.y = odometricPose_.y; 
  odometricPose_msg.theta = odometricPose_.theta; 
  pub_motionModel_.publish(odometricPose_msg);   
}

///////////////////////////////////////////////////////////////////////////////////////////
// this callback is executed every time an odometry measurement is received
void slam_node::odometryMeasurementCallback(geometry_msgs::Pose2D::ConstPtr const& msg){

  // compute relative pose from wheel odometry
  gtsam::Pose2 odometricPose_t(msg->theta, gtsam::Point2(msg->x, msg->y));
  gtsam::Pose2 odometricPose_tm1_t = odometricPose_tm1_.between(odometricPose_t);

  // create between factor

  // add factor to nonlinear factor graph

    // odometricPose_.theta = theta_t;
  // geometry_msgs::Pose2D odometricPose_msg;   
  // odometricPose_msg.x = odometricPose_.x; 
  // odometricPose_msg.y = odometricPose_.y; 
  // odometricPose_msg.theta = odometricPose_.theta; 
  // estimatedPoses_.publish(odometricPose_msg); 

}

///////////////////////////////////////////////////////////////////////////////////////////
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
	pub_numbers_.publish(sum_msg);						
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
	pub_numbers_.publish(moving_average_msg);
	// reset the moving average
	moving_average_sum_ = 0;
	moving_average_count_ = 0;
}