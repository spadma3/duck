#include "ros/ros.h" // main ROS include
#include "std_msgs/Float32.h" // number message datatype
#include <ros/console.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
//#include "duckietown_msgs/Pixel.h"
#include "duckietown_msgs/WheelsCmd.h"
#include "duckietown_msgs/Vector2D.h"
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <cmath> // needed for nan
#include <stdint.h>

// GTSAM includes
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Rot2.h>

// #include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <fstream>

// using namespace gtsam;
// TODO LIST:
// 1) visualize pose estimate from vicon and from motor duty
// 2) do rough calibration of the parameters 
// 3) add optimization
// 4) add priors from VICON

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
  ros::Subscriber sub_odometryMeasurementCB_;
  ros::Subscriber sub_landmarkMeasurementCB_;

	ros::Publisher pub_motionModel_;
  ros::Publisher pub_odometry_; 
  ros::Publisher pub_landmark_; 
	ros::Publisher pub_numbers_; // TODO: delete this
  ros::Publisher pub_gtTrajectory;
  ros::Publisher pub_odomTrajectory;

  // TODO: these three variables should be computed/given by calibration
  double radius_l_; // radius of the left wheel
  double radius_r_; // radius of the right wheel
  double baseline_lr_; //distance between the center of the two wheels
  geometry_msgs::Pose2D odomPose_; // 2D pose obtained by integrating odometry till time t
  gtsam::Pose2 odomPose_tm1_; // 2D pose obtained by integrating odometry till time t-1
  gtsam::noiseModel::Diagonal::shared_ptr odomNoise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.01, 0.1, 0.1));
  gtsam::noiseModel::Diagonal::shared_ptr priorNoise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.01, 0.1, 0.1));

  gtsam::Key poseId_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initialGuess_;

  // visualization
  visualization_msgs::Marker gtTrajectory_;
  int gtSubsampleStep_;
  int gtSubsampleCount_;
  visualization_msgs::Marker odomTrajectory_;
  int odomSubsampleStep_;
  int odomSubsampleCount_;

	ros::Timer timer_; // TODO: delete this
	double running_sum_; //TODO: delete this
	double moving_average_period_; // TODO: delete this
	double moving_average_sum_; // TODO: delete this
	int moving_average_count_; // TODO: delete this

	// callback function declarations
  // TODO: move motion model to suitable node
  void motionModelCallback(duckietown_msgs::WheelsCmd::ConstPtr const& msg);
	void odometryMeasurementCallback(geometry_msgs::Pose2D::ConstPtr const& msg);
  void landmarkMeasurementCallback(geometry_msgs::PoseStamped::ConstPtr const& msg);

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
radius_l_(0.02), radius_r_(0.02), baseline_lr_(0.1),
running_sum_(0), moving_average_period_(30), moving_average_sum_(0), poseId_(0), gtSubsampleStep_(50),
moving_average_count_(0){

	// subscribe to the number stream topic
  sub_motionModel_           = nh_.subscribe("/ferrari/joy_mapper/wheels_cmd", 1, &slam_node::motionModelCallback, this);
  sub_odometryMeasurementCB_ = nh_.subscribe("odomPose", 1, &slam_node::odometryMeasurementCallback, this);
	sub_landmarkMeasurementCB_ = nh_.subscribe("/duckiecar/pose", 1, &slam_node::landmarkMeasurementCallback, this);
  
	// advertise that we'll publish on the corresponding topic
	pub_motionModel_ = nh_.advertise<geometry_msgs::Pose2D>("odomPose", 1);
  pub_odometry_    = nh_.advertise<geometry_msgs::Pose2D>("relativePose", 1);
  pub_landmark_    = nh_.advertise<geometry_msgs::PoseStamped>("viconPose", 1);
  pub_gtTrajectory = nh_.advertise<visualization_msgs::Marker>("gtTrajectory", 10);
  pub_odomTrajectory = nh_.advertise<visualization_msgs::Marker>("odomTrajectory", 10);
  
  // TODO: delete
	pub_numbers_     = nh_.advertise<std_msgs::Float32>("moving_average", 1);

  // add prior on first node: this will be the reference frame for us
  graph_.add(gtsam::PriorFactor<gtsam::Pose2>(0, gtsam::Pose2(), priorNoise_));
  initialGuess_.insert(0, gtsam::Pose2());

  // http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
  gtTrajectory_.header.frame_id = "/odom";
  gtTrajectory_.ns = "groundTruthTrajectory";
  gtTrajectory_.action = visualization_msgs::Marker::ADD;
  gtTrajectory_.pose.orientation.w = 1.0;
  gtTrajectory_.id = 1;
  gtTrajectory_.type = visualization_msgs::Marker::LINE_STRIP;
  gtTrajectory_.scale.x = 0.1;
  gtTrajectory_.color.g = 1.0f;
  gtTrajectory_.color.a = 1.0;
  gtSubsampleCount_ = gtSubsampleStep_;

  odomTrajectory_.header.frame_id = "/odom";
  odomTrajectory_.ns = "odometricTrajectory";
  odomTrajectory_.action = visualization_msgs::Marker::ADD;
  odomTrajectory_.pose.orientation.w = 1.0;
  odomTrajectory_.id = 1;
  odomTrajectory_.type = visualization_msgs::Marker::LINE_STRIP;
  odomTrajectory_.scale.x = 0.1;
  odomTrajectory_.color.r = 1.0;
  odomTrajectory_.color.a = 1.0;
  odomSubsampleCount_ = odomSubsampleStep_;
  

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
  double k_d_r = 0.1; 
  double k_d_l = 0.1; 

  // Convertion from motor duty to motor rotation rate (currently a naive multiplication)
  double w_r =  k_d_r * msg->vel_right;
  double w_l =  k_d_l * msg->vel_left;

  // compute linear and angular velocity of the platform
  double v = (radius_r_ * w_r + radius_l_* w_l) / 2;
  double omega =  (radius_r_ * w_r - radius_l_* w_l) / baseline_lr_; 
 
  // TODO: this should be an actual deltaT
  double deltaT = 1;   

  // odomPose_
  double theta_tm1 = odomPose_.theta; // orientation at time t
  double theta_t = theta_tm1 + omega * deltaT; // orientation at time t+1
  
  if (fabs(omega) <= 0.0001){
    // straight line
    odomPose_.x = odomPose_.x + sin(theta_tm1) * v;
    odomPose_.y = odomPose_.y + cos(theta_tm1) * v;
  }else{
    // arc of circle, see "Probabilitic robotics"
    double v_w_ratio = v / omega;
    odomPose_.x = odomPose_.x - v_w_ratio * sin(theta_tm1) + v_w_ratio * sin(theta_t);
    odomPose_.y = odomPose_.y + v_w_ratio * cos(theta_tm1) - v_w_ratio * sin(theta_t);
  }
  odomPose_.theta = theta_t;
  geometry_msgs::Pose2D odomPose_msg;   
  odomPose_msg.x = odomPose_.x; 
  odomPose_msg.y = odomPose_.y; 
  odomPose_msg.theta = odomPose_.theta; 
  pub_motionModel_.publish(odomPose_msg);   

  
  geometry_msgs::Point p;
  p.x = odomPose_.x;
  p.y = odomPose_.y;
  p.z = 0.0;
  odomTrajectory_.points.push_back(p);
  pub_odomTrajectory.publish(odomTrajectory_);
}

///////////////////////////////////////////////////////////////////////////////////////////
// this callback is executed every time an odometry measurement is received
void slam_node::odometryMeasurementCallback(geometry_msgs::Pose2D::ConstPtr const& msg){

  // compute relative pose from wheel odometry
  gtsam::Pose2 odomPose_t(msg->theta, gtsam::Point2(msg->x, msg->y)); // odometric pose at time t
  gtsam::Pose2 odomPose_tm1_t = odomPose_tm1_.between(odomPose_t); // relative pose between t-1 and t

  // key of the new pose to be inserted in the factor graph
  poseId_ += 1;

  // create between factor
  gtsam::BetweenFactor<gtsam::Pose2> odometryFactor(poseId_-1, poseId_, odomPose_tm1_t, odomNoise_);

  // add factor to nonlinear factor graph
  graph_.add(odometryFactor);

  // add initial guess for the new pose
  gtsam::Pose2 initialGuess_t = initialGuess_.at<gtsam::Pose2>(poseId_-1).compose(odomPose_tm1_t); // improved pose estimate
  initialGuess_.insert(poseId_,initialGuess_t);

  // update state
  odomPose_tm1_ = odomPose_t;

  // debug: visualize odometric pose change
  geometry_msgs::Pose2D relativePose_msg;   
  relativePose_msg.x = odomPose_tm1_t.x(); 
  relativePose_msg.y = odomPose_tm1_t.y(); 
  relativePose_msg.theta = odomPose_tm1_t.theta();
  pub_odometry_.publish(relativePose_msg);
}

///////////////////////////////////////////////////////////////////////////////////////////
// the callback function for the number stream topic subscription
void slam_node::landmarkMeasurementCallback(geometry_msgs::PoseStamped::ConstPtr const& msg){
  // compensate for body-camera relative pose (extrinsic calibration)
	// add the data to the running sums
  // points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
  gtSubsampleCount_ -= 1;

  if(gtSubsampleCount_ <= 0){
    geometry_msgs::Point p;
    p = msg->pose.position;
    gtTrajectory_.points.push_back(p);
    pub_gtTrajectory.publish(gtTrajectory_);
    pub_landmark_.publish(msg);	
    gtSubsampleCount_ = gtSubsampleStep_; // reset counter
    // printf("nr points in gtTrajectory_: % \n", gtTrajectory_.points.size());
    std::cout << "size of myints: " << gtTrajectory_.points.size() << std::endl;
    int s = gtTrajectory_.points.size();
    ROS_ERROR("nr points in gtTrajectory: %d", s);
  }
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
