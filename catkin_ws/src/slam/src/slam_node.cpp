#include "ros/ros.h" // main ROS include
#include "std_msgs/Float32.h" // number message datatype
#include <ros/console.h>
#include <cmath> // needed for nan
#include <stdint.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

// MESSAGES - sensor_msgs
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
// MESSAGES - geometry_msgs
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
// MESSAGES - duckietown_msgs
#include "duckietown_msgs/Pose2DStamped.h"
#include "duckietown_msgs/Pixel.h"
#include "duckietown_msgs/WheelsCmd.h"
#include "duckietown_msgs/WheelsCmdStamped.h"
#include "duckietown_msgs/Vector2D.h"
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>

// GTSAM includes
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Rot2.h>

#define PI 3.14159265
// using namespace gtsam;
// TODO LIST:
// 1) X visualize pose estimate from vicon and from motor duty
// 1b) visualize trajectory from odometry
// 1c) visualize trajectory from iSAM2
// 2) do rough calibration of the parameters 
// 3) add optimization
// 4) add priors from VICON

class slam_node
{
public:
slam_node(); // constructor

// class variables_
private:
	ros::NodeHandle nh_; // interface to this node

  ros::Subscriber sub_republishWheelCmd_; 
  ros::Subscriber sub_forward_kinematics_;
  ros::Subscriber sub_odometryCB_;
  ros::Subscriber sub_landmarkCB_;
  ros::Subscriber sub_imuCB_;
  ros::Subscriber sub_estimateIMUbiasCB;
  ros::Subscriber sub_viconCB_;

  ros::Publisher pub_republishWheelCmd_;
	ros::Publisher pub_forward_kinematics_;
  ros::Publisher pub_odometryCB_; 
  ros::Publisher pub_landmarkCB_; 
  ros::Publisher pub_imuCB_; 
  ros::Publisher pub_viconCB_;   
  ros::Publisher pub_viconCB_gtTrajectory_;
  ros::Publisher pub_slamTrajectory_;

  ros::Publisher pub_odomTrajectory;

  // TODO: these three variables should be computed/given by calibration
  double radius_l_; // radius of the left wheel
  double radius_r_; // radius of the right wheel
  double baseline_lr_; //distance between the center of the two wheels
  double K_r_;
  double K_l_;

  bool initializedForwardKinematic_;
  bool initializedOdometry_;
  bool initializedIMU_;
  bool initializedCheckIfStill_;
  bool estimateIMUbias_;
  bool insertedAnchor_;

  bool isam2useIMU_;
  bool isam2useVicon_;

  geometry_msgs::Pose2D odomPose_; // 2D pose obtained by integrating odometry till time t
  geometry_msgs::Pose2D imuPose_; // 2D pose (actually rotation only) obtained by integrating odometry till time t
  double tm1_odom_; // last time we acquired an odometry measurement
  double tm1_imu_; // last time we acquired an imu measurement
  gtsam::Pose2 odomPose_tm1_; // 2D pose obtained by integrating odometry till time t-1

  double initialTimeStill_;
  double timeStillThreshold_;
  double gyroOmegaBias_;
  double movingAverageOmega_z_;

  gtsam::noiseModel::Diagonal::shared_ptr odomNoise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.01, 0.1, 0.1));
  gtsam::noiseModel::Diagonal::shared_ptr priorNoise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.0, 1, 1));
  gtsam::noiseModel::Diagonal::shared_ptr imuNoise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.01, 0.0, 0.0));
  gtsam::Key poseId_;

  gtsam::ISAM2 isam;
  gtsam::NonlinearFactorGraph newFactors_;
  gtsam::Values newInitials_;
  gtsam::Values slamEstimate_;

  // visualization
  visualization_msgs::Marker gtTrajectory_;
  int gtSubsampleStep_;
  int gtSubsampleCount_;

  visualization_msgs::Marker odomTrajectory_;
  int odomSubsampleStep_;
  int odomSubsampleCount_;

  visualization_msgs::Marker slamTrajectory_;

	// callback function declarations
  // TODO: move motion model to suitable node
  void optimizeFactorGraph();
  void checkIfStillCallback(duckietown_msgs::WheelsCmdStamped::ConstPtr const& msg);
  void includeIMUfactor();
  void republishWheelsCmdCallback(duckietown_msgs::WheelsCmd::ConstPtr const& msg);
  void forwardKinematicCallback(duckietown_msgs::WheelsCmdStamped::ConstPtr const& msg);
	void odometryCallback(duckietown_msgs::Pose2DStamped::ConstPtr const& msg);
  void imuCallback(sensor_msgs::Imu::ConstPtr const& msg);
  void landmarkCallback(duckietown_msgs::Pose2DStamped::ConstPtr const& msg);
  void viconCallback(geometry_msgs::PoseStamped::ConstPtr const& msg);
};

///////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
	// initialize the ROS client API, giving the default node name
	ros::init(argc, argv, "slam_node");
	slam_node node;
	ros::spin(); // enter the ROS main loop
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////
// class constructor; subscribe to topics and advertise intent to publish
slam_node::slam_node() :
radius_l_(0.02), radius_r_(0.02), baseline_lr_(0.1), K_r_(30), K_l_(30),
poseId_(0), gtSubsampleStep_(50), odomSubsampleStep_(1), 
initializedForwardKinematic_(false), initializedOdometry_(false), estimateIMUbias_(true), initializedCheckIfStill_(false),
timeStillThreshold_(2.0), gyroOmegaBias_(0.0), insertedAnchor_(false), initializedIMU_(false), isam2useIMU_(true), isam2useVicon_(true) {

  sub_republishWheelCmd_ = nh_.subscribe("/ferrari/joy_mapper/wheels_cmd", 1, &slam_node::republishWheelsCmdCallback, this);
  pub_republishWheelCmd_ = nh_.advertise<duckietown_msgs::WheelsCmdStamped>("wheelsCmdStamped", 1);

  sub_forward_kinematics_ = nh_.subscribe("wheelsCmdStamped", 1, &slam_node::forwardKinematicCallback, this);
  pub_forward_kinematics_ = nh_.advertise<duckietown_msgs::Pose2DStamped>("odomPose", 1);
  pub_odomTrajectory = nh_.advertise<visualization_msgs::Marker>("odomTrajectory", 10);

  sub_odometryCB_ = nh_.subscribe("odomPose", 1, &slam_node::odometryCallback, this);
  pub_odometryCB_ = nh_.advertise<duckietown_msgs::Pose2DStamped>("relativePose", 1);  

  sub_imuCB_ = nh_.subscribe("/ferrari/imu/data_raw", 1, &slam_node::imuCallback, this);
  pub_imuCB_ = nh_.advertise<duckietown_msgs::Pose2DStamped>("relativeAngle", 1); 
  sub_estimateIMUbiasCB = nh_.subscribe("wheelsCmdStamped", 1, &slam_node::checkIfStillCallback, this);
  
 //  sub_landmarkCB_ = nh_.subscribe("/duckiecar/pose", 1, &slam_node::landmarkCallback, this);
 //  pub_landmarkCB_ = nh_.advertise<duckietown_msgs::Pose2DStamped>("viconPose", 1);

  pub_slamTrajectory_ = nh_.advertise<visualization_msgs::Marker>("slamTrajectory", 1);

  sub_viconCB_ = nh_.subscribe("/duckiecar/pose", 1, &slam_node::viconCallback, this);
  pub_viconCB_ = nh_.advertise<duckietown_msgs::Pose2DStamped>("viconPose", 1);
  pub_viconCB_gtTrajectory_ = nh_.advertise<visualization_msgs::Marker>("gtTrajectory", 1);

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
  
	ros::NodeHandle private_nh("~");
}

// ///////////////////////////////////////////////////////////////////////////////////////////
void slam_node::optimizeFactorGraph(){
  // Update iSAM with the new factors
  isam.update(newFactors_, newInitials_);
  // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
  isam.update();
  slamEstimate_ = isam.calculateEstimate();

  // Clear the factor graph and values for the next iteration
  newFactors_.resize(0);
  newInitials_.clear();

  // create slam trajectory to publish
  visualization_msgs::Marker slamTrajectory;
  slamTrajectory.header.frame_id = "/odom";
  slamTrajectory.ns = "slamTrajectory";
  slamTrajectory.action = visualization_msgs::Marker::ADD;
  slamTrajectory.pose.orientation.w = 1.0;
  slamTrajectory.id = 1;
  slamTrajectory.type = visualization_msgs::Marker::LINE_STRIP;
  slamTrajectory.scale.x = 0.1;
  slamTrajectory.color.b = 1.0;
  slamTrajectory.color.a = 1.0;

  gtsam::Values poseEstimates = slamEstimate_.filter<gtsam::Pose2>();
  geometry_msgs::Point p;
  for(size_t i = 0; i < poseEstimates.size(); i++){
    gtsam::Pose2 pose_i = poseEstimates.at<gtsam::Pose2>(i);
    p.x = pose_i.x(); p.y = pose_i.y(); p.z = 0.0;
    slamTrajectory.points.push_back(p);
  }
  pub_slamTrajectory_.publish(slamTrajectory);
}

///////////////////////////////////////////////////////////////////////////////////////////
// TODO: get rid of this callback: move to suitable node
void slam_node::republishWheelsCmdCallback(duckietown_msgs::WheelsCmd::ConstPtr const& msg){

  duckietown_msgs::WheelsCmdStamped wheelsCmdStamped_msg;
  ros::Time currentTime = ros::Time::now();
  wheelsCmdStamped_msg.header.stamp = currentTime;
  wheelsCmdStamped_msg.vel_right = msg->vel_right;
  wheelsCmdStamped_msg.vel_left = msg->vel_left;
  pub_republishWheelCmd_.publish(wheelsCmdStamped_msg);
}

///////////////////////////////////////////////////////////////////////////////////////////
// TODO: get rid of this callback: move to suitable node
void slam_node::forwardKinematicCallback(duckietown_msgs::WheelsCmdStamped::ConstPtr const& msg){

  if(initializedForwardKinematic_ == false){
    // initialize: since we do integration, we need initial conditions
    tm1_odom_ = msg->header.stamp.toSec(); 
    initializedForwardKinematic_ = true;
  }else{
    // Convertion from motor duty to motor rotation rate (currently a naive multiplication)
    double w_r =  K_r_ * msg->vel_right;
    double w_l =  K_l_ * msg->vel_left;

    // compute linear and angular velocity of the platform
    double v = (radius_r_ * w_r + radius_l_* w_l) / 2;
    double omega =  (radius_r_ * w_r - radius_l_* w_l) / baseline_lr_; 
   
    // TODO: this should be an actual deltaT
    double t = msg->header.stamp.toSec(); 
    double deltaT = t - tm1_odom_;   

    // ROS_ERROR("deltaT: %g", deltaT);

    // odomPose_
    double theta_tm1 = odomPose_.theta; // orientation at time t
    double theta_t = theta_tm1 + omega * deltaT; // orientation at time t+1
    
    if (fabs(omega) <= 0.0001){
      // straight line
      odomPose_.x = odomPose_.x + cos(theta_tm1) * v * deltaT;
      odomPose_.y = odomPose_.y + sin(theta_tm1) * v * deltaT;
    }else{
      // arc of circle, see "Probabilitic robotics"
      double v_w_ratio = v / omega;
      odomPose_.x = odomPose_.x - v_w_ratio * sin(theta_tm1) + v_w_ratio * sin(theta_t);
      odomPose_.y = odomPose_.y + v_w_ratio * cos(theta_tm1) - v_w_ratio * cos(theta_t);
    }
    odomPose_.theta = theta_t;
    tm1_odom_ = t;

    duckietown_msgs::Pose2DStamped odomPose_msg;   
    odomPose_msg.header = msg->header; // TODO: this looks weird to me
    odomPose_msg.x = odomPose_.x; 
    odomPose_msg.y = odomPose_.y; 
    odomPose_msg.theta = odomPose_.theta; 
    pub_forward_kinematics_.publish(odomPose_msg);   

    // for trajectory visualization
    geometry_msgs::Point p;
    p.x = odomPose_.x; p.y = odomPose_.y; p.z = 0.0;
    odomTrajectory_.points.push_back(p);
    pub_odomTrajectory.publish(odomTrajectory_);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
void slam_node::includeIMUfactor(){

  gtsam::Pose2 imuPose_tm1_t(imuPose_.theta, gtsam::Point2(0.0, 0.0));
  // create between factor
  gtsam::BetweenFactor<gtsam::Pose2> imuFactor(poseId_-1, poseId_, imuPose_tm1_t, imuNoise_);
  // add factor to nonlinear factor graph
  newFactors_.add(imuFactor);
  // reset imu integration
  imuPose_.theta = 0.0;
}

///////////////////////////////////////////////////////////////////////////////////////////
void slam_node::checkIfStillCallback(duckietown_msgs::WheelsCmdStamped::ConstPtr const& msg){

  if(initializedCheckIfStill_ == false){
    initialTimeStill_ = msg->header.stamp.toSec(); 
    initializedCheckIfStill_ = true;
  }else{
     double t = msg->header.stamp.toSec(); 
    // check if vehicle is not moving
    if(fabs(msg->vel_left)>0.0 || fabs(msg->vel_left)>0.0){
      // vehicle is not still any more
      initialTimeStill_ = t;
    }     
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
// this callback is executed every time an odometry measurement is received
void slam_node::odometryCallback(duckietown_msgs::Pose2DStamped::ConstPtr const& msg){

  if(insertedAnchor_ == false){
    // add prior on first node: this will be the reference frame for us
    newFactors_.add(gtsam::PriorFactor<gtsam::Pose2>(0, gtsam::Pose2(), priorNoise_));
    newInitials_.insert(0, gtsam::Pose2());
    slamEstimate_.insert(0, gtsam::Pose2());
    insertedAnchor_ = true;
  }
  // compute relative pose from wheel odometry
  gtsam::Pose2 odomPose_t(msg->theta, gtsam::Point2(msg->x, msg->y)); // odometric pose at time t
  gtsam::Pose2 odomPose_tm1_t = odomPose_tm1_.between(odomPose_t); // relative pose between t-1 and t

  // TODO: add condition that there is enough displacement

  // key of the new pose to be inserted in the factor graph
  poseId_ += 1;
  // create between factor
  gtsam::BetweenFactor<gtsam::Pose2> odometryFactor(poseId_-1, poseId_, odomPose_tm1_t, odomNoise_);
  // add factor to nonlinear factor graph
  newFactors_.add(odometryFactor);
  // add initial guess for the new pose
  gtsam::Pose2 newInitials_t = slamEstimate_.at<gtsam::Pose2>(poseId_-1).compose(odomPose_tm1_t); // improved pose estimate
  newInitials_.insert(poseId_,newInitials_t);
  // update state
  odomPose_tm1_ = odomPose_t;
  // debug: visualize odometric pose change
  duckietown_msgs::Pose2DStamped relativePose_msg;   
  relativePose_msg.header = msg->header; 
  relativePose_msg.x = odomPose_tm1_t.x(); 
  relativePose_msg.y = odomPose_tm1_t.y(); 
  relativePose_msg.theta = odomPose_tm1_t.theta();
  pub_odometryCB_.publish(relativePose_msg);
  // include IMU factors and optimize
  includeIMUfactor();
  optimizeFactorGraph();
}

// ///////////////////////////////////////////////////////////////////////////////////////////
void slam_node::imuCallback(sensor_msgs::Imu::ConstPtr const& msg){

  if(isam2useIMU_ == true){
    if(initializedIMU_ == false){
      initializedIMU_ = true;
      tm1_imu_ = msg->header.stamp.toSec(); 
      movingAverageOmega_z_ = msg->angular_velocity.z;
      imuPose_.x = 0.0; imuPose_.y = 0.0; imuPose_.theta = 0.0;
    }else{
      double t_imu = msg->header.stamp.toSec();
      double deltaT_imu = t_imu - tm1_imu_;
      double omega_z = msg->angular_velocity.z;

      if(estimateIMUbias_ = true){
        // compute moving average of biases
        double alpha = 0.05; // coefficient in the moving average . TODO: relate this to the stillTimeTreshold
        movingAverageOmega_z_ = (1-alpha) * movingAverageOmega_z_ + (alpha) * omega_z; 
        
        // if vehicle has been still for long enough, we estimate biases
        double deltaTstill = t_imu - initialTimeStill_;
        if(deltaTstill > timeStillThreshold_){
          gyroOmegaBias_ = movingAverageOmega_z_;
          ROS_ERROR("gyroOmegaBias_: %f", gyroOmegaBias_);
        }
      }

      double omega_imu = omega_z - gyroOmegaBias_; // we correct with our bias estimate 
      ROS_ERROR("omega_imu: %f", omega_imu);
      imuPose_.theta = imuPose_.theta + omega_imu * deltaT_imu;
      tm1_imu_ = t_imu;
    } 
  } // end "isam2useIMU_"
}

// ///////////////////////////////////////////////////////////////////////////////////////////
// void slam_node::landmarkCallback(duckietown_msgs::Pose2DStamped::ConstPtr const& msg){

// }

///////////////////////////////////////////////////////////////////////////////////////////
void slam_node::viconCallback(geometry_msgs::PoseStamped::ConstPtr const& msg){

  gtSubsampleCount_ -= 1;
  if(gtSubsampleCount_ <= 0){
    geometry_msgs::Point p;
    p = msg->pose.position;
    gtTrajectory_.points.push_back(p);
    pub_viconCB_gtTrajectory_.publish(gtTrajectory_);
    gtSubsampleCount_ = gtSubsampleStep_; // reset counter
    int s = gtTrajectory_.points.size();
    ROS_WARN("nr points in gtTrajectory: %d", s);
  }
  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  double theta = atan2(msg->pose.orientation.w, msg->pose.orientation.z) * 2 + PI/2; // TODO: check this PI/2 
  duckietown_msgs::Pose2DStamped viconPose2D_msg;
  viconPose2D_msg.header = msg->header; // TODO: this looks weird to me
  viconPose2D_msg.x = x;
  viconPose2D_msg.y = y;
  viconPose2D_msg.theta = theta; 
  pub_viconCB_.publish(viconPose2D_msg); 

  if(isam2useVicon_ == true && insertedAnchor_ == false){
    // add prior on first node: this will be the reference frame for us
    gtsam::Pose2 posePrior(theta, gtsam::Point2(x,y));
    newFactors_.add(gtsam::PriorFactor<gtsam::Pose2>(0, posePrior, priorNoise_));
    newInitials_.insert(0, gtsam::Pose2());
    slamEstimate_.insert(0, gtsam::Pose2());
    insertedAnchor_ = true;
  }
}
