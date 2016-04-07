#include "ros/ros.h" 
#include "std_msgs/Float32.h" 
#include <ros/console.h>
#include <cmath> 
#include <stdint.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <std_srvs/Empty.h>
#include <map>
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
#include "duckietown_msgs/SegmentList.h"
#include "duckietown_msgs/AprilTags.h"
#include "duckietown_msgs/TagDetection.h"
#include "duckietown_msgs/WheelsCmdStamped.h"
#include "duckietown_msgs/Vector2D.h"
#include <visualization_msgs/Marker.h>
// GTSAM includes
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
// PARAMETERS
#define PI 3.14159265

// KNOWN ISSUES:
// 1) landmarks are initialized on the back of the car
// 2) camera-body transformation is not considered
// 3) angles are not normalized in [-pi,pi]

// SLAM CLASS
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
  ros::Subscriber sub_lineSegmentsCB_;
  ros::Subscriber sub_imuCB_;
  ros::Subscriber sub_estimateIMUbiasCB;
  ros::Subscriber sub_viconCB_;

  ros::Publisher pub_republishWheelCmd_;
	ros::Publisher pub_forward_kinematics_;
  ros::Publisher pub_odometryCB_; 
  ros::Publisher pub_landmarkCB_; 
  ros::Publisher pub_lineSegmentsCB_;
  ros::Publisher pub_imuCB_; 
  ros::Publisher pub_viconCB_;   
  ros::Publisher pub_viconCB_gtTrajectory_;
  ros::Publisher pub_viconCB_gtLandmarks_;
  ros::Publisher pub_slamTrajectory_;
  ros::Publisher pub_slamLandmarks_;
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
  bool isam2useLandmarks_;

  geometry_msgs::Pose2D odomPose_; // 2D pose obtained by integrating odometry till time t
  geometry_msgs::Pose2D imuPose_; // 2D pose (actually rotation only) obtained by integrating odometry till time t
  double tm1_odom_; // last time we acquired an odometry measurement
  double tm1_imu_; // last time we acquired an imu measurement
  gtsam::Pose2 odomPose_tm1_; // 2D pose obtained by integrating odometry till time t-1
  gtsam::Pose2 gtPose_t_;

  double initialTimeStill_;
  double timeStillThreshold_;
  double gyroOmegaBias_;
  double movingAverageOmega_z_;

  gtsam::noiseModel::Diagonal::shared_ptr priorNoise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.01, 0.01, 0.01));
  gtsam::noiseModel::Diagonal::shared_ptr odomNoise_ = gtsam::noiseModel::Diagonal::Precisions(gtsam::Vector3(1/0.01, 1/0.01, 0.0));
  gtsam::noiseModel::Diagonal::shared_ptr imuNoise_  = gtsam::noiseModel::Diagonal::Precisions(gtsam::Vector3(0.0, 0.0, 1/0.002));
  gtsam::noiseModel::Diagonal::shared_ptr landmarkNoise_ = gtsam::noiseModel::Diagonal::Precisions(gtsam::Vector3(1, 1, 0));
  gtsam::Key poseId_;

  gtsam::ISAM2 isam;
  gtsam::NonlinearFactorGraph newFactors_;
  gtsam::Values newInitials_;
  gtsam::Values slamEstimate_;

  // visualization
  visualization_msgs::Marker gtTrajectory_;
  int gtSubsampleStep_;
  int gtSubsampleCount_;
  visualization_msgs::Marker gtLandmarks_;

  visualization_msgs::Marker odomTrajectory_;
  int odomSubsampleStep_;
  int odomSubsampleCount_;

  visualization_msgs::Marker slamTrajectory_;
  visualization_msgs::Marker slamLandmarks_;
  std::map<int,std_msgs::ColorRGBA> colorMap_;

  visualization_msgs::Marker laneSegments_;

	// callback function declarations
  // TODO: move motion model to suitable node
  void optimizeFactorGraph();
  void visualizeSLAMestimate();
  void checkIfStillCallback(duckietown_msgs::WheelsCmdStamped::ConstPtr const& msg);
  void includeIMUfactor();
  void republishWheelsCmdCallback(duckietown_msgs::WheelsCmd::ConstPtr const& msg);
  void forwardKinematicCallback(duckietown_msgs::WheelsCmdStamped::ConstPtr const& msg);
  void lineSegmentsCallback(duckietown_msgs::SegmentList::ConstPtr const& msg);
	void odometryCallback(duckietown_msgs::Pose2DStamped::ConstPtr const& msg);
  void imuCallback(sensor_msgs::Imu::ConstPtr const& msg);
  void landmarkCallback(duckietown_msgs::AprilTags::ConstPtr const& msg);
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
timeStillThreshold_(2.0), gyroOmegaBias_(0.0), insertedAnchor_(false), initializedIMU_(false), 
isam2useIMU_(true), isam2useLandmarks_(true), isam2useVicon_(true) {

  sub_republishWheelCmd_ = nh_.subscribe("wheels_driver/wheels_cmd", 1, &slam_node::republishWheelsCmdCallback, this);
  pub_republishWheelCmd_ = nh_.advertise<duckietown_msgs::WheelsCmdStamped>("wheelsCmdStamped", 1);

  sub_forward_kinematics_ = nh_.subscribe("wheelsCmdStamped", 1, &slam_node::forwardKinematicCallback, this);
  pub_forward_kinematics_ = nh_.advertise<duckietown_msgs::Pose2DStamped>("odomPose", 1);
  pub_odomTrajectory = nh_.advertise<visualization_msgs::Marker>("odomTrajectory", 10);

  sub_odometryCB_ = nh_.subscribe("odomPose", 1, &slam_node::odometryCallback, this);
  pub_odometryCB_ = nh_.advertise<duckietown_msgs::Pose2DStamped>("relativePose", 1);  

  sub_imuCB_ = nh_.subscribe("imu/data_raw", 1, &slam_node::imuCallback, this);
  pub_imuCB_ = nh_.advertise<duckietown_msgs::Pose2DStamped>("relativeAngle", 1); 
  sub_estimateIMUbiasCB = nh_.subscribe("wheelsCmdStamped", 1, &slam_node::checkIfStillCallback, this);
  
  sub_landmarkCB_ = nh_.subscribe("apriltags/apriltags", 1, &slam_node::landmarkCallback, this);
  pub_landmarkCB_ = nh_.advertise<duckietown_msgs::Pose2DStamped>("landmarkPose", 1);

  pub_slamTrajectory_ = nh_.advertise<visualization_msgs::Marker>("slamTrajectory", 1);
  pub_slamLandmarks_ = nh_.advertise<visualization_msgs::Marker>("slamLandmarks", 1);

  sub_viconCB_ = nh_.subscribe("/duckiecar/pose", 1, &slam_node::viconCallback, this);
  pub_viconCB_ = nh_.advertise<duckietown_msgs::Pose2DStamped>("viconPose", 1);
  pub_viconCB_gtTrajectory_ = nh_.advertise<visualization_msgs::Marker>("gtTrajectory", 1);
  pub_viconCB_gtLandmarks_ = nh_.advertise<visualization_msgs::Marker>("gtLandmarks", 1);

  sub_lineSegmentsCB_ = nh_.subscribe("ground_projection/lineseglist_out", 1, &slam_node::lineSegmentsCallback, this);
  pub_lineSegmentsCB_ = nh_.advertise<visualization_msgs::Marker>("laneSegments", 1);

  // http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
  // TODO: add time stamps to all following?
  // points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  gtTrajectory_.header.frame_id = "/odom";
  gtTrajectory_.ns = "groundTruthTrajectory";
  gtTrajectory_.action = visualization_msgs::Marker::ADD;
  gtTrajectory_.pose.orientation.w = 1.0;
  gtTrajectory_.id = 0;
  gtTrajectory_.type = visualization_msgs::Marker::LINE_STRIP;
  gtTrajectory_.scale.x = 0.1;
  gtTrajectory_.color.g = 1.0f;
  gtTrajectory_.color.a = 1.0;
  gtSubsampleCount_ = gtSubsampleStep_;

  odomTrajectory_.header.frame_id = "/odom";
  odomTrajectory_.ns = "odomTrajectory";
  odomTrajectory_.action = visualization_msgs::Marker::ADD;
  odomTrajectory_.pose.orientation.w = 1.0;
  odomTrajectory_.id = 1;
  odomTrajectory_.type = visualization_msgs::Marker::LINE_STRIP;
  odomTrajectory_.scale.x = 0.1;
  odomTrajectory_.color.r = 1.0;
  odomTrajectory_.color.a = 1.0;
  odomSubsampleCount_ = odomSubsampleStep_;

  slamTrajectory_.header.frame_id = "/odom";
  slamTrajectory_.ns = "slamTrajectory";
  slamTrajectory_.action = visualization_msgs::Marker::ADD;
  slamTrajectory_.pose.orientation.w = 1.0;
  slamTrajectory_.id = 2;
  slamTrajectory_.type = visualization_msgs::Marker::LINE_STRIP;
  slamTrajectory_.scale.x = 0.1;
  slamTrajectory_.color.b = 1.0;
  slamTrajectory_.color.a = 1.0;

  slamLandmarks_.header.frame_id = "/odom";
  slamLandmarks_.ns = "slamLandmarks";
  slamLandmarks_.action = visualization_msgs::Marker::ADD;
  slamLandmarks_.pose.orientation.w = 1.0;
  slamLandmarks_.id = 3;
  slamLandmarks_.type = visualization_msgs::Marker::POINTS;
  slamLandmarks_.scale.x = 0.1;
  slamLandmarks_.scale.y = 0.1;
  slamLandmarks_.color.b = 1.0;
  slamLandmarks_.color.a = 1.0;

  gtLandmarks_.header.frame_id = "/odom";
  gtLandmarks_.ns = "gtLandmarks";
  gtLandmarks_.action = visualization_msgs::Marker::ADD;
  gtLandmarks_.pose.orientation.w = 1.0;
  gtLandmarks_.id = 4;
  gtLandmarks_.type = visualization_msgs::Marker::POINTS;
  gtLandmarks_.scale.x = 0.1;
  gtLandmarks_.scale.y = 0.1;
  gtLandmarks_.color.a = 1.0;
  // gtLandmarks_.color.g = 1.0;

  laneSegments_.header.frame_id = "/odom";
  laneSegments_.ns = "laneSegments";
  laneSegments_.action = visualization_msgs::Marker::ADD;
  laneSegments_.pose.orientation.w = 1.0;
  laneSegments_.id = 5;
  laneSegments_.type = visualization_msgs::Marker::LINE_LIST;
  laneSegments_.scale.x = 0.1;
  laneSegments_.scale.y = 0.1;
  laneSegments_.color.a = 1.0;
  laneSegments_.color.g = 1.0;

	ros::NodeHandle private_nh("~");
}

// ///////////////////////////////////////////////////////////////////////////////////////////
void slam_node::optimizeFactorGraph(){
  // Update iSAM with the new factors
  isam.update(newFactors_, newInitials_);
  // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
  for (size_t iter=0;iter<5;iter++){
    isam.update();
  }

  slamEstimate_ = isam.calculateEstimate();

  // Clear the factor graph and values for the next iteration
  newFactors_.resize(0);
  newInitials_.clear();
}

// ///////////////////////////////////////////////////////////////////////////////////////////
void slam_node::visualizeSLAMestimate(){

  // reset trajectory
  slamTrajectory_.points.resize(0);
  slamLandmarks_.points.resize(0);
  // push back new values
  gtsam::Values poseEstimates = slamEstimate_.filter<gtsam::Pose2>();
  geometry_msgs::Point p;

  BOOST_FOREACH(const gtsam::Values::KeyValuePair& key_value, poseEstimates) {
    gtsam::Key key_i = key_value.key;
    gtsam::Pose2 pose_i = key_value.value.cast<gtsam::Pose2>();
    p.x = pose_i.x(); p.y = pose_i.y(); p.z = 0.0;
    if(gtsam::symbolChr(key_i)=='L'){
      slamLandmarks_.points.push_back(p);
    }else{
      slamTrajectory_.points.push_back(p);
    }
  }
  pub_slamTrajectory_.publish(slamTrajectory_);
  int s = slamTrajectory_.points.size();
  ROS_ERROR("poses in slamTrajectory_: %d", s);

  pub_slamLandmarks_.publish(slamLandmarks_);
  s = slamLandmarks_.points.size();
  ROS_ERROR("landmarks in slamLandmarks_: %d", s);
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
  gtsam::Key keyPose_tm1 = gtsam::Symbol('X', poseId_-1); 
  gtsam::Key keyPose_t = gtsam::Symbol('X', poseId_); 
  gtsam::BetweenFactor<gtsam::Pose2> imuFactor(keyPose_tm1, keyPose_t, imuPose_tm1_t, imuNoise_);
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
    gtsam::Key keyPose_0 = gtsam::Symbol('X', 0); 
    newFactors_.add(gtsam::PriorFactor<gtsam::Pose2>(keyPose_0, gtsam::Pose2(), priorNoise_));
    newInitials_.insert(keyPose_0, gtsam::Pose2());
    slamEstimate_.insert(keyPose_0, gtsam::Pose2());
    insertedAnchor_ = true;
  }
  // compute relative pose from wheel odometry
  gtsam::Pose2 odomPose_t(msg->theta, gtsam::Point2(msg->x, msg->y)); // odometric pose at time t
  gtsam::Pose2 odomPose_tm1_t = odomPose_tm1_.between(odomPose_t); // relative pose between t-1 and t

  // TODO: add condition that there is enough displacement

  // key of the new pose to be inserted in the factor graph
  poseId_ += 1;
  gtsam::Key keyPose_tm1 = gtsam::Symbol('X', poseId_-1); 
  gtsam::Key keyPose_t = gtsam::Symbol('X', poseId_); 
  // create between factor
  gtsam::BetweenFactor<gtsam::Pose2> odometryFactor(keyPose_tm1, keyPose_t, odomPose_tm1_t, odomNoise_);
  // add factor to nonlinear factor graph
  newFactors_.add(odometryFactor);
  // add initial guess for the new pose
  gtsam::Pose2 newInitials_t = slamEstimate_.at<gtsam::Pose2>(keyPose_tm1).compose(odomPose_tm1_t); // improved pose estimate
  newInitials_.insert(keyPose_t,newInitials_t);
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
  visualizeSLAMestimate();
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
        // if vehicle has been still for long enough, we estimate biases
        double deltaTstill = t_imu - initialTimeStill_;
        if(deltaTstill > timeStillThreshold_){
          // compute moving average of biases
          double alpha = 0.05; // coefficient in the moving average . TODO: relate this to the stillTimeTreshold
          movingAverageOmega_z_ = (1-alpha) * movingAverageOmega_z_ + (alpha) * omega_z; 
          gyroOmegaBias_ = movingAverageOmega_z_;
          //ROS_ERROR("gyroOmegaBias_: %f", gyroOmegaBias_);
        }
      }

      double omega_imu = omega_z - gyroOmegaBias_; // we correct with our bias estimate 
      //ROS_ERROR("omega_imu: %f", omega_imu);
      imuPose_.theta = imuPose_.theta + omega_imu * deltaT_imu;
      tm1_imu_ = t_imu;
    } 
  } // end "isam2useIMU_"
}

// ///////////////////////////////////////////////////////////////////////////////////////////
void slam_node::landmarkCallback(duckietown_msgs::AprilTags::ConstPtr const& msg){

  
  if(isam2useLandmarks_ == true){
    ROS_WARN("LANDMARK CALLBACK!!!!");
    for (int l=0; l < msg->detections.size(); l++) // for each tag detection
    {
      duckietown_msgs::TagDetection detection_l = msg->detections[l];
      int id_l = detection_l.id;
      // ROS_INFO_STREAM("using landmark with ID: " << detection_k.id);
      ROS_WARN("using landmark with ID: %d", id_l);

      // parse landmark measurement
      double scale = 4.2;
      double x_l = detection_l.transform.translation.x / scale;
      double y_l = detection_l.transform.translation.y / scale;
      // we project on the XY plane, hence z = 0;
      double theta_l = atan2(detection_l.transform.rotation.w, detection_l.transform.rotation.z) * 2; // TODO: check this PI/2, make this into function 

      gtsam::Key key_l = gtsam::Symbol('L', id_l); 
      gtsam::Pose2 localLandmarkPose(theta_l, gtsam::Point2(x_l,y_l)); 

      ROS_WARN("landmarks measurement: id %d, (%f %f %f)", id_l, x_l,y_l,theta_l);

      // TODO: if last pose is far from current one, add a new pose
      // create between factor
      gtsam::Key keyPose_t = gtsam::Symbol('X', poseId_); 
      gtsam::BetweenFactor<gtsam::Pose2> landmarkFactor(keyPose_t, key_l, localLandmarkPose, 
        gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), landmarkNoise_));
      // add factor to nonlinear factor graph
      newFactors_.add(landmarkFactor);

      if(!isam.valueExists(key_l)){
      // add initial guess for the new landmark pose
      gtsam::Pose2 newInitials_l = slamEstimate_.at<gtsam::Pose2>(keyPose_t).compose(localLandmarkPose);
      newInitials_.insert(key_l,newInitials_l);

      // TODO: try to use landmark rotations later on
      gtsam::noiseModel::Diagonal::shared_ptr landmarkPriorNoise_ = gtsam::noiseModel::Diagonal::Precisions(gtsam::Vector3(0, 0, 10));
      gtsam::PriorFactor<gtsam::Pose2> landmarkPriorFactor(key_l, newInitials_l, landmarkPriorNoise_);
      newFactors_.add(landmarkPriorFactor);
      }

      // visualize landmark position using vicon pose
      gtsam::Pose2 gtInitials_l = gtPose_t_.compose(localLandmarkPose);
      geometry_msgs::Point p;
      p.x = gtInitials_l.x(); p.y = gtInitials_l.y(); p.z = 0.0;
      gtLandmarks_.points.push_back(p);
      std_msgs::ColorRGBA color_l;
      auto search = colorMap_.find(id_l);
      if(search != colorMap_.end()) {
        // we found the key
        color_l = search->second;
        ROS_ERROR("found");
      }
      else{
        double cr = (double)rand()/ (double)RAND_MAX; // random number in [0,1]
        double cg = (double)rand()/ (double)RAND_MAX; // random number in [0,1]
        double cb = (double)rand()/ (double)RAND_MAX; // random number in [0,1]
        color_l.a = 1.0;
        color_l.r = cr;
        color_l.g = cg;
        color_l.b = cb;
        colorMap_.insert( std::pair<int,std_msgs::ColorRGBA>(id_l,color_l) );
      }
      gtLandmarks_.colors.push_back(color_l);
      pub_viconCB_gtLandmarks_.publish(gtLandmarks_);
      int s = gtLandmarks_.points.size();
      ROS_ERROR("landmarks in gtLandmarks_: %d", s);
    } 
  }else{
    ROS_WARN("LANDMARK CALLBACK DISABLED :-(");
  }
}

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
    // ROS_WARN("nr points in gtTrajectory: %d", s);
  }
  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  double theta = atan2(msg->pose.orientation.w, msg->pose.orientation.z) * 2 + PI/2; // TODO: check this PI/2, make this into function 
  duckietown_msgs::Pose2DStamped viconPose2D_msg;
  viconPose2D_msg.header = msg->header; // TODO: this looks weird to me
  viconPose2D_msg.x = x;
  viconPose2D_msg.y = y;
  viconPose2D_msg.theta = theta; 
  pub_viconCB_.publish(viconPose2D_msg); 

  gtPose_t_ = gtsam::Pose2(theta, gtsam::Point2(x,y));

  if(isam2useVicon_ == true && insertedAnchor_ == false){
    // add prior on first node: this will be the reference frame for us
    gtsam::Key keyPose_0 = gtsam::Symbol('X', 0); 
    newFactors_.add(gtsam::PriorFactor<gtsam::Pose2>(keyPose_0, gtPose_t_, priorNoise_));
    newInitials_.insert(keyPose_0, gtsam::Pose2());
    slamEstimate_.insert(keyPose_0, gtsam::Pose2());
    insertedAnchor_ = true;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
void slam_node::lineSegmentsCallback(duckietown_msgs::SegmentList::ConstPtr const& msg){
  // laneSegments_.points.resize(0);
  for (int s=0; s < msg->segments.size(); s++){ // for each segment
    duckietown_msgs::Segment segment_s = msg->segments[s];

    double len = sqrt( pow(segment_s.points[0].x - segment_s.points[1].x , 2) + pow(segment_s.points[0].y - segment_s.points[1].y , 2) );
    ROS_ERROR("len_: %f", len);
    if( len > 0.2 && len < 0.4 ){ // add only segments of reasonable length
      for (int i=0; i < 2; i++){ // for both points
        double x_l = segment_s.points[i].x;  
        double y_l = segment_s.points[i].y;
        gtsam::Point2 p_l(x_l,y_l); 
        gtsam::Point2 p_g = gtPose_t_.transform_from(p_l); // transform to global frame
        geometry_msgs::Point p;
        p.x = p_g.x();
        p.y = p_g.y();
        p.z = 0.0; 
        laneSegments_.points.push_back(p);
        std_msgs::ColorRGBA color_l;
        if (segment_s.color == 0){
          color_l.a = 1.0;
          color_l.r = 1.0;
          color_l.g = 1.0;
          color_l.b = 1.0;
          laneSegments_.colors.push_back(color_l);
        }
      }
    }
  }
  pub_lineSegmentsCB_.publish(laneSegments_);
// uint8 WHITE=0
// uint8 YELLOW=1  
// uint8 RED=2
// uint8 color
// duckietown_msgs/Vector2D[2] pixels_normalized
// duckietown_msgs/Vector2D normal
// geometry_msgs/Point[2] points

}