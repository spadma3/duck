#include "ros/ros.h"
#include "std_msgs/String.h"
#include "duckietown_msgs/AprilTagsWithInfos.h"
#include "duckietown_msgs/Twist2DStamped.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BearingRangeFactor.h>


using namespace std;
using namespace gtsam;

// Global because we want to access it both in main and in the
// callback
NonlinearFactorGraph graph;
Values initialEstimate;
noiseModel::Diagonal::shared_ptr measurementNoise = noiseModel::Diagonal::Sigmas((Vector(2) << 0.1, 0.2));
noiseModel::Diagonal::shared_ptr odomNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.1));

// current pose will change every odometry measurement.
// its used in callback to create landmark "attachment"
int curposeindex = 0;
float curx = 0;
float cury = 0;
float curtheta = 0;

void aprilcallback(const duckietown_msgs::AprilTagsWithInfos::ConstPtr& msg)
{
  vector<duckietown_msgs::AprilTagDetection>::const_iterator it;
  for(it = msg->detections.begin(); it != msg->detections.end(); it++) {
    static Symbol l('l', it->id);
    float x = it->pose.pose.position.x;
    float y = it->pose.pose.position.y;
    float range = std::sqrt(x*x + y*y);
    Rot2 bearing = Rot2::atan2(y,x);
    graph.add(BearingRangeFactor<Pose2, Point2>(curposeindex, l, bearing, range, measurementNoise));
    initialEstimate.insert(l, Point2(1.8, 2.1));
    
    // ROS_INFO("range: [%f]", range);
    // ROS_INFO("bearing: [%d]", bearing);
  }
}

void velcallback(const duckietown_msgs::Twist2DStamped::ConstPtr& msg)
{
  // Twist2DStamped msg type:
  // std_msgs/Header header
  //   uint32 seq
  //   time stamp
  //   string frame_id
  // float32 v
  // float32 omega
  curposeindex += 1;
  printf("%d", curposeindex);
  float delta_t = 0.5; // From rosmsg hz duckietown_msgs/Twist2DStamped
  // maybe msg.omega needs to be switched to degrees/radians?
  graph.add(BetweenFactor<Pose2>(curposeindex-1,curposeindex, Pose2(delta_t * msg->v, 0, delta_t * msg->omega), odomNoise));
  initialEstimate.insert(curposeindex, Pose2(0.0, 0.0,  0.0));
  
}

void timerCallback(const ros::TimerEvent&)
{
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  // Prior on the first pose, set at the origin
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.3, 0.3, 0.1));
  graph.add(PriorFactor<Pose2>(0, Pose2(0, 0, 0), priorNoise));
  initialEstimate.insert(0, Pose2(0.0, 0.0,  0.0   ));

  ros::NodeHandle n;
  // Listen to apriltags
  ros::Subscriber aprilsub = n.subscribe("/mrgoobers/apriltags_postprocessing_node/apriltags_out", 1000, aprilcallback);
  // Listen to velocity msgs
  ros::Subscriber velsub = n.subscribe("/mrgoobers/joy_mapper_node/car_cmd", 1000, velcallback);

  
  // Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.
    
  ros::Timer timer = n.createTimer(ros::Duration(1), timerCallback);
  initialEstimate.print("\nInitial Estimate:\n"); // print
  
  

  // Calculate and print marginal covariances for all variables
  // Marginals marginals(graph, result);
  // print(marginals.marginalCovariance(1), "1 covariance");

  ros::spin();
  return 0;
}



