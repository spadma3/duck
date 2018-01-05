#include "ros/ros.h"
#include "std_msgs/String.h"
#include "duckietown_msgs/AprilTagsWithInfos.h"

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
noiseModel::Diagonal::shared_ptr measurementNoise = noiseModel::Diagonal::Sigmas((Vector(2) << 0.1, 0.2));

// current pose will change every odometry measurement.
// its used in callback to create landmark "attachment"
static Symbol curpose('x',0);

void callback(const duckietown_msgs::AprilTagsWithInfos msg)
{
  // just taking first detection ([0]) for now. Should include all of them
  static Symbol l('l', msg.detections[0].id);
  float x = msg.detections[0].pose.pose.position.x;
  float y = msg.detections[0].pose.pose.position.y;
  float range = std::sqrt(x*x + y*y);
  Rot2 bearing = Rot2::fromDegrees(atan(y/x));
  graph.add(BearingRangeFactor<Pose2, Point2>(curpose, l, bearing, range, measurementNoise));
  
  ROS_INFO("range: [%f]", range);
  // ROS_INFO("bearing: [%d]", bearing);
}

// Make a callback for odometry measurements (perhaps like liam said)

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  // Listen to apriltags
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/mrgoobers/apriltags_postprocessing_node/apriltags_out", 1000, callback);

  // Prior on the first pose, set at the origin
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.3, 0.3, 0.1));
  graph.add(PriorFactor<Pose2>(1, Pose2(0, 0, 0), priorNoise));

  // For simplicity, we will use the same noise model for odometry and loop closures
  noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.1));

  // Odometry factors
  // These should come from visualodometry node (set to 2 for now)
  int VISUAL_ODOMETRY = 2;
  graph.add(BetweenFactor<Pose2>(1, 2, Pose2(VISUAL_ODOMETRY, 0, 0     ), model));
  graph.add(BetweenFactor<Pose2>(2, 3, Pose2(VISUAL_ODOMETRY, 0, M_PI_2), model));
  graph.add(BetweenFactor<Pose2>(3, 4, Pose2(VISUAL_ODOMETRY, 0, M_PI_2), model));
  graph.add(BetweenFactor<Pose2>(4, 5, Pose2(VISUAL_ODOMETRY, 0, M_PI_2), model));

  // Loop closure constraint
  // These will just be our landmarks (april tags) which may or may
  // not be loop closures (depending if they are seen from more than 1
  // pose)
  graph.add(BetweenFactor<Pose2>(5, 2, Pose2(2, 0, M_PI_2), model));

  // 3. Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initialEstimate;
  initialEstimate.insert(1, Pose2(0.5, 0.0,  0.2   ));
  initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2   ));
  initialEstimate.insert(3, Pose2(4.1, 0.1,  M_PI_2));
  initialEstimate.insert(4, Pose2(4.0, 2.0,  M_PI  ));
  initialEstimate.insert(5, Pose2(2.1, 2.1, -M_PI_2));
  initialEstimate.print("\nInitial Estimate:\n"); // print

  // Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // Calculate and print marginal covariances for all variables
  Marginals marginals(graph, result);
  // print(marginals.marginalCovariance(x1), "x1 covariance");
  // print(marginals.marginalCovariance(x2), "x2 covariance");
  // print(marginals.marginalCovariance(x3), "x3 covariance");
  // print(marginals.marginalCovariance(l1), "l1 covariance");
  // print(marginals.marginalCovariance(l2), "l2 covariance");

  ros::spin();
  return 0;
}



