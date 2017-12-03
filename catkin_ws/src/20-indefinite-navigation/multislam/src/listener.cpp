#include <stdint.h>
#include <set>
#include <math.h>
#include <fstream>

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/String.h"
#include <tf/LinearMath/Quaternion.h>
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
Values result;
noiseModel::Diagonal::shared_ptr measurementNoise = noiseModel::Diagonal::Sigmas((Vector(2) << 0.1, 0.2));
noiseModel::Diagonal::shared_ptr odomNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.3));
set<int> tagsSeen;

// current pose will change every odometry measurement.
// its used in callback to create landmark "attachment"
int curposeindex = 0;
float curx = 0;
float cury = 0;
float curtheta = 0;
double lastTimeSecs;

ros::Publisher marker_pub;
ros::Publisher marker_arr_pub;

#define ADD_ACTION 0
#define OPTIMIZE_ACTION 1

visualization_msgs::Marker make_marker(int marker_id, uint8_t action, double x, double y, double theta)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = "misteur";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "multislam";
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::ARROW;

	// Both ADD and MODIFY have the same value
	// TODO: we might have to explicitly send a command to DELETE the
	// markers when calling modify
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0;

	const tfScalar yaw = 0.0; // angle around Y
	const tfScalar pitch = 0.0; // angle around X
	// Note: curtheta is the theta after the rotation (i.e. we assume rotation is
	// done already)
	const tfScalar roll = curtheta; //angle around Z
	tf::Quaternion q_tf;
	q_tf.setEuler(yaw, pitch, roll);

	marker.pose.orientation.x = q_tf.getX();
	marker.pose.orientation.y = q_tf.getY();
	marker.pose.orientation.z = q_tf.getZ();
	marker.pose.orientation.w = q_tf.getW();

	// Set the scale of the marker
	marker.scale.x = 0.2; // Arrow length
	marker.scale.y = 0.13; // Arrow width
	marker.scale.z = 0.1; // Arrow height

	if (action == ADD_ACTION)
	{
		// Set new markers red (they haven't been optimized yet)
		marker.color.r = 1.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;
	} else {
		// Set modified (i.e. optimized) markers green
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;
	}

	marker.lifetime = ros::Duration(120); // 2 mins

	return marker;
}


void aprilcallback(const duckietown_msgs::AprilTagsWithInfos::ConstPtr& msg)
{
  vector<duckietown_msgs::AprilTagDetection>::const_iterator it;
  for(it = msg->detections.begin(); it != msg->detections.end(); it++) {
    Symbol l('l', it->id);
    float x = it->pose.pose.position.x;
    float y = it->pose.pose.position.y;
    float range = std::sqrt(x*x + y*y);
    Rot2 bearing = Rot2::atan2(y,x);
    graph.add(BearingRangeFactor<Pose2, Point2>(curposeindex, l, bearing, range, measurementNoise));

    if (tagsSeen.find(it->id) == tagsSeen.end()) {
      initialEstimate.insert(l, Point2(curx + x, cury + y));
      tagsSeen.insert(it->id);
    }
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

  double timeNowSecs = ros::Time::now().toSec();
  double delta_t = timeNowSecs - lastTimeSecs;
  lastTimeSecs = timeNowSecs;

  // maybe msg.omega needs to be switched to degrees/radians?
  double delta_d = delta_t * msg->v;
  double delta_theta = delta_t * msg->omega;
  graph.add(BetweenFactor<Pose2>(curposeindex-1,curposeindex, Pose2(delta_d, 0, delta_theta), odomNoise));

  curx += cos(curtheta) * delta_d;
  cury += sin(curtheta) * delta_d;
  curtheta = fmod(curtheta + delta_theta,M_PI);
  initialEstimate.insert(curposeindex, Pose2(curx, cury, curtheta));

  // Visualize
  marker_pub.publish(make_marker(curposeindex, ADD_ACTION, curx, cury, curtheta));
}

void optimizeCallback(const ros::TimerEvent&)
{
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  result = optimizer.optimize();
  result.print("Final Result:\n");

  visualization_msgs::MarkerArray ma;

  // TODO: When we'll have april tags, we might have to differentiate in the
  // loop (to cast for a Pose2 or Point2)
  // maybe look at https://github.com/devbharat/gtsam/blob/5f15d264ed639cb0add335e2b089086141127fff/gtsam/nonlinear/tests/testValues.cpp
  BOOST_FOREACH(const Values::KeyValuePair& key_val, result)
  {
	  visualization_msgs::Marker marker;
	  const Pose2 &val = dynamic_cast<const Pose2&>(key_val.value);
	  marker = make_marker(key_val.key, OPTIMIZE_ACTION,
			       val.x(), val.y(), val.theta());
	  ma.markers.push_back(marker);
  }

  marker_arr_pub.publish(ma);

}

void vizCallback(const ros::TimerEvent&)
{
  ofstream os("/home/samlaf/winning.dot");
  graph.saveGraph(os, result);
  printf("\n\n\n printed graph to os gogo \n\n\n");
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  lastTimeSecs = ros::Time::now().toSec();

  // Prior on the first pose, set at the origin
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.3, 0.3, 0.1));
  graph.add(PriorFactor<Pose2>(0, Pose2(0, 0, 0), priorNoise));
  initialEstimate.insert(0, Pose2(0.0, 0.0, 0.0));

  marker_pub = n.advertise<visualization_msgs::Marker>("graph_visualization", 1);
  marker_arr_pub = n.advertise<visualization_msgs::MarkerArray>("graph_visualization_arr", 1);

  // Listen to apriltags
  ros::Subscriber aprilsub = n.subscribe("/misteur/apriltags_postprocessing_node/apriltags_out", 1000, aprilcallback);
  // Listen to velocity msgs
  ros::Subscriber velsub = n.subscribe("/misteur/joy_mapper_node/car_cmd", 1000, velcallback);


  // Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.

  ros::Timer opttimer = n.createTimer(ros::Duration(1), optimizeCallback);
  initialEstimate.print("\nInitial Estimate:\n");

//  ros::Timer viztimer = n.createTimer(ros::Duration(60), vizCallback);


  // Calculate and print marginal covariances for all variables
  // Marginals marginals(graph, result);
  // print(marginals.marginalCovariance(1), "1 covariance");

  ros::spin();
  return 0;
}
