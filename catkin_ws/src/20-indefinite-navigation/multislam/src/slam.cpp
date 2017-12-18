#include <stdint.h>
#include <set>
#include <math.h>

#include "ros/ros.h"
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/String.h"
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include "duckietown_msgs/AprilTagsWithInfos.h"
#include "duckietown_msgs/Twist2DStamped.h"
#include "nav_msgs/Odometry.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BearingRangeFactor.h>

#define ADD_ACTION 0
#define OPTIMIZE_ACTION 1

using namespace std;
using namespace gtsam;

ros::Publisher marker_pub;
ros::Publisher marker_arr_pub;
std::string veh_name;

visualization_msgs::Marker make_pose_marker(int marker_id, uint8_t action, double x, double y, double theta)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = veh_name;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "multislam";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::ARROW;

    // Both ADD and MODIFY have the same value
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;

    const tfScalar yaw = 0.0; // angle around Y
    const tfScalar pitch = 0.0; // angle around X
    // Note: curtheta is the theta after the rotation (i.e. we assume rotation is
    // done already)
    const tfScalar roll = theta; //angle around Z
    tf::Quaternion q_tf;
    q_tf.setEuler(yaw, pitch, roll);

    marker.pose.orientation.x = q_tf.getX();
    marker.pose.orientation.y = q_tf.getY();
    marker.pose.orientation.z = q_tf.getZ();
    marker.pose.orientation.w = q_tf.getW();

    // Set the scale of the marker
    marker.scale.x = 0.2; // Arrow length
    marker.scale.y = 0.03; // Arrow width
    marker.scale.z = 0.03; // Arrow height

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

visualization_msgs::Marker make_april_marker(int marker_id, uint8_t action, double x, double y)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = veh_name;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "multislam";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::CUBE;

    // Both ADD and MODIFY have the same value
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;

    // Set the scale of the marker
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.2;

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

class GraphSlam
{
    ros::NodeHandle nh_;
    ros::Subscriber apriltagsSub;
    ros::Subscriber carcmdSub;
    ros::Subscriber odomSub;
    ros::Timer optimizerTimer;
    NonlinearFactorGraph graph;
    Values initialEstimate;
    Values result;
    noiseModel::Diagonal::shared_ptr measurementNoise;
    noiseModel::Diagonal::shared_ptr odomNoise;
    set<int> tagsSeen;

    int curposeindex;
    float curx;
    float cury;
    float curtheta;
    float imutheta;
    float oldimutheta;
    double lastTimeSecs;

public:
    GraphSlam()
	: nh_(), measurementNoise(noiseModel::Diagonal::Sigmas((Vector(2) << 0.1, 0.1))),
	  odomNoise(noiseModel::Diagonal::Sigmas((Vector(3) << 0.05, 0.01, 0.1))),
	  curposeindex(0), curx(0.0f), cury(0.0f), curtheta(0.0f), imutheta(0.0f), oldimutheta(0.0f)
	{
	    nh_.getParam("duckiebot_visualizer/veh_name", veh_name);

	    lastTimeSecs = ros::Time::now().toSec();

	    // Prior on the first pose, set at the origin
	    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.0001, 0.0001, 0.0001));
	    graph.add(PriorFactor<Pose2>(0, Pose2(0, 0, 0), priorNoise));
	    initialEstimate.insert(0, Pose2(0.0, 0.0, 0.0));

	    // Publishers
	    marker_pub = nh_.advertise<visualization_msgs::Marker>("graph_visualization", 1);
	    marker_arr_pub = nh_.advertise<visualization_msgs::MarkerArray>("graph_visualization_arr", 1);

	    // Subscribers
	    apriltagsSub = nh_.subscribe("apriltags_postprocessing_node/apriltags_out", 1000, &GraphSlam::aprilcallback, this);
	    odomSub = nh_.subscribe("mono_odometer/odometry", 1000, &GraphSlam::odomCallback, this);

	    optimizerTimer = nh_.createTimer(ros::Duration(1), &GraphSlam::optimizeCallback, this);
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

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
	{
	    curposeindex += 1;

	    double timeNowSecs = ros::Time::now().toSec();
	    double delta_t = timeNowSecs - lastTimeSecs;
	    lastTimeSecs = timeNowSecs;

	    // Make sure that you set the speed gain *on the bot* to CURRENT_SPEEDGAIN
	    // rosparam set /<veh>/joy_mapper_node/speed_gain CURRENT_SPEEDGAIN
	    const double CURRENT_SPEEDGAIN = 0.41;
	    double delta_d = delta_t * CURRENT_SPEEDGAIN;

	    if (msg->twist.twist.linear.x == 0.0)
		delta_d = 0.0;

	    tf::Quaternion q_tf(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
				msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	    tf::Matrix3x3 m(q_tf);
	    double roll, pitch, yaw;
	    m.getRPY(roll, pitch, yaw);

	    double old_theta = curtheta;
	    curtheta = yaw;
	    graph.add(BetweenFactor<Pose2>(curposeindex-1,curposeindex,
					   Pose2(delta_d, 0, fmod(curtheta - old_theta, 2 * M_PI)), odomNoise));
	    curx += cos(curtheta) * delta_d;
	    cury += sin(curtheta) * delta_d;
	    initialEstimate.insert(curposeindex, Pose2(curx, cury, curtheta));

	    marker_pub.publish(make_pose_marker(curposeindex, ADD_ACTION, curx, cury, curtheta));
	}


    void optimizeCallback(const ros::TimerEvent&)
	{
	    visualization_msgs::MarkerArray ma;
	    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
	    result = optimizer.optimize();

	    BOOST_FOREACH(const Values::KeyValuePair& key_val, result)
	    {
		visualization_msgs::Marker marker;
		if (typeid(key_val.value) == typeid(Pose2))
		{
	    const Pose2 &val = dynamic_cast<const Pose2&>(key_val.value);
		    marker = make_pose_marker(key_val.key, OPTIMIZE_ACTION,
					      val.x(), val.y(), val.theta());
		} else if (typeid(key_val.value) == typeid(Point2)) {
		    const Point2 &val = dynamic_cast<const Point2&>(key_val.value);
		    marker = make_april_marker(key_val.key, OPTIMIZE_ACTION,
					       val.x(), val.y());
		} else {
		    ROS_WARN_ONCE("optimizeCallback: Unknown value type");
		}

		ma.markers.push_back(marker);
	    }

	    marker_arr_pub.publish(ma);
	}

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam");

    GraphSlam gs;

    ros::spin();
    return 0;
}
