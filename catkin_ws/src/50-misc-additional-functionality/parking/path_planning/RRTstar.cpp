#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


// Our collision checker. Our robot's state space
// lies in [0,1]x[0,1]x[0,2*pi], with parking lane lines represented as obstacles .
// Any states lying in the lane line regions are
// considered "in collision".


class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
            ob::StateValidityChecker(si) {}
    // Returns whether the given state's position overlaps a
    // lane line
    bool isValid(const ob::State* state) const
    {
        return this->clearance(state) > 0.0;
    }
    // Returns the distance from the given state's position to the
    // boundary of the lane line obstacle.
    double clearance(const ob::State* state) const
    {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ob::SE2StateSpace::StateType* state3D =
                state->as<ob::SE2StateSpace::StateType>();
        // Extract the robot's (x,y,theta) position from its state

        double x = state3D->getX();
        double y = state3D->getY();
        //double theta = state3D->getYaw();

        // build lane rectangles: {min_x, max_x, min_y, max_y} in mm
        double line_one[4] = {900,1170,0,25};
        double line_two[4] = {900,1170,280,305};

        // dx = max(min_x-x, 0 ,x-max_x)
        // dy = max(min_y-y, 0, y-max_y)
        // dist = sqrt(dx*dx+dy*dy)
        double base = 0;

        double dx_one_temp =  std::max(line_one[0]-x,base);
        double dx_one = std::max(dx_one_temp,x-line_one[1]);
        double dy_one_temp =  std::max(line_one[2]-y,base);
        double dy_one = std::max(dy_one_temp,y-line_one[3]);
        double dist_one = sqrt(dx_one*dx_one+dy_one*dy_one);

        double dx_two_temp =  std::max(line_two[0]-x,base);
        double dx_two = std::max(dx_two_temp,x-line_two[1]);
        double dy_two_temp =  std::max(line_two[2]-x,base);
        double dy_two = std::max(dy_two_temp,x-line_two[3]);
        double dist_two = sqrt(dx_two*dx_two+dy_two*dy_two);

        double r = 60; // robot "radius" in mm
        // return the minimum clearance minus the robot radius
        return std::min(dist_one,dist_two)-r;
    }

};


int main() {

// Construct the robot state space in which we're planning. We're
// planning in [0,1.17]x[0,1.17]x[0,2*pi]
    ob::StateSpacePtr space(new ob::SE2StateSpace());

// Set the bounds of space to be in [0,1.17], [0,1.17], [0,2*pi].
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0,0.0);
    bounds.setHigh(0,1170);
    bounds.setLow(1,0.0);
    bounds.setHigh(1,1170);
    bounds.setLow(2,0.0);
    bounds.setHigh(2,2*3.1415926535897);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);

// Construct a space information instance for this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

// Set the object used to check which states in the space are valid
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
    si->setup();

// Set our robot's starting state to be (0,0,0).
    ob::ScopedState<> start(space);
    start->as<ob::SE2StateSpace::StateType>()->setX(0.0);
    start->as<ob::SE2StateSpace::StateType>()->setY(0.0);
    start->as<ob::SE2StateSpace::StateType>()->setYaw(0.0);

// Set our robot's goal state to be the top-right corner of the
// environment, or (1.17,1.17,0).
    ob::ScopedState<> goal(space);
    goal->as<ob::SE2StateSpace::StateType>()->setX(1.0);
    goal->as<ob::SE2StateSpace::StateType>()->setY(1.0);
    goal->as<ob::SE2StateSpace::StateType>()->setYaw(0.0);

// Create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

// Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

// Construct our optimizing planner using the RRTstar algorithm.
    ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));

// Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

// attempt to solve the planning problem within one second of
// planning time
    ob::PlannerStatus solved = optimizingPlanner->solve(1.0);

    return 0;
}