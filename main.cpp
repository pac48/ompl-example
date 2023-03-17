#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <iostream>
#include <utility>
#include "ompl/src/ompl/base/spaces/SE3StateSpace.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;


class RobotJointState : public ob::State {
public:
    std::vector<double> q;

    RobotJointState() = delete;
    RobotJointState(int numJoints) {
        q.assign(numJoints, 0);
    };

};

class RobotStateSampler : public ob::StateSampler {
public:
    using StateSampler::StateSampler;

    void sampleUniform(ompl::base::State *state) override {
        constexpr double pi = 3.14;

        auto robotState = state->as<RobotJointState>();
        for (auto &q: robotState->q) {
            q = rng_.uniformReal(-pi, pi);
        }

        space_->enforceBounds(state);
    }

    void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override {
        auto robotState = state->as<RobotJointState>();
        for (auto &q: robotState->q) {
            q = rng_.uniformReal(q - distance / 2.0, q + distance / 2.0);
        }

        space_->enforceBounds(state);
    }

    void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override {
        auto *robotState = state->as<RobotJointState>();
        auto *robotMean = mean->as<RobotJointState>();
        for (auto i = 0ul; i < robotMean->q.size(); i++) {
            auto &q = robotState->q[i];
            q = rng_.gaussian(robotMean->q[i], stdDev);
        }
        space_->enforceBounds(state);
    }

};

class RobotStateSpace : public ob::StateSpace {
public:
    int numJoints;
    ob::RealVectorBounds bounds;

    RobotStateSpace() = delete;

    RobotStateSpace(int numJointsIn, ob::RealVectorBounds boundsIn) : numJoints{numJointsIn},
                                                                      bounds{std::move(boundsIn)} {}


    unsigned int getDimension() const override {
        return numJoints;
    }

    double getMaximumExtent() const override {
        double m = 0;
        for (auto i = 0ul; i < bounds.high.size(); i++) {
            m = std::max(m, (bounds.high[i] - bounds.low[i]));
        }

        return m;

    }

    double getMeasure() const override {
        double m = 1;
        for (auto i = 0ul; i < bounds.high.size(); i++) {
            m *= (bounds.high[i] - bounds.low[i]);
        }

        return m;
    }

    void enforceBounds(ompl::base::State *state) const override {
        auto robotState = state->as<RobotJointState>();
        for (auto i = 0ul; i < robotState->q.size(); i++) {
            auto &q = robotState->q[i];
            q = std::max(q, bounds.low[i]);
            q = std::min(q, bounds.high[i]);
        }
    }

    bool satisfiesBounds(const ompl::base::State *state) const override {
        auto robotState = state->as<RobotJointState>();
        for (auto i = 0ul; i < robotState->q.size(); i++) {
            const auto &q = robotState->q[i];
            if (q < bounds.low[i] || q > bounds.high[i]) {
                return false;
            }
        }
        return true;
    }

    void copyState(ompl::base::State *destination, const ompl::base::State *source) const override {
        (destination->as<RobotJointState>())->q = (source->as<RobotJointState>())->q;
    }

    double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override {
        auto robotState1 = state1->as<RobotJointState>();
        auto robotState2 = state2->as<RobotJointState>();
        double dist = 0;
        for (auto i = 0ul; i < robotState1->q.size(); i++) {
            double tmp = (robotState1->q[i] - robotState2->q[i]);
            dist += tmp * tmp;
        }
        return std::sqrt(dist);
    }

    bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const override {
        auto robotState1 = state1->as<RobotJointState>();
        auto robotState2 = state2->as<RobotJointState>();
        return robotState1->q == robotState2->q;
    }

    void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t,
                     ompl::base::State *state) const override {
        auto robotStateFrom = from->as<RobotJointState>();
        auto robotStateTo = to->as<RobotJointState>();
        auto robotState = state->as<RobotJointState>();
        for (auto i = 0ul; i < robotStateFrom->q.size(); i++) {
            robotState->q[i] = (1 - t) * robotStateFrom->q[i] + t * robotStateTo->q[i];
        }
    }

    ompl::base::StateSamplerPtr allocDefaultStateSampler() const override {
        return std::make_shared<RobotStateSampler>((ob::StateSpace *) this);
    }

    ompl::base::State *allocState() const override {
        return new RobotJointState(bounds.low.size());
    }

    void freeState(ompl::base::State *state) const override {
        delete state->as<RobotJointState>();
    }

};


class ValidityChecker : public ob::StateValidityChecker {
public:
    bool isValid(const ompl::base::State *state) const override {
        const auto *pos = state->as<ob::SE2StateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0);
        return !(pos->values[0] > 5 && pos->values[1] > 5);  // avoid collision
    }

    ValidityChecker(std::shared_ptr<ompl::base::SpaceInformation> &si) : StateValidityChecker(si) {

    }


};

bool isStateValid(const ob::State *state) {
    // this needs to check if the state is valid after already checking bounds. This is for scenes objects and such
    auto robotState = state->as<RobotJointState>();

    for (auto i = 0ul; i < robotState->q.size(); i++) {
        const auto &q = robotState->q[i];
        if (q < 2.0 &&  q > -2.0) {
            return false;
        }
    }
    return true;
}


int main() {
    // create the state space for 2D space
    ob::RealVectorBounds bounds0(7);
    bounds0.setLow(-3.14);
    bounds0.setHigh(3.14);
    auto space0(std::make_shared<RobotStateSpace>(7, bounds0));

    og::SimpleSetup ss(space0);
    ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });

    auto startState = RobotJointState(7);
    startState.q.assign(7, -2.1);
    ob::ScopedState<RobotStateSpace> start0(space0);
    start0 = startState;

    auto goalState = RobotJointState(7);
    goalState.q.assign(7, 2.1);
    ob::ScopedState<RobotStateSpace> goal0(space0);
    goal0 = goalState;

    ss.setStartAndGoalStates(start0, goal0);

    ob::PlannerStatus solved0 = ss.solve(1.0);
    if (solved0) {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    }

    return 0;


    auto space(std::make_shared<ob::SE2StateSpace>());

    // create the state validity checker for collision detection
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->setBounds(bounds);


    // create the space information
    auto si = std::make_shared<ob::SpaceInformation>(space);
    auto svChecker = std::make_shared<ValidityChecker>(si);

    si->setStateValidityChecker(svChecker);
    si->setStateValidityCheckingResolution(0.03); // 3%
    si->setup();

    // create the start and goal states
    ob::ScopedState<ob::SE2StateSpace> start(space), goal(space);
    start->setXY(-5, -5);
    start->setYaw(0);
    goal->setXY(5, 5);

    // create the problem definition
    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);

    // create the planner and set it up
    auto planner(std::make_shared<og::RRTConnect>(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    // solve the problem
    auto solved(pdef->hasApproximateSolution());
    if (solved) {
        std::cout << "Found approximate solution in " << pdef->getSolutionPath()->length() << " seconds\n";
    } else {
        std::cout << "No approximate solution found\n";
    }

    return 0;
}
