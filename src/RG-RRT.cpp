///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Mihir
//////////////////////////////////////


#include "RG-RRT.h"
// TODO: Implement RGRRT as described

#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/util/Exception.h>
#include <limits>
#include <ompl/control/spaces/RealVectorControlSpace.h>

using namespace ompl::control;


RGRRT::RGRRT(const SpaceInformationPtr &si) : ompl::base::Planner(si, "RG-RRT")
{
    range_ = 0.5;                 
    maxReachableStates_ = 10;     
    specs_.approximateSolutions = true;
    specs_.directed = true;

    declareParam<double>("range", this, &RGRRT::setRange, &RGRRT::getRange, "0.:1.:1000.");
}


RGRRT::~RGRRT()
{
    freeMemory();
}


void RGRRT::freeMemory()
{
    for (auto &motion : motions_)
    {
        if (motion->state)
        {
            si_->freeState(motion->state);
            motion->state = nullptr; 
        }

        for (auto &reachableState : motion->reachableSet)
        {
            si_->freeState(reachableState);
        }

        delete motion;
        motion = nullptr;
    }
    motions_.clear();
}


// Solve function
ompl::base::PlannerStatus RGRRT::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    if (!pdef_ || !pdef_->getGoal()) {
        OMPL_ERROR("Problem definition or goal is not defined.");
        return ompl::base::PlannerStatus::INVALID_START;
    }

    auto *goal = dynamic_cast<ompl::base::GoalSampleableRegion *>(pdef_->getGoal().get());
    if (!goal) {
        OMPL_ERROR("Goal undefined!!!!ERROR");
        return ompl::base::PlannerStatus::INVALID_GOAL;
    }

    auto *controlSI = static_cast<ompl::control::SpaceInformation *>(si_.get());
    if (!controlSI) {
        OMPL_ERROR("Invalid Control space information.");
        return ompl::base::PlannerStatus::INVALID_START;
    }

    if (!sampler_) {
        sampler_ = si_->allocStateSampler();
    }

    const ompl::base::State *startState = pis_.nextStart();
    if (!startState) {
        OMPL_ERROR("No valid start state is available.");
        return ompl::base::PlannerStatus::INVALID_START;
    }

    auto *startMotion = new Motion();
    startMotion->state = si_->cloneState(startState);
    motions_.push_back(startMotion);

    while (!ptc) {
        ompl::base::ScopedState<> q_rand(si_);
        sampler_->sampleUniform(q_rand.get());

        Motion *q_near = nullptr;
        double minDistance = std::numeric_limits<double>::infinity();
        for (auto &motion : motions_) {
            double dist = si_->distance(motion->state, q_rand.get());
            if (dist < minDistance) {
                minDistance = dist;
                q_near = motion;
            }
        }

        if (!q_near) {
            continue;
        }

        ompl::base::ScopedState<> q_new(si_);
        auto *control = controlSI->allocControl();
        auto *controlValues = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
        controlValues[0] = 0.0;

        controlSI->getStatePropagator()->propagate(q_near->state, control, 1.0, q_new.get());

        if (si_->isValid(q_new.get())) {
            auto *newMotion = new Motion();
            newMotion->state = si_->cloneState(q_new.get());
            newMotion->parent = q_near;
            motions_.push_back(newMotion);

            if (goal->isSatisfied(newMotion->state)) {
                auto path = std::make_shared<ompl::geometric::PathGeometric>(si_);
                std::unordered_set<Motion *> visited;
                for (Motion *m = newMotion; m != nullptr; m = m->parent) {
                    if (visited.count(m)) {
                        OMPL_ERROR("Cycle detected in the motion tree.");
                        break;
                    }
                    visited.insert(m);
                    path->append(m->state);
                }

                pdef_->addSolutionPath(path);
                controlSI->freeControl(control);
                return ompl::base::PlannerStatus::EXACT_SOLUTION;
            }
        }

        controlSI->freeControl(control);

        // Limiting the number of motions
        if (motions_.size() > 1000000) {
            OMPL_ERROR("Too many motions, terminating to prevent memory issues.");
            break;
        }
    }

    // Freeing any unprocessed states
    for (auto &motion : motions_) {
        if (motion->state) {
            si_->freeState(motion->state);
        }
        delete motion;
    }
    motions_.clear();

    return ompl::base::PlannerStatus::TIMEOUT;
}

void RGRRT::clear()
{
    Planner::clear();
    freeMemory();
}

void RGRRT::setRange(double range)
{
    range_ = range;
}

double RGRRT::getRange() const
{
    return range_;
}

void RGRRT::setMaxReachableStates(unsigned int maxStates)
{
    maxReachableStates_ = maxStates;
}

unsigned int RGRRT::getMaxReachableStates() const
{
    return maxReachableStates_;
}

void ompl::control::RGRRT::setup()
{
    // Calling the base class setup to ensure everything is initialized
    ompl::base::Planner::setup();

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_DEBUG("Yayy!!RGRRT setup complete.");
}
