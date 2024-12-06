///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Mihir
//////////////////////////////////////

#ifndef RG_RRT_H
#define RG_RRT_H

#include <ompl/control/SpaceInformation.h>
#include <ompl/control/StatePropagator.h> 
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/geometric/PathGeometric.h> 
#include <ompl/base/StateSampler.h>
#include <ompl/base/State.h> 
#include <vector>
#include <memory>

namespace ompl
{
    namespace control
    {
        class RGRRT : public ompl::base::Planner
        {
        public:            
            RGRRT(const ompl::control::SpaceInformationPtr &si);           
            ~RGRRT() override;

           
            ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;            
            void clear() override;            
            void setup() override;            
            void setRange(double range);            
            double getRange() const;            
            void setMaxReachableStates(unsigned int maxStates);            
            unsigned int getMaxReachableStates() const;
        protected:            
            void freeMemory();
            
            double range_;            
            unsigned int maxReachableStates_;            
            std::shared_ptr<ompl::base::StateSampler> sampler_;
            
            struct Motion
            {
                ompl::base::State *state;
                Motion *parent;
                std::vector<ompl::base::State *> reachableSet;
            };

            
            std::vector<Motion *> motions_;
        };
    } 
} 

#endif  RG_RRT_H
