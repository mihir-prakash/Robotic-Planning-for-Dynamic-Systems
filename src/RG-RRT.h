///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Mihir
//////////////////////////////////////



#ifndef OMPL_CONTROL_PLANNERS_RGRRT_RGRRT_
#define OMPL_CONTROL_PLANNERS_RGRRT_RGRRT_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/Control.h>
#include <vector>
#include <limits>
#include <math.h>

namespace ompl
{
    namespace control
    {
        class RGRRT : public base::Planner
        {
        public:
            RGRRT(const SpaceInformationPtr &si);
            virtual ~RGRRT(void);
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);
            virtual void clear(void);
            
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }
            
            double getGoalBias(void) const
            {
                return goalBias_;
            }
            
            bool getIntermediateStates(void) const
            {
                return addIntermediateStates_;
            }
            
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }
            
            virtual void getPlannerData(base::PlannerData &data) const;
            
            template<template<typename T> class NN>
            void setNearestNeighbors(void)
            {
                nn_.reset(new NN<Motion*>());
            }
            
            virtual void setup(void);

        protected:
            class Motion
            {
            public:
                Motion(void) : state(NULL), control(NULL), steps(0), parent(NULL)
                {
                }
                
                Motion(const SpaceInformation *si) : state(si->allocState()), control(si->allocControl()), steps(0), parent(NULL)
                {
                }
                
                ~Motion(void)
                {
                }
                
                base::State       *state;
                Control          *control;
                unsigned int     steps;
                Motion          *parent;
                std::vector<Motion*> reachable;
            };
            
            void freeMemory(void);
            
            double distanceFunction(const Motion* a, const Motion* b) const
            {
                return si_->distance(a->state, b->state);
            }
            
            base::StateSamplerPtr sampler_;
            DirectedControlSamplerPtr controlSampler_;
            const SpaceInformation *siC_;
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;
            double goalBias_;
            bool addIntermediateStates_;
            RNG rng_;
            Motion *lastGoalMotion_;
            
            const int RSIZE = 15;
            std::vector<double> control_offset;
            
            void setupReachableSet(Motion* const m);
            int selectReachableMotion(const Motion* qnear, const Motion* qrand);
        };
    }
}

#endif