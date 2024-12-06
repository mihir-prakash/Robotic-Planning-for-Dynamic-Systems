///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Mihir 
//////////////////////////////////////

//Importing header files

#include <iostream>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <fstream>
#include <ompl/base/Path.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include "RG-RRT.h"
#include <cmath>
#include <vector>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/Planner.h>
#include <ompl/control/SpaceInformation.h>

namespace {
    const double g = 9.81; 
}

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    { //ProjectionEvaluator class would define  projection of the pendulum's state space to a lower-dimensional space. (Useful for KPIECE1)
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the pendulum
        
        return 2; //we will reduce the dimension to a 2d plane (theta, omega) for KPIECE1
    }

    void project(const ompl::base::State * state , Eigen::Ref<Eigen::VectorXd>  projection ) const override
    {
       // TODO: Your projection for the pendulum
       //extracting theta and omega and setting them in projection vector
        const auto *pendulumState = state->as<ompl::base::RealVectorStateSpace::StateType>();
        projection[0] = pendulumState->values[0]; // this gives theta
         projection[1] = pendulumState->values[1]; // this gives angular velocity omega
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType & q , const ompl::control::Control * control,
                ompl::control::ODESolver::StateType & qdot )
{
    // TODO: Fill in the ODE for the pendulum's dynamics
    
        // Extracting the state variables
        double theta = q[0];  // Pendulum angle
        double omega = q[1];  // Angular velocity

        // Extracting the control input (torque)
        const auto *u = control->as<ompl::control::RealVectorControlSpace::ControlType>();
        double tau = u->values[0];  // this would give us Torque

        // Computing our derivatives
        qdot[0] = omega;                // d(theta)/dt = omega
        qdot[1] = -g * std::sin(theta) + tau;  // d(omega)/dt = -g*cos(theta) + tau



}
// Creating A Path visualizing function
void visualizeSolutionPathAsSVG(ompl::control::SimpleSetupPtr &ss, const std::string &filename) {
    if (ss->haveSolutionPath()) {
        // Extracting the solution path
        auto path = ss->getSolutionPath().asGeometric();
        std::ofstream svgFile(filename);

        if (svgFile.is_open()) {
            // Writing the SVG header
            svgFile << "<svg xmlns='http://www.w3.org/2000/svg' width='500' height='500' viewBox='-10 -10 20 20'>\n";
            svgFile << "<g transform='scale(1, -1)'>\n";

            // Writing the path points
            svgFile << "<polyline points='";
            for (size_t i = 0; i < path.getStateCount(); ++i) {
                const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
                svgFile << state->values[0] << "," << state->values[1] << " ";
            }
            svgFile << "' style='fill:none;stroke:blue;stroke-width:0.1' />\n";

            // Writing the SVG footer
            svgFile << "</g>\n</svg>";
            svgFile.close();

            std::cout << "Solution path saved to " << filename << std::endl;
        } else {
            std::cerr << "Failed to open file: " << filename << std::endl;
        }
    } else {
        std::cerr << "No solution path found to visualize." << std::endl;
    }
}



ompl::control::SimpleSetupPtr createPendulum(double  torque)
{
    // TODO: Create and setup the pendulum's state space, control space, validity checker, everything you need for
    // planning.
    
    //  state space
    auto stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(2); // State space: (theta, omega)

    // Setting bounds
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, -M_PI);  //thetaa
    bounds.setHigh(0, M_PI);  //theta
    bounds.setLow(1, -10.0);  //omega
    bounds.setHigh(1, 10.0);  //omega
    stateSpace->setBounds(bounds);

    //control space
    auto controlSpace = std::make_shared<ompl::control::RealVectorControlSpace>(stateSpace, 1); 
    ompl::base::RealVectorBounds controlBounds(1);
    controlBounds.setLow(-torque);  
    controlBounds.setHigh(torque);  
    controlSpace->setBounds(controlBounds);

    //Initializing rhe SimpleSetup object
    auto ss = std::make_shared<ompl::control::SimpleSetup>(controlSpace);

    //ODE for the pendulum
    auto odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), pendulumODE);
    ss->getSpaceInformation()->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));

    //state validity checker
    ss->setStateValidityChecker([](const ompl::base::State *state) {
        const auto *pendulumState = state->as<ompl::base::RealVectorStateSpace::StateType>();
        double omega = pendulumState->values[1]; 
        double theta = pendulumState->values[0];
        return std::abs(omega) <= 10.0 && std::abs(theta) <= M_PI;; 
    });

    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr &ss, int choice) {   
    // TODO: Fill in the ODE for the pendulum's dynamics
    if (choice == 1) {
        ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    } else if (choice == 2) {
        ss->setPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
    } else if (choice == 3) {
        ss->setPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));
    } else {
        std::cerr << "Invalid choice! Select a Valid Planner" << std::endl;
        return;
    }

    ompl::base::ScopedState<> start(ss->getStateSpace());
    start[0] = -M_PI / 2; // Start theta
    start[1] = 0;         // Start omega

    ompl::base::ScopedState<> goal(ss->getStateSpace());
    goal[0] = M_PI / 2;   // Goal theta
    goal[1] = 0;          // Goal omega

    ss->setStartAndGoalStates(start, goal, 0.1);

    if (ss->solve(10.0)) {
        std::cout << "Yayy!! We found the Solution!" << std::endl;
        ss->getSolutionPath().asGeometric().print(std::cout);

        // Visualizing and saving the solution as SVG file
        visualizeSolutionPathAsSVG(ss, "pendulum_solution.svg");
    } else {
        std::cout << "No solution found." << std::endl;
    }
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &ss)
{
    // Define the start state
    ompl::base::ScopedState<> start(ss->getStateSpace());
    start[0] = -M_PI / 2; // Start theta
    start[1] = 0;         // Start omega

    // Define the goal state
    ompl::base::ScopedState<> goal(ss->getStateSpace());
    goal[0] = M_PI / 2;   // Goal theta
    goal[1] = 0;          // Goal omega

    // Set start and goal states with a threshold tolerance
    ss->setStartAndGoalStates(start, goal, 0.1);

    // Create a benchmark object
    ompl::tools::Benchmark benchmark(*ss, "Pendulum Benchmark");

    // Add planners to benchmark
    benchmark.addPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    benchmark.addPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
    benchmark.addPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));

    // Configure benchmark request
    ompl::tools::Benchmark::Request request;
    request.maxTime = 10.0;     // Maximum time for each planner
    request.maxMem = 1000.0;    // Maximum memory usage in MB
    request.runCount = 5;       // Number of runs for each planner
    request.displayProgress = true;

    // Perform benchmarking
    benchmark.benchmark(request);

    // Save results to a log file
    if (benchmark.saveResultsToFile("pendulum_benchmark.log")) {
        std::cout << "Benchmark completed. Results saved to pendulum_benchmark.log" << std::endl;
    } else {
        std::cerr << "Failed to save benchmark results." << std::endl;
    }
}

int main(int argc , char **  argv )
{
    int choice;
    do
    {
        std::cout << "Do you want to Plan or Benchmark? (Press relevant key) " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Select the Torque: " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

    
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner would you like to use? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planPendulum(ss, planner);
    }
    
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "Oops...Something went wrong. Invalid choice." << std::endl;

    return 0;
}
