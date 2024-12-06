///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Mihir
//////////////////////////////////////

#include <iostream>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <fstream>
// The collision checker routines
#include "CollisionChecking.h"
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/tools/benchmark/Benchmark.h>
// Your implementation of RG-RRT
#include "RG-RRT.h"

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        //we will project the car's state to a 2D plane.
        return 2;
    }

    void project(const ompl::base::State * state , Eigen::Ref<Eigen::VectorXd> projection ) const override
    {
        // TODO: Your projection for the car
        const auto *carState = state->as<ompl::base::RealVectorStateSpace::StateType>();
        projection[0] = carState->values[0]; // x
        projection[1] = carState->values[1]; // y
        
    }
};

void carODE(const ompl::control::ODESolver::StateType &q , const ompl::control::Control *control,
            ompl::control::ODESolver::StateType &qdot)
{
    // TODO: Fill in the ODE for the car's dynamics
    double x = q[0];    
    double y = q[1];   
    double theta = q[2]; // theta is the heading angle of car

    const auto *u = control->as<ompl::control::RealVectorControlSpace::ControlType>();
    double v = u->values[0];     
    double omega = u->values[1]; 

    // ODE equations
    qdot[0] = v * std::cos(theta); // dx/dt
    qdot[1] = v * std::sin(theta); // dy/dt
    qdot[2] = omega;               // dtheta/dt

}

void makeStreet(std::vector<Rectangle>&obstacles)
{
    // TODO: Fill in the vector of rectangles with your street environment.
    //rectangles-- obsctacles
    obstacles.push_back(Rectangle{-5, -5, 2, 2}); 
    obstacles.push_back(Rectangle{3, 3, 4, 1});   
    obstacles.push_back(Rectangle{0, -2, 2, 5});  
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> & obstacles)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    
    //state space
    auto stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(3); 

    //set bounds
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, -10); // x 
    bounds.setHigh(0, 10); // x
    bounds.setLow(1, -10); // y 
    bounds.setHigh(1, 10); // y 
    bounds.setLow(2, -M_PI); // theta 
    bounds.setHigh(2, M_PI); // theta
    stateSpace->setBounds(bounds);

    //cpntrol space
    auto controlSpace = std::make_shared<ompl::control::RealVectorControlSpace>(stateSpace, 2);

    ompl::base::RealVectorBounds controlBounds(2);
    controlBounds.setLow(0, -1);  // v 
    controlBounds.setHigh(0, 1);  // v 
    controlBounds.setLow(1, -1);  // omega 
    controlBounds.setHigh(1, 1);  // omega 
    controlSpace->setBounds(controlBounds);

    auto ss = std::make_shared<ompl::control::SimpleSetup>(controlSpace);

    // ODE solver
    auto odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), carODE);
    ss->getSpaceInformation()->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));

    //collision checking
    ss->setStateValidityChecker([&obstacles](const ompl::base::State *state) {
    const auto *carState = state->as<ompl::base::RealVectorStateSpace::StateType>();
    double x = carState->values[0];
    double y = carState->values[1];
    double theta = carState->values[2];
    double sideLength = 1.0; 
    return isValidStateSquare(state, sideLength, obstacles);
});

    return ss;
}

//Function to visualize the solution path as an SVG file

void visualizeSolutionPathAsSVG(ompl::control::SimpleSetupPtr &ss, const std::string &filename) {
    if (ss->haveSolutionPath()) {
        // Extracting the solution path
        auto path = ss->getSolutionPath().asGeometric();
        std::ofstream svgFile(filename);

        if (svgFile.is_open()) {
            //SVG header
            svgFile << "<svg xmlns='http://www.w3.org/2000/svg' width='500' height='500' viewBox='-10 -10 20 20'>\n";
            svgFile << "<g transform='scale(1, -1)'>\n";

            //path points
            svgFile << "<polyline points='";
            for (size_t i = 0; i < path.getStateCount(); ++i) {
                const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
                svgFile << state->values[0] << "," << state->values[1] << " ";
            }
            svgFile << "' style='fill:none;stroke:blue;stroke-width:0.1' />\n";

            //SVG footer
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
void planCar(ompl::control::SimpleSetupPtr & ss, int choice )
{    
    if (choice == 1)
        ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    else if (choice == 2)
        ss->setPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
    else if (choice == 3)
        ss->setPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));
    else {
        std::cerr << "Oops, Invalid planner choice!" << std::endl;
        return;
    }

    // Defining the start state
    ompl::base::ScopedState<> start(ss->getStateSpace());
    start[0] = -8;  // x
    start[1] = -8;  // y
    start[2] = 0;   // theta

    //goal state
    ompl::base::ScopedState<> goal(ss->getStateSpace());
    goal[0] = 8;    
    goal[1] = 8;    
    goal[2] = 0; 
    
    ss->setStartAndGoalStates(start, goal, 0.1);    
    if (ss->solve(10.0)) {
        std::cout << "Yayyy....Solution found!" << std::endl;        
        ss->getSolutionPath().asGeometric().print(std::cout);        
        visualizeSolutionPathAsSVG(ss, "car_solution.svg");
    } else {
        std::cout << "Oops....No solution found." << std::endl;
    }

}

void benchmarkCar(ompl::control::SimpleSetupPtr &ss)
{
    // Define the start state
    ompl::base::ScopedState<> start(ss->getStateSpace());
    start[0] = -8;  // x
    start[1] = -8;  // y
    start[2] = 0;   // theta

    // Define the goal state
    ompl::base::ScopedState<> goal(ss->getStateSpace());
    goal[0] = 8;    // x
    goal[1] = 8;    // y
    goal[2] = 0;    // theta

    // Set start and goal states with a threshold tolerance
    ss->setStartAndGoalStates(start, goal, 0.1);

    // Create a benchmark object
    ompl::tools::Benchmark benchmark(*ss, "Car Benchmark");

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
    benchmark.saveResultsToFile("car_benchmark.log");

    std::cout << "Benchmark results saved to 'car_benchmark.log'" << std::endl;
}


int main(int argc, char ** argv)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Do you want to Plan or Benchmark? (Press relevant key) " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

    
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

        planCar(ss, planner);
    }
    
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "Oops...Something went wrong. Invalid choice." << std::endl;

    return 0;
}
