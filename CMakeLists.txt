cmake_minimum_required(VERSION 3.16)
project(PendulumPlanning)


set(CMAKE_CXX_STANDARD 17)

find_package(ompl REQUIRED)


include_directories(/opt/homebrew/Cellar/ompl/1.6.0_8/include/ompl-1.6)
include_directories(/opt/homebrew/Cellar/boost/1.86.0_2/include)
include_directories(/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3)
include_directories(/opt/homebrew/Cellar/ompl/1.6.0_8/include/ompl-1.6)
include_directories(/opt/homebrew/Cellar/ompl/1.6.0_8/include/ompl-1.6)

link_directories(/opt/homebrew/lib /opt/homebrew/Cellar/ompl/1.6.0_8/lib)


add_executable(pendulum_project src/Project4Pendulum.cpp src/RG-RRT.cpp)
# add_executable(car_project src/Project4Car.cpp src/CollisionChecking.cpp src/RG-RRT.cpp)

target_link_libraries(pendulum_project PRIVATE ompl)
# target_link_libraries(car_project PRIVATE ompl)



