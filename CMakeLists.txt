cmake_minimum_required(VERSION 3.0.2)
project(apple_picking_state_machine)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)

# Get the environment variable that holds the path to indicators
# set(INDICATORS_PATH $ENV{INDICATORS_PATH})

# if(INDICATORS_PATH)
#   include_directories(${INDICATORS_PATH}/include)
# else()
#   message(FATAL_ERROR "INDICATORS_PATH environment variable not set")
# endif()

## Find ROS packages and Threads for pthread support
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_srvs
)
find_package(Threads REQUIRED)

## catkin specific configuration
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_srvs
)

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare executables
add_executable(${PROJECT_NAME} src/main.cpp src/state_machine.cpp)
# add_executable(${PROJECT_NAME}_test src/progress_manager_test.cpp src/progress_manager.cpp)

## Specify libraries to link executables
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    Threads::Threads  # Link pthread here
)
