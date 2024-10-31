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

# Find ROS packages and Threads for pthread support
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_srvs
    pcl_ros
    pcl_conversions
)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED
find_package(PCL REQUIRED)
)

# catkin specific configuration
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_srvs pcl_ros pcl_conversions
)

# Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# State Machine
add_executable(${PROJECT_NAME} src/main.cpp src/state_machine.cpp)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    Threads::Threads  # Link pthread here
    yaml-cpp
    ${PCL_LIBRARIES}
)


# PCD publisher
add_executable(pcd_publisher src/pcd_publisher.cpp) 
target_link_libraries(pcd_publisher
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    yaml-cpp
)