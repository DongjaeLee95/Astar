cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 14)

project(astar)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        geometry_msgs
        nav_msgs
        tf
        visualization_msgs
        )

catkin_package(
        DEPENDS Eigen3
        INCLUDE_DIRS include
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp src/Astar.cpp src/OccMapTransform.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(astar_dj src/main_woMap.cpp src/AstarWoMap.cpp src/OccMapTransform.cpp)
target_link_libraries(astar_dj Eigen3::Eigen ${catkin_LIBRARIES} ${OpenCV_LIBS} ${EIGEN_LIBRARIES})

add_executable(obs_visualize src/main_visualization.cpp)
target_link_libraries(obs_visualize Eigen3::Eigen ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

add_executable(publish_simul src/main_publishSimul.cpp)
target_link_libraries(publish_simul Eigen3::Eigen ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})