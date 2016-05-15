cmake_minimum_required(VERSION 2.8.3)
project(zahlil)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED gazebo_ros)
catkin_package()

add_executable(location_control src/location_control.cpp)
target_link_libraries(location_control ${catkin_LIBRARIES})
add_dependencies(location_control my_generate_messages_cpp)
