cmake_minimum_required(VERSION 3.0.2)
project(track_follower)

add_compile_options(-std=c++17 -O3 -Wall)

find_package(catkin REQUIRED tf geometry_msgs visualization_msgs)
find_package(Eigen3 REQUIRED)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(fake_track_generator src/fake_track_generator.cpp)
target_link_libraries(fake_track_generator ${catkin_LIBRARIES})

add_executable(track_follower src/track_follower.cpp)
target_link_libraries(track_follower ${catkin_LIBRARIES})