cmake_minimum_required(VERSION 3.0.2)
project(my_package)


find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(jsoncpp REQUIRED)




find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  tf
  jsoncpp
  message_generation
)





add_message_files(
  FILES
  Num.msg
)


 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
  INCLUDE_DIRS 
  "include/"
   CATKIN_DEPENDS roscpp std_msgs nav_msgs tf
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(my_package SHARED src/robot.cpp src/world.cpp src/lidar.cpp)
target_include_directories(my_package PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(my_package PUBLIC ${OpenCV_LIBRARIES} Eigen3::Eigen)



add_executable(test_node bin/mrsim_node.cpp src/robot.cpp src/world.cpp src/lidar.cpp)
add_executable(cmd_vel_pub bin/cmd_vel_pub_node.cpp)







target_include_directories(test_node PUBLIC 
    include 
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)


target_link_libraries(test_node PUBLIC 
    ${OpenCV_LIBRARIES} 
    Eigen3::Eigen
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    jsoncpp
)

target_link_libraries(cmd_vel_pub 
    ${OpenCV_LIBRARIES} 
    Eigen3::Eigen
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    jsoncpp
)


