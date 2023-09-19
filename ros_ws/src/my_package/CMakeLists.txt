cmake_minimum_required(VERSION 3.0.2)
project(my_package)

# add_compile_options(-std=c++11)


find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
#  genmsg
)

# include_directories(${catkin_INCLUDE_DIRS})
# find_package(Boost REQUIRED COMPONENTS system)


# catkin_python_setup()



# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

#  generate_messages(
#    DEPENDENCIES
#    std_msgs
#  )


# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
  INCLUDE_DIRS 
  "include/"
#  LIBRARIES my_package
   CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(my_package SHARED src/robot.cpp src/world.cpp src/lidar.cpp)
target_include_directories(my_package PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(my_package PUBLIC ${OpenCV_LIBRARIES} Eigen3::Eigen)


# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/my_package.cpp
# )


# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(test_node bin/mrsim_node.cpp)


# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")


# add_dependencies(robot ${robot_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_include_directories(test_node PUBLIC 
    include 
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# target_link_libraries(robot
#   ${catkin_LIBRARIES}
# )

target_link_libraries(test_node PUBLIC 
    ${OpenCV_LIBRARIES} 
    Eigen3::Eigen
    ${catkin_LIBRARIES}
)

# add_executable(robot src/robot.cpp)
#     target_include_directories(robot PUBLIC 
#         include 
#         ${catkin_INCLUDE_DIRS}
#         ${PCL_INCLUDE_DIRS}

#     )
#     target_link_libraries(robot 
#         robot
#         ${catkin_LIBRARIES}
#         ${PCL_LIBRARIES}
#     )



# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_package.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# catkin_add_nosetests(test)



