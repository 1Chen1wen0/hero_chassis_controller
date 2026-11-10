cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  realtime_tools
  roscpp
  roslint
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    control_toolbox
    controller_interface
    geometry_msgs
    hardware_interface
    nav_msgs
    pluginlib
    realtime_tools
    roscpp
    sensor_msgs
    tf2
    tf2_geometry_msgs
    tf2_msgs
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/hero_chassis_controller.cpp
  src/mecanum_kinematics.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
add_dependencies(keyboard_teleop ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyboard_teleop ${catkin_LIBRARIES})

add_executable(simple_navigator src/simple_navigator.cpp)
add_dependencies(simple_navigator ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_navigator ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} keyboard_teleop simple_navigator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch rviz worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES hero_chassis_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test
    test/test_main.cpp
    test/mecanum_kinematics_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_test)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
endif()

roslint_cpp()
