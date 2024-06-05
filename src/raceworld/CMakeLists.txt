cmake_minimum_required(VERSION 2.8.3)

project(raceworld)

find_package(catkin REQUIRED)



find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  message_generation
)

add_message_files(
   FILES
   status.msg
#   Message2.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

 catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pkg
  CATKIN_DEPENDS roscpp rospy tf message_runtime
#  DEPENDS system_lib

)

catkin_install_python(PROGRAMS
  scripts/key_op.py
  scripts/servo_commands1.py
  scripts/servo_commands.py
  scripts/follow.py
  scripts/control_servo.py
  scripts/tag_detect.py
  scripts/lane.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(main scripts/platoon/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES})
add_dependencies(main ${PROJECT_NAME}_generate_messages_cpp)

add_executable(pure scripts/platoon/pure.cpp)
target_link_libraries(pure ${catkin_LIBRARIES})
