cmake_minimum_required(VERSION 2.8.3)

project(raceworld_master)

find_package(catkin REQUIRED)


find_package(roslaunch)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  message_generation
  visualization_msgs
  cv_bridge_4
  image_transport
)
find_package(OpenCV REQUIRED)
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
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

catkin_install_python(PROGRAMS

  scripts/0final/singleCar_changePath/running_car.py
  scripts/0final/singleCar_changePath/running_car2.py
  scripts/0final/multicars_follow/leader.py
  scripts/0final/multicars_follow/follower1.py
  scripts/0final/multicars_follow/follower2.py
  scripts/0final/multicars_follow/follower3.py

  scripts/camera/get_camera.py
  scripts/control/key_op.py
  scripts/control/servo_commands1.py
  scripts/control/follow.py
  scripts/control/follow2.py
  scripts/control/control_servo.py
  scripts/tag_locate/tag_detect.py
  scripts/tag_locate/change_path1.py
  scripts/tag_locate/change_path2.py
  scripts/tag_locate/loc_analyse.py
  scripts/words_detection/words_detection.py
  scripts/words_detection/detection_and_follow.py
  scripts/words_detection/test_stop.py
  scripts/multi_cars/lane_master.py
  scripts/multi_cars/servo_commands.py
  scripts/status_led_pkg/status_led_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(leader_master scripts/multi_cars/platoon/leader_master.cpp)
target_link_libraries(leader_master ${catkin_LIBRARIES})
#add_dependencies(master stage_first_generate_messages_cpp)

#add_executable(slave scripts/platoon/slave.cpp)
#target_link_libraries(slave ${catkin_LIBRARIES})

add_executable(main_master scripts/multi_cars/platoon/main_master.cpp)
target_link_libraries(main_master ${catkin_LIBRARIES})
add_dependencies(main ${PROJECT_NAME}_generate_messages_cpp)

add_executable(pure_master scripts/multi_cars/platoon/pure_master.cpp)
target_link_libraries(pure_master ${catkin_LIBRARIES})

add_executable(camera scripts/camera.cpp)
target_link_libraries(camera ${catkin_LIBRARIES} ${OpenCV_LIBS})