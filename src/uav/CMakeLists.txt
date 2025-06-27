cmake_minimum_required(VERSION 2.8.3)

project(uav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  mavros_msgs
  std_msgs)

catkin_package(
	 CATKIN_DEPENDS 
	 roscpp 
	 rospy 
	 std_msgs 
	 mavros_msgs 
	 sensor_msgs
	 geometry_msgs
)

find_package(roslaunch)

catkin_install_python(PROGRAMS
  scripts/imu_monitor.py
  scripts/takeoff.py
  scripts/circle_traj.py
  scripts/figure8.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
