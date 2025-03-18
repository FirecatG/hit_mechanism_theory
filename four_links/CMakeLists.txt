cmake_minimum_required(VERSION 2.8.3)

project(four_links)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs tf)
include_directories(${catkin_INCLUDE_DIRS})
find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)
include_directories(include)
add_executable(main 
	src/main.cpp
	src/four_bar_mechanism.cpp)
target_link_libraries(main ${catkin_LIBRARIES})

include_directories(include)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
