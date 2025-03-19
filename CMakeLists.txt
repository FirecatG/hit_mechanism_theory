cmake_minimum_required(VERSION 2.8.3)

project(hit_mechanism_theory)
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
add_executable(calculate_only 
	src/calculate_only.cpp
	src/four_bar_mechanism.cpp)
add_executable(publish_per_30_degree
	src/publish_per_30_degree.cpp
	src/four_bar_mechanism.cpp
)

target_link_libraries(main ${catkin_LIBRARIES})
target_link_libraries(calculate_only ${catkin_LIBRARIES})
target_link_libraries(publish_per_30_degree ${catkin_LIBRARIES})

include_directories(include)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
