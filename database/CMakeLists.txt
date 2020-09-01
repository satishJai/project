cmake_minimum_required(VERSION 2.8.3)
project(database)

add_compile_options ("-Wall" "-std=c++14")

list (APPEND CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})
include (CommonMacros)

require_custom_package (PCL 1)
#find_package (PCL)
add_definitions (${PCL_DEFINITIONS})
include_directories (${PCL_INCLUDE_DIRS})

find_package(catkin REQUIRED roscpp)

catkin_package(
  LIBRARIES postgresql_database
  CATKIN_DEPENDS roscpp
)

find_package (catkin REQUIRED COMPONENTS
	roscpp
	handling_msgs
	pcl_ros
	pcl_conversions

)

catkin_package(CATKIN_DEPENDS
	roscpp
	handling_msgs
	pcl_ros
	pcl_conversions

)

include_directories (
	${PROJECT_SOURCE_DIR}/include
	${catkin_INCLUDE_DIRS}
)

add_library (database
	src/Database.cpp
)
target_link_libraries (database soci_core ${PCL_LIBRARIES})



add_executable (main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp database)
