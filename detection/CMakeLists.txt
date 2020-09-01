cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(detection)

add_compile_options ("-Wall" "-std=c++14" "-fopenmp")

list (APPEND CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})
include (CommonMacros)

require_custom_package (PCL 1)
#find_package (PCL)
add_definitions (${PCL_DEFINITIONS})
include_directories (${PCL_INCLUDE_DIRS})

find_package (catkin REQUIRED COMPONENTS
	roscpp
	tf
	sensor_msgs
	std_msgs
	pcl_ros
	handling_msgs
	geometry_msgs
	tf_conversions
	operator_interface
)

catkin_package(CATKIN_DEPENDS
	roscpp
	tf
	sensor_msgs
	std_msgs
	pcl_ros
	handling_msgs
	geometry_msgs
	tf_conversions
)

find_package (Qt5 COMPONENTS Widgets REQUIRED)
find_package (VTK 6 REQUIRED)


include_directories (
	${PROJECT_SOURCE_DIR}/include
	${catkin_INCLUDE_DIRS}
)


add_library (detectionLib
	src/segmentation/Segmentation.cpp
	src/segmentation/SegmentsCreator.cpp
	src/descriptor/CShotCreator.cpp
	src/descriptor/ShotCreator.cpp
	src/descriptor/RSDCreator.cpp
	src/descriptor/FPFHCreator.cpp
	src/descriptor/ShapeContextCreator.cpp
	src/descriptor/UShapeContextCreator.cpp
	src/keypoint/ISSCreator.cpp
	src/keypoint/NDCreator.cpp
	src/keypoint/VoxelGridCreator.cpp
	src/matching/ColorMatching.cpp
	src/matching/KColorMatching.cpp
	src/matching/KSColorMatching.cpp
	src/matching/SimpleMatching.cpp
	src/pose_estimator/GCClustering.cpp
	src/pose_estimator/HoughClustering.cpp
	src/Control.cpp
	src/Blackboard.cpp
	src/evaluation/CorrespondenceVoting.cpp
	src/evaluation/Guesser.cpp
	src/visualization/visualization.cpp
	src/RosParameterManager.cpp
	src/ClassLoadManager.cpp
	src/evaluation/CorrespondenceVotingICP.cpp
)
target_link_libraries(detectionLib ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(main src/main.cpp )
target_link_libraries(main
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	detectionLib
)

add_executable(main2 src/main2.cpp )
target_link_libraries(main2
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	detectionLib
)

add_executable(evaluation src/evaluation.cpp )
target_link_libraries(evaluation
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	detectionLib
)

add_executable(evaluation2 src/evaluation2.cpp )
target_link_libraries(evaluation2
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	detectionLib
)

add_executable(detection src/detection.cpp )
target_link_libraries(detection
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	detectionLib
)

# ScannerWidget
add_widget (
    NAME          DetectionWidget
    INCLUDE_DIRS  ${catkin_INCLUDE_DIRS}
    LIBS          ${catkin_LIBRARIES} Qt5::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES}
    FILES        widgets/DetectionWidget.cpp
)

