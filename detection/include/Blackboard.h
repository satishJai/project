/*
 * Blackboard.h
 *
 *  Created on: Nov 14, 2018
 *      Author: philipf
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <memory>

using PointType = pcl::PointXYZRGB;
using NormalType = pcl::Normal;
using RFType = pcl::ReferenceFrame;

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_BLACKBOARD_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_BLACKBOARD_H_

//Forward declaration:
class KeypointCreator;
class DescriptorCreator;
class SegmentsCreator;
class PoseEstimator;
class Matching;
class ResultEvaluation;
class Guesser;
class RosParameterManager;
template<typename T>
struct RosParameter;
struct Model;
struct Segment;

/*Blackboard stores all classes and some parameters which are currently used for the detection system. It also stores the pointclouds.*/
struct Blackboard
{
	Blackboard();
	//Pointer to keypoint and descriptor creator class. If descriptor will be changed, just let them point to the new type of creator.
	std::shared_ptr<KeypointCreator> keypointCreator;
	std::shared_ptr<DescriptorCreator> descriptorCreator;
	std::shared_ptr<SegmentsCreator> segmentation;
	std::shared_ptr<PoseEstimator> poseEstimator;
	std::shared_ptr<Matching> matching;
	std::shared_ptr<ResultEvaluation> resultEvaluation;
	std::shared_ptr<Guesser> guesser;

	static RosParameterManager parameterManager;
	static constexpr float RESOLUTION = 0.000680933;

	static std::shared_ptr<RosParameter<int>> normalRadius;
	static std::shared_ptr<RosParameter<double>> rfRadius;
	static std::shared_ptr<RosParameter<bool>> debugDescriptors;

	//Array of all learned models
	std::vector<Model> models;
	 //Array of all currently analysed segments
	std::vector<Segment> segments;
	 //Currently analyzed point cloud
	pcl::PointCloud<PointType>::Ptr inputCloud;
	 //Id of the latest detection run. Will be increased on every detection run.
	unsigned long counter;
};

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_BLACKBOARD_H_ */
