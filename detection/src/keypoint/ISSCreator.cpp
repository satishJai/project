/*
 * ISSCreator.cpp
 *
 *  Created on: Nov 21, 2018
 *      Author: satishj
 */

#include "../include/keypoint/ISSCreator.h"

ISSCreator::ISSCreator()
{
	minNeighbors = RosParameterManager::createParam<int> (5,ParamType::INT,0,100,"/keypoints/iss/minNeighbors");
	salientRadius = RosParameterManager::createParam<double> (6,ParamType::DOUBLE,0.0,50.0,"/keypoints/iss/salientRadius");
	nonMaxRadius = RosParameterManager::createParam<double> (4,ParamType::DOUBLE,0.0,50.0,"/keypoints/iss/nonMaxRadius");
	threshold21 = RosParameterManager::createParam<double> (0.975,ParamType::DOUBLE,0.0,50.0,"/keypoints/iss/threshold21");
	threshold32 = RosParameterManager::createParam<double> (0.975,ParamType::DOUBLE,0.0,50.0,"/keypoints/iss/threshold32");
}
ISSCreator::~ISSCreator()
{
	RosParameterManager::deleteParam(minNeighbors);
	RosParameterManager::deleteParam(salientRadius);
	RosParameterManager::deleteParam(nonMaxRadius);
	RosParameterManager::deleteParam(threshold21);
	RosParameterManager::deleteParam(threshold32);
}

std::string ISSCreator::getName()
{
	return "ISSCreator";
}

void ISSCreator::run(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<PointType>::Ptr keypoints)
{
	pcl::ISSKeypoint3D<PointType, PointType> detector;

	detector.setInputCloud (cloud);
	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
	detector.setSearchMethod (tree);

	// Set the radius of the spherical neighborhood used to compute the scatter matrix.
	detector.setSalientRadius (salientRadius->var * radius->var * Blackboard::RESOLUTION);
	// Set the radius for the application of the non maxima supression algorithm.
	detector.setNonMaxRadius (nonMaxRadius->var * radius->var * Blackboard::RESOLUTION);
	// Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
	detector.setMinNeighbors (minNeighbors->var);
	// Set the upper bound on the ratio between the second and the first eigenvalue.
	detector.setThreshold21 (threshold21->var);
	// Set the upper bound on the ratio between the third and the second eigenvalue.
	detector.setThreshold32 (threshold32->var);
	// Set the number of prpcessing threads to use. 0 sets it to automatic.
	detector.setNumberOfThreads (0);
	detector.compute (*keypoints);
}
