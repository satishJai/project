/*
 * NDCreator.cpp
 *
 *  Created on: Nov 21, 2018
 *      Author: satishj
 */

#include "../include/keypoint/NDCreator.h"

void NDCreator::run(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<PointType>::Ptr keypoints)
{
	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud (cloud);
	uniform_sampling.setRadiusSearch (radius->var * Blackboard::RESOLUTION);
	uniform_sampling.filter (*keypoints);
}

std::string NDCreator::getName()
{
	return "NDCreator";
}
