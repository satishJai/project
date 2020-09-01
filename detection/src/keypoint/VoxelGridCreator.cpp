/*
 * VoxelGridCreator.cpp
 *
 *  Created on: Nov 16, 2018
 *      Author: satishj
 */

#include "../../include/keypoint/VoxelGridCreator.h"

void VoxelGridCreator::run(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<PointType>::Ptr keypoints)
{
	pcl::VoxelGrid<PointType> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (radius->var * Blackboard::RESOLUTION, radius->var * Blackboard::RESOLUTION,radius->var * Blackboard::RESOLUTION);
	sor.filter (*keypoints);
}

std::string VoxelGridCreator::getName()
{
	return "VoxelGridCreator";
}
