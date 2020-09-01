/*
 * ShotDescriptorCreator.cpp
 *
 *  Created on: Nov 16, 2018
 *      Author: satishj
 */
#include "../include/descriptor/ShotCreator.h"

ShotCreator::ShotCreator()
{
	squareDistanceThreshold = RosParameterManager::createParam<float> (0.25,ParamType::FLOAT,0.0,1.0,"/matching/shot/squareDistanceThreshold");
}

std::string ShotCreator::getName()
{
	return "Shot";
};

void ShotCreator::run(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<PointType>::ConstPtr keypoints,
		 pcl::PointCloud<NormalType>::ConstPtr normal, boost::shared_ptr<Descriptors>& descriptors)
{
	if (!descriptors)
	{
		std::cout << "[ShotCreator]: Creating new descriptors!\n";
		descriptors = boost::make_shared<ShotDescriptors> ();
	}
	pcl::SHOTEstimationOMP<PointType, NormalType, pcl::SHOT352> descr_est;
	descr_est.setInputCloud (keypoints);
	descr_est.setInputNormals (normal);
	descr_est.setSearchSurface (cloud);
	descr_est.setRadiusSearch (radius->var * Blackboard::RESOLUTION);
	boost::shared_ptr<ShotDescriptors> shotDesc = boost::dynamic_pointer_cast<ShotDescriptors>(descriptors);
	if(!shotDesc)
	{
		std::cout << "[ShotCreator]: Error! Descriptors are of wrong type! Expected: " << this->getName() << " but is " << descriptors->datatype << std::endl;
	}
	descr_est.compute (*shotDesc->descriptors);

	shotDesc->setSquareDistanceThreshold(squareDistanceThreshold->var);
	shotDesc->matching.setInputCloud(shotDesc->descriptors);
	shotDesc->kdTreeValid = true;
}
boost::shared_ptr<Descriptors> ShotCreator::getEmptyDescriptor()
{
	return boost::make_shared<ShotDescriptors> ();
}
