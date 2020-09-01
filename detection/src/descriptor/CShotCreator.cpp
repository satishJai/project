/*
 * ShotDescriptorCreator.cpp
 *
 *  Created on: Nov 16, 2018
 *      Author: satishj
 */


#include "../include/descriptor/CShotCreator.h"

CShotCreator::CShotCreator()
{
	squareDistanceThreshold = RosParameterManager::createParam<float> (0.5,ParamType::FLOAT,0.0,1.0,"/matching/cshot/squareDistanceThreshold");
}

std::string CShotCreator::getName()
{
	return "CShot";
}

void CShotCreator::run(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<PointType>::ConstPtr keypoints,
					   pcl::PointCloud<NormalType>::ConstPtr normals, boost::shared_ptr<Descriptors>& descriptors)
{
	if (!descriptors)
	{
		std::cout << "[CShotCreator]: Creating new descriptors!\n";
		descriptors = boost::make_shared<CShotDescriptors> ();
	}
	pcl::SHOTColorEstimationOMP<PointType, NormalType, pcl::SHOT1344> descr_est;
	descr_est.setInputCloud (keypoints);
	descr_est.setInputNormals (normals);
	descr_est.setSearchSurface (cloud);
	descr_est.setRadiusSearch (radius->var * Blackboard::RESOLUTION);

	boost::shared_ptr<CShotDescriptors> shotDesc = boost::dynamic_pointer_cast<CShotDescriptors>(descriptors);
	if(!shotDesc)
	{
		std::cout << "[RSDCreator]: Error! Descriptors are of wrong type! Expected: " << this->getName() << " but is " << descriptors->datatype << std::endl;
	}
	descr_est.compute (*shotDesc->descriptors);

	shotDesc->setSquareDistanceThreshold(squareDistanceThreshold->var);
	shotDesc->matching.setInputCloud(shotDesc->descriptors);
	shotDesc->kdTreeValid = true;
}
boost::shared_ptr<Descriptors> CShotCreator::getEmptyDescriptor()
{
	return boost::make_shared<CShotDescriptors> ();
}
