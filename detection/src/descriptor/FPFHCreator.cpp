/*
 * ShotDescriptorCreator.cpp
 *
 *  Created on: Nov 16, 2018
 *      Author: satishj
 */


#include "../include/descriptor/FPFHCreator.h"

FPFHCreator::FPFHCreator()
{
	squareDistanceThreshold = RosParameterManager::createParam<float> (300,ParamType::FLOAT,0.0,10000.0,"/matching/fpfh/squareDistanceThreshold");
}

std::string FPFHCreator::getName()
{
	return "FPFHCreator";
}

void FPFHCreator::run(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<PointType>::ConstPtr keypoints,
					   pcl::PointCloud<NormalType>::ConstPtr normals, boost::shared_ptr<Descriptors>& descriptors)
{
	if (!descriptors)
	{
		std::cout << "[FPFHCreator]: Creating new descriptors!\n";
		descriptors = boost::make_shared<FPFHDescriptors> ();
	}
	pcl::FPFHEstimationOMP<PointType, NormalType, pcl::FPFHSignature33> descr_est;
	descr_est.setInputCloud (keypoints);
	descr_est.setInputNormals (normals);
	descr_est.setSearchSurface (cloud);
	descr_est.setRadiusSearch (radius->var * Blackboard::RESOLUTION);

	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
	descr_est.setSearchMethod (tree);

	boost::shared_ptr<FPFHDescriptors> fpfhDesc = boost::dynamic_pointer_cast<FPFHDescriptors>(descriptors);
	if(!fpfhDesc)
	{
		std::cout << "[RSDCreator]: Error! Descriptors are of wrong type! Expected: " << this->getName() << " but is " << descriptors->datatype << std::endl;
	}
	descr_est.compute (*fpfhDesc->descriptors);

	fpfhDesc->setSquareDistanceThreshold(squareDistanceThreshold->var);
	fpfhDesc->matching.setInputCloud(fpfhDesc->descriptors);
	fpfhDesc->kdTreeValid = true;
}
boost::shared_ptr<Descriptors> FPFHCreator::getEmptyDescriptor()
{
	return boost::make_shared<FPFHDescriptors> ();
}
