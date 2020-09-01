/*
 * ShotDescriptorCreator.cpp
 *
 *  Created on: Nov 16, 2018
 *      Author: satishj
 */


#include "../include/descriptor/UShapeContextCreator.h"

UShapeContextCreator::UShapeContextCreator()
{
	squareDistanceThreshold = RosParameterManager::createParam<float> (0.25,ParamType::FLOAT,0.0,1.0,"/matching/usc/squareDistanceThreshold");
	minimalRadius = RosParameterManager::createParam<double> (10.0,ParamType::DOUBLE,0.0,100.0,"/descriptor/usc/minimalRadius");
	pointDensityRadius = RosParameterManager::createParam<double> (5.0,ParamType::DOUBLE,0.0,100.0,"/descriptor/usc/pointDensityRadius");
	localRadius = RosParameterManager::createParam<double> (75.0,ParamType::DOUBLE,0.0,200.0,"/descriptor/usc/localRadius");
}

std::string UShapeContextCreator::getName()
{
	return "UShapeContext";
}

void UShapeContextCreator::run(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<PointType>::ConstPtr keypoints,
					   pcl::PointCloud<NormalType>::ConstPtr normals, boost::shared_ptr<Descriptors>& descriptors)
{
	if (!descriptors)
	{
		std::cout << "[UShapeContextCreator]: Creating new descriptors!\n";
		descriptors = boost::make_shared<UShapeContextDescriptors> ();
	}
	pcl::UniqueShapeContext<PointType, pcl::UniqueShapeContext1960> descr_est;
	descr_est.setInputCloud (keypoints);
	descr_est.setSearchSurface (cloud);
	descr_est.setRadiusSearch (radius->var * Blackboard::RESOLUTION);
	// The minimal radius value for the search sphere, to avoid being too sensitive
	// in bins close to the center of the sphere.
	descr_est.setMinimalRadius((radius->var * Blackboard::RESOLUTION) / minimalRadius->var);
	// Radius used to compute the local point density for the neighbors
	// (the density is the number of points within that radius).
	descr_est.setPointDensityRadius((radius->var * Blackboard::RESOLUTION) / pointDensityRadius->var);
	descr_est.setLocalRadius(localRadius->var * Blackboard::RESOLUTION);

	boost::shared_ptr<UShapeContextDescriptors> uscDesc = boost::dynamic_pointer_cast<UShapeContextDescriptors>(descriptors);
	if(!uscDesc)
	{
		std::cout << "[ShapeContextCreator]: Error! Descriptors are of wrong type! Expected: " << this->getName() << " but is " << descriptors->datatype << std::endl;
	}
	descr_est.compute (*uscDesc->descriptors);

	uscDesc->setSquareDistanceThreshold(squareDistanceThreshold->var);
	uscDesc->matching.setInputCloud(uscDesc->descriptors);
	uscDesc->kdTreeValid = true;
}
boost::shared_ptr<Descriptors> UShapeContextCreator::getEmptyDescriptor()
{
	return boost::make_shared<UShapeContextDescriptors> ();
}
