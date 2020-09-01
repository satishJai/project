/*
 * ShotDescriptorCreator.cpp
 *
 *  Created on: Nov 16, 2018
 *      Author: satishj
 */


#include "../include/descriptor/ShapeContextCreator.h"

ShapeContextCreator::ShapeContextCreator()
{
	squareDistanceThreshold = RosParameterManager::createParam<float> (0.25,ParamType::FLOAT,0.0,1.0,"/matching/3dsc/squareDistanceThreshold");
	minimalRadius = RosParameterManager::createParam<double> (10.0,ParamType::DOUBLE,0.0,100.0,"/descriptor/3dsc/minimalRadius");
	pointDensityRadius = RosParameterManager::createParam<double> (5.0,ParamType::DOUBLE,0.0,100.0,"/descriptor/3dsc/pointDensityRadius");
}

std::string ShapeContextCreator::getName()
{
	return "ShapeContext";
}

void ShapeContextCreator::run(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<PointType>::ConstPtr keypoints,
					   pcl::PointCloud<NormalType>::ConstPtr normals, boost::shared_ptr<Descriptors>& descriptors)
{
	if (!descriptors)
	{
		std::cout << "[ShapeContextCreator]: Creating new descriptors!\n";
		descriptors = boost::make_shared<ShapeContextDescriptors> ();
	}
	pcl::ShapeContext3DEstimation<PointType, NormalType, pcl::ShapeContext1980> descr_est;
	descr_est.setInputCloud (keypoints);
	descr_est.setInputNormals (normals);
	descr_est.setSearchSurface (cloud);
	descr_est.setRadiusSearch (radius->var * Blackboard::RESOLUTION);
	// The minimal radius value for the search sphere, to avoid being too sensitive
	// in bins close to the center of the sphere.
	descr_est.setMinimalRadius((radius->var * Blackboard::RESOLUTION) / minimalRadius->var);
	// Radius used to compute the local point density for the neighbors
	// (the density is the number of points within that radius).
	descr_est.setPointDensityRadius((radius->var * Blackboard::RESOLUTION) / pointDensityRadius->var);

	boost::shared_ptr<ShapeContextDescriptors> scDesc = boost::dynamic_pointer_cast<ShapeContextDescriptors>(descriptors);
	if(!scDesc)
	{
		std::cout << "[ShapeContextCreator]: Error! Descriptors are of wrong type! Expected: " << this->getName() << " but is " << descriptors->datatype << std::endl;
	}
	descr_est.compute (*scDesc->descriptors);

	scDesc->setSquareDistanceThreshold(squareDistanceThreshold->var);
	scDesc->matching.setInputCloud(scDesc->descriptors);
	scDesc->kdTreeValid = true;
}
boost::shared_ptr<Descriptors> ShapeContextCreator::getEmptyDescriptor()
{
	return boost::make_shared<ShapeContextDescriptors> ();
}
