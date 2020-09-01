/*
 * ShotDescriptorCreator.cpp
 *
 *  Created on: Nov 16, 2018
 *      Author: satishj
 */

#include "../include/descriptor/RSDCreator.h"
#include "../include/matching/ColorMatching.h"

RSDCreator::RSDCreator()
{
	squareDistanceThreshold = RosParameterManager::createParam<float> (0.00002,ParamType::FLOAT,0.0,1.0,"/matching/rsd/squareDistanceThreshold");
	planeRadius = RosParameterManager::createParam<double> (20.0,ParamType::DOUBLE,0.0,100.0,"/descriptor/rsd/planeRadius");
	useColor = RosParameterManager::createParam<bool> (true,ParamType::BOOL,"/descriptor/rsd/color");
}

std::string RSDCreator::getName()
{
	return "RSD";
}

void RSDCreator::run (pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<PointType>::ConstPtr keypoints,
					  pcl::PointCloud<NormalType>::ConstPtr normals, boost::shared_ptr<Descriptors>& descriptors)
{
	if(!descriptors)
	{
		std::cout << "[RSDCreator]: Creating new descriptors!\n";
		descriptors = boost::make_shared<RSDDescriptors>();
	}
	pcl::RSDEstimation<PointType, NormalType, pcl::PrincipalRadiiRSD> desc_rsd;
	desc_rsd.setInputCloud (keypoints);
	desc_rsd.setInputNormals (normals);
	desc_rsd.setSearchSurface (cloud);
	desc_rsd.setRadiusSearch (radius->var * Blackboard::RESOLUTION);
	desc_rsd.setPlaneRadius (planeRadius->var);
	desc_rsd.setSaveHistograms (false);
	boost::shared_ptr<RSDDescriptors> rsdDesc = boost::dynamic_pointer_cast<RSDDescriptors>(descriptors);
	if(!rsdDesc)
	{
		std::cout << "[RSDCreator]: Error! Descriptors are of wrong type! Expected: " << this->getName() << " but is " << descriptors->datatype << std::endl;
	}
	desc_rsd.compute (*rsdDesc->descriptors);
	rsdDesc->descriptorWithColor->clear();
	/*Create a new descriptor which also stores the hue value*/
	for(size_t i = 0; i < rsdDesc->descriptors->size(); i++)
	{
		pcl::PointXYZ p;
		p.x = rsdDesc->descriptors->at(i).r_max;
		p.y = rsdDesc->descriptors->at(i).r_min;
		if(useColor->var)
		{
			std::cout << "UseColor == true is deprecated! Do not use this!\n";
			ColorMatching::hsv color = ColorMatching::rgb2hsv(ColorMatching::rgb(keypoints->at(i).r,keypoints->at(i).g,keypoints->at(i).b));
			p.z = color.h;
		}else
		{
			p.z = 0.0;
		}
		rsdDesc->descriptorWithColor->push_back(p);
	}
	rsdDesc->setSquareDistanceThreshold(squareDistanceThreshold->var);
	rsdDesc->matching.setInputCloud(rsdDesc->descriptorWithColor);
	rsdDesc->kdTreeValid = true;
}
boost::shared_ptr<Descriptors> RSDCreator::getEmptyDescriptor()
{
	return boost::make_shared<RSDDescriptors> ();
}
