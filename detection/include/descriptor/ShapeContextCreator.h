/*
 * ShotDescriptorCreator.h
 *
 *  Created on: March 22, 2019
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_SHAPECONTEXTCREATOR_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_SHAPECONTEXTCREATOR_H_

#include "DescriptorCreator.h"
#include <pcl/features/3dsc.h>
#include "../include/descriptor/ShapeContextDescriptors.h"

class ShapeContextCreator : public DescriptorCreator
{
	public:
		ShapeContextCreator();
		void run(const pcl::PointCloud<PointType>::ConstPtr cloud, const pcl::PointCloud<PointType>::ConstPtr keypoints,
				 const pcl::PointCloud<NormalType>::ConstPtr normal, boost::shared_ptr<Descriptors>& descriptors) override;
		std::string getName() override;
		boost::shared_ptr<Descriptors> getEmptyDescriptor() override;

	protected:
		RosParameter<float>::Ptr squareDistanceThreshold;
		RosParameter<double>::Ptr minimalRadius;
		RosParameter<double>::Ptr pointDensityRadius;
};

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_SHAPECONTEXTCREATOR_H_ */
