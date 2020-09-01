/*
 * RSDCreator.h
 *
 *  Created on: Nov 16, 2018
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_RSDCREATOR_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_RSDCREATOR_H_

#include "DescriptorCreator.h"
#include <pcl/features/rsd.h>
#include "../include/descriptor/RSDDescriptors.h"
class RSDCreator : public DescriptorCreator
{
	public:
		RSDCreator();
		void run (const pcl::PointCloud<PointType>::ConstPtr cloud, const pcl::PointCloud<PointType>::ConstPtr keypoints,
				  const pcl::PointCloud<NormalType>::ConstPtr normals, boost::shared_ptr<Descriptors>& descriptors);
		std::string getName() override;
		boost::shared_ptr<Descriptors> getEmptyDescriptor() override;

	protected:
		RosParameter<float>::Ptr squareDistanceThreshold;
		RosParameter<double>::Ptr planeRadius;
		RosParameter<bool>::Ptr useColor;
};

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_RSDCREATOR_H_ */
