/*
 * ShotDescriptorCreator.h
 *
 *  Created on: March 22, 2019
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_USHAPECONTEXTCREATOR_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_USHAPECONTEXTCREATOR_H_

#include "DescriptorCreator.h"
#include <pcl/features/usc.h>
#include "../include/descriptor/UShapeContextDescriptors.h"

class UShapeContextCreator : public DescriptorCreator
{
	public:
		UShapeContextCreator();
		void run(const pcl::PointCloud<PointType>::ConstPtr cloud, const pcl::PointCloud<PointType>::ConstPtr keypoints,
				 const pcl::PointCloud<NormalType>::ConstPtr normal, boost::shared_ptr<Descriptors>& descriptors) override;
		std::string getName() override;
		boost::shared_ptr<Descriptors> getEmptyDescriptor() override;

	protected:
		RosParameter<float>::Ptr squareDistanceThreshold;
		RosParameter<double>::Ptr minimalRadius;
		RosParameter<double>::Ptr pointDensityRadius;
		RosParameter<double>::Ptr localRadius;
};

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_USHAPECONTEXTCREATOR_H_ */
