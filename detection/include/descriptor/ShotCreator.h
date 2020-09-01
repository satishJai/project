/*
 * ShotDescriptorCreator.h
 *
 *  Created on: Nov 16, 2018
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_SHOTCREATOR_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_SHOTCREATOR_H_

#include "DescriptorCreator.h"
#include <pcl/features/shot_omp.h>
#include "../include/descriptor/ShotDescriptors.h"

class ShotCreator : public DescriptorCreator
{
	public:
		ShotCreator();
		void run(const pcl::PointCloud<PointType>::ConstPtr cloud, const pcl::PointCloud<PointType>::ConstPtr keypoints,
				 const pcl::PointCloud<NormalType>::ConstPtr normal, boost::shared_ptr<Descriptors>& descriptors) override;
		std::string getName() override;
		boost::shared_ptr<Descriptors> getEmptyDescriptor() override;

	protected:
		RosParameter<float>::Ptr squareDistanceThreshold;
};

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_SHOTCREATOR_H_ */
