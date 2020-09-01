/*
 * CSHOTCreator.h
 *
 *  Created on: March 22, 2019
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_FPFHCREATOR_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_FPFHCREATOR_H_

#include "DescriptorCreator.h"
#include <pcl/features/fpfh_omp.h>
#include "../include/descriptor/FPFHDescriptors.h"

class FPFHCreator : public DescriptorCreator
{
	public:
		FPFHCreator();

		void run(const pcl::PointCloud<PointType>::ConstPtr cloud, const pcl::PointCloud<PointType>::ConstPtr keypoints,
				 const pcl::PointCloud<NormalType>::ConstPtr normal, boost::shared_ptr<Descriptors>& descriptors) override;
		std::string getName() override;
		boost::shared_ptr<Descriptors> getEmptyDescriptor() override;

	protected:
		RosParameter<float>::Ptr squareDistanceThreshold;
};



#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_FPFHCREATOR_H_ */
