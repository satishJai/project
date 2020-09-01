/*
 * NDCreator.h
 *
 *  Created on: Nov 16, 2018
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_KEYPOINT_NDCREATOR_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_KEYPOINT_NDCREATOR_H_

#include "KeypointCreator.h"
#include <pcl/keypoints/uniform_sampling.h>
#include <ros/ros.h>

class NDCreator : public KeypointCreator
{
	public:
		void run(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<PointType>::Ptr keypoints) override;
		std::string getName() override;
};

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_KEYPOINT_NDCREATOR_H_ */
