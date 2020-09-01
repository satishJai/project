/*
 * PoseEstimator.h
 *
 *  Created on: Nov 19, 2018
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_POSE_ESTIMATOR_POSEESTIMATOR_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_POSE_ESTIMATOR_POSEESTIMATOR_H_

#include "../Model.h"
#include "../Result.h"
#include "../Blackboard.h"
#include "../RosParameter.h"
#include "../RosParameterManager.h"

class PoseEstimator{
	public:
		virtual ~PoseEstimator () = default;
		virtual void run(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<RFType>::ConstPtr rf, pcl::PointCloud<PointType>::ConstPtr keypoints, const Model& model,
						 const pcl::CorrespondencesPtr& correspondences, std::vector<Result>* results) = 0;
};


#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_POSE_ESTIMATOR_POSEESTIMATOR_H_ */
