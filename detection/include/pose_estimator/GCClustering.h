/*
 * GeometricConcistencyClustering.h
 *
 *  Created on: Nov 19, 2018
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_POSE_ESTIMATOR_GCCLUSTERING_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_POSE_ESTIMATOR_GCCLUSTERING_H_
#include "PoseEstimator.h"
#include <ros/ros.h>
#include <pcl/recognition/cg/geometric_consistency.h>

class GCClustering: public PoseEstimator
{
	public:
		GCClustering();
		void run(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<RFType>::ConstPtr rf, pcl::PointCloud<PointType>::ConstPtr keypoints, const Model& model,
				const pcl::CorrespondencesPtr& correspondences, std::vector<Result>* results) override;

	protected:
		RosParameter<double>::Ptr cgThreshold;
		RosParameter<double>::Ptr cgSize;
};


#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_POSE_ESTIMATOR_GCCLUSTERING_H_ */
