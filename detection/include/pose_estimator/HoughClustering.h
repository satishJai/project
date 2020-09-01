/*
 * HoughClustering.h
 *
 *  Created on: Nov 27, 2018
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_POSE_ESTIMATOR_HOUGHCLUSTERING_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_POSE_ESTIMATOR_HOUGHCLUSTERING_H_

#include "PoseEstimator.h"
#include <ros/ros.h>
#include <pcl/recognition/cg/hough_3d.h>

class HoughClustering: public PoseEstimator
{
	public:
		HoughClustering();
		void run(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<RFType>::ConstPtr rf, pcl::PointCloud<PointType>::ConstPtr keypoints, const Model& model,
				const pcl::CorrespondencesPtr& correspondences, std::vector<Result>* results) override;

	protected:
		RosParameter<double>::Ptr hgThreshold;
		RosParameter<double>::Ptr hgSize;
};


#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_POSE_ESTIMATOR_HOUGHCLUSTERING_H_ */
