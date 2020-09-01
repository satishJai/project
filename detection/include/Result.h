/*
 * Result.h
 *
 *  Created on: Nov 16, 2018
 *      Author: philipf
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_RESULT_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_RESULT_H_

#include "Blackboard.h"
#include <tf/tf.h>
#include <pcl/point_cloud.h>
#include "descriptor/Descriptors.h"

struct Result{
		pcl::Correspondences model_scene_corrs;
		bool validPose = false;
		unsigned int modelId = 0;
		std::string name = "UNKNOWN";
		tf::Pose pose;
		float probability = 0.0;

		//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
		//Eigen::Matrix4f model_rototranslations;
		//Using Eigen variables needs this macro: http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
};



#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_RESULT_H_ */
