/*
 * Matching.h
 *
 *  Created on: Nov 16, 2018
 *      Author: philipf
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_MATCHING_MATCHING_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_MATCHING_MATCHING_H_

#include "../descriptor/Descriptors.h"
#include "../Model.h"
#include <pcl/correspondence.h>

class Matching{
	public:
		virtual ~Matching () = default;
		virtual void run(Descriptors& descriptors, pcl::PointCloud<PointType>::ConstPtr keypoints, const std::vector<Model>* models,
			    		 std::vector<pcl::CorrespondencesPtr>* model_scene_corrs_vec, std::vector<unsigned long> *notFilteredCorresCountVec) = 0;
};

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_MATCHING_MATCHING_H_ */
