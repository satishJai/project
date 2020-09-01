/*
 * ColorMatching.h
 *
 *  Created on: Nov 16, 2018
 *      Author: philipf
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_MATCHING_SIMPLEMATCHING_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_MATCHING_SIMPLEMATCHING_H_

#include "Matching.h"
#include "../descriptor/Descriptors.h"
#include "../RosParameter.h"
#include "../RosParameterManager.h"

class SimpleMatching : public Matching
{
	public:
		void run(Descriptors& descriptors, pcl::PointCloud<PointType>::ConstPtr keypoints, const std::vector<Model>* models,
			 std::vector<pcl::CorrespondencesPtr>* model_scene_corrs_vec, std::vector<unsigned long> *notFilteredCorresCountVec) override;
};



#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_MATCHING_COLORMATCHING_H_ */
