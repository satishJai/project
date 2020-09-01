/*
 * Segment.h
 *
 *  Created on: Nov 16, 2018
 *      Author: philipf
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_SEGMENT_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_SEGMENT_H_

#include "Blackboard.h"
#include "descriptor/Descriptors.h"
#include "Result.h"
#include "ProcessedCloud.h"
#include <pcl/point_cloud.h>

struct Segment
{
		Segment()
		{
			recognized = false;
		}
		ProcessedCloud data;
		//All unclustered correspondences, per model.
		std::vector<pcl::CorrespondencesPtr> model_scene_corrs_vec;
		std::vector<unsigned long> notFilteredCorresCountVec;
		std::vector<std::vector<Result>> results; //results[model][instance]
		bool recognized;
		Result bestGuess;
};



#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_SEGMENT_H_ */
