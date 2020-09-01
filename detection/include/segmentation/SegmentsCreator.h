/*
 * SegmentsCreator.h
 *
 *  Created on: Nov 23, 2018
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_SEGMENTATION_SEGMENTSCREATOR_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_SEGMENTATION_SEGMENTSCREATOR_H_

#include <pcl/point_cloud.h>
#include "../Blackboard.h"
#include "../Segment.h"
#include <memory>
#include "Segmentation.h"

class SegmentsCreator
{
	public:
		SegmentsCreator();
		 void runSegmentation (pcl::PointCloud<PointType>::ConstPtr cloud, std::vector <Segment>* segmentVector);
		 std::shared_ptr<Segmentation> seg;
};

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_SEGMENTATION_SEGMENTSCREATOR_H_ */
