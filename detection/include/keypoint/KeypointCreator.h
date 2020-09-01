/*
 * KeypointCreator.h
 *
 *  Created on: Nov 16, 2018
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_KEYPOINT_KEYPOINTCREATOR_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_KEYPOINT_KEYPOINTCREATOR_H_

#include <pcl/point_cloud.h>
#include "../Blackboard.h"
#include "../RosParameter.h"
#include "../RosParameterManager.h"

class KeypointCreator
{
	public:
		KeypointCreator()
		{
			radius = RosParameterManager::createParam<double> (20.0,ParamType::DOUBLE,0.0,100.0,"/keypoints/radius");
		}
		virtual ~KeypointCreator()
		{
			RosParameterManager::deleteParam(radius);
		}
		virtual void run(pcl::PointCloud<PointType>::ConstPtr cloud,
						 pcl::PointCloud<PointType>::Ptr keypoints) = 0;
		virtual std::string getName() = 0;

	protected:
		RosParameter<double>::Ptr radius;
};

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_KEYPOINT_KEYPOINTCREATOR_H_ */
