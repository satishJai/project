/*
 * ISSCreator.h
 *
 *  Created on: Nov 16, 2018
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_KEYPOINT_ISSCREATOR_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_KEYPOINT_ISSCREATOR_H_

#include "KeypointCreator.h"
#include <pcl/keypoints/iss_3d.h>
#include "../RosParameter.h"
#include "../RosParameterManager.h"

class ISSCreator : public KeypointCreator
{

	public:
		ISSCreator();
		~ISSCreator();
		void run(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<PointType>::Ptr keypoints) override;
		std::string getName() override;

	protected:
		RosParameter<int>::Ptr minNeighbors;
		RosParameter<double>::Ptr salientRadius;
		RosParameter<double>::Ptr nonMaxRadius;
		RosParameter<double>::Ptr threshold21;
		RosParameter<double>::Ptr threshold32;
};

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_KEYPOINT_ISSCREATOR_H_ */
