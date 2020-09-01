/*
 * Model.h
 *
 *  Created on: Nov 16, 2018
 *      Author: philipf
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_MODEL_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_MODEL_H_

#include "Blackboard.h"
#include <pcl/point_cloud.h>
#include "descriptor/Descriptors.h"
#include "ProcessedCloud.h"
#include <tf/tf.h>

struct Model{
		Model()
		{
		}
		Model(unsigned int gId, std::string gName)
		{
			id = gId;
			name = gName;
			isValid = false;

		}
		ProcessedCloud data;
		unsigned int id = 0;
		std::string name = "UNKNOWN";
		tf::Vector3 positionOffset = tf::Vector3(0,0,0); //Used for visualization
		//Is set to valid if all items have been calculated and set regarding the current active profile.
		//If the profile has been changed, is valid has to be set to false!
		bool isValid = false;
};



#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_MODEL_H_ */
