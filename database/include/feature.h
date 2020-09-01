/*
 * feature.h
 *
 *  Created on: Feb 19, 2019
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DATABASE_INCLUDE_FEATURE_H_
#define SRC_DEXTERITY_DATABASE_INCLUDE_FEATURE_H_

#include <pcl/point_cloud.h>

struct Features{
		std::vector<int> id;
		std::vector<std::string> descriptor;
		std::vector<std::string> keypoint;
		std::vector<std::string> normal;
		std::vector<std::string> refFrame;

};



#endif /* SRC_DEXTERITY_DATABASE_INCLUDE_FEATURE_H_ */
