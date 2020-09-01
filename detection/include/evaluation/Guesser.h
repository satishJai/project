/*
 * Guesser.h
 *
 *  Created on: Nov 27, 2018
 *      Author: philipf
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_EVALUATION_GUESSER_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_EVALUATION_GUESSER_H_

#include "../Blackboard.h"
#include "../Result.h"
#include "../Model.h"
#include "../Segment.h"
#include <tf/tf.h>
#include <pcl/point_cloud.h>
#include "../RosParameter.h"
#include "../RosParameterManager.h"

class Guesser
{

	public:
		Guesser();
		~Guesser();
		void run(const Segment & segment, const std::vector<Model> * models, Result& guess);
		bool guessObjectCenter(pcl::PointCloud<PointType>::ConstPtr cloud, tf::Point* center);
	protected:
		RosParameter<double>::Ptr threshold;
};



#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_EVALUATION_GUESSER_H_ */
