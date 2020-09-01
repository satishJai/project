/*
 * CorrespondenceVoting.h
 *
 *  Created on: Nov 27, 2018
 *      Author: philipf
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_EVALUATION_CORRESPONDENCEVOTINGICP_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_EVALUATION_CORRESPONDENCEVOTINGICP_H_

#include "ResultEvaluation.h"
#include "../Segment.h"
#include "../Result.h"
#include "../Model.h"
#include "../RosParameter.h"
#include "../RosParameterManager.h"

class CorrespondenceVotingICP: public ResultEvaluation
{
	public:
		CorrespondenceVotingICP();
		~CorrespondenceVotingICP();
		bool run(Segment& segment, Result& solution, std::vector<Model> &models) override;
	protected:
		RosParameter<double>::Ptr correspondenceRatioThreshold;
		RosParameter<int>::Ptr maxIterations;
		RosParameter<double>::Ptr resolution;
		RosParameter<double>::Ptr transformationEpsilon;
 		bool getICPPose(pcl::PointCloud<PointType>::Ptr source, pcl::PointCloud<PointType>::Ptr destination, Result& guess, tf::Pose& pose);
};

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_EVALUATION_CORRESPONDENCEVOTINGICP_H_ */
