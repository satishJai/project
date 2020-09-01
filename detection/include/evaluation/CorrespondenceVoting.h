/*
 * CorrespondenceVoting.h
 *
 *  Created on: Nov 27, 2018
 *      Author: philipf
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_EVALUATION_CORRESPONDENCEVOTING_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_EVALUATION_CORRESPONDENCEVOTING_H_

#include "ResultEvaluation.h"
#include "../Segment.h"
#include "../Result.h"
#include "../Model.h"
#include "../RosParameter.h"
#include "../RosParameterManager.h"

class CorrespondenceVoting: public ResultEvaluation
{
	public:
		CorrespondenceVoting();
		~CorrespondenceVoting();
		bool run(Segment& segment, Result& solution, std::vector<Model> &models) override;
	protected:
		RosParameter<double>::Ptr correspondenceRatioThreshold;
};



#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_EVALUATION_CORRESPONDENCEVOTING_H_ */
