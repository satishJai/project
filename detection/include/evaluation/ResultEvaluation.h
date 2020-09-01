/*
 * ResultEvaluation.h
 *
 *  Created on: Nov 27, 2018
 *      Author: philipf
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_EVALUATION_RESULTEVALUATION_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_EVALUATION_RESULTEVALUATION_H_
#include "../Result.h"
#include "../RosParameter.h"
#include "../RosParameterManager.h"

class ResultEvaluation
{
	public:
		ResultEvaluation()
		{
			sizeChecking = RosParameterManager::createParam<bool> (true,ParamType::BOOL,"/ResultEvaluation/sizeChecking");
		}
		virtual ~ResultEvaluation () = default;
		virtual bool run(Segment& segment, Result& solution, std::vector<Model> &models) = 0;
	protected:
		RosParameter<bool>::Ptr sizeChecking;
};



#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_EVALUATION_RESULTEVALUATION_H_ */
