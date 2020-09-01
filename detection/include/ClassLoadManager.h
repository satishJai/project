/*
 * ClassLoadManager.h
 *
 *  Created on: Feb 12, 2019
 *      Author: philipf
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_CLASSLOADMANAGER_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_CLASSLOADMANAGER_H_

#include <map>
#include <memory>

#include "descriptor/DescriptorCreator.h"
#include "keypoint/KeypointCreator.h"
#include "matching/Matching.h"
#include "pose_estimator/PoseEstimator.h"
#include "evaluation/ResultEvaluation.h"


class ClassLoadManager
{
	static const std::map<std::string,std::shared_ptr<DescriptorCreator>> descriptorCreators;
	static const std::map<std::string,std::shared_ptr<KeypointCreator>> keypointCreators;
	static const std::map<std::string,std::shared_ptr<Matching>> matchings;
	static const std::map<std::string,std::shared_ptr<PoseEstimator>> poseEstimators;
	static const std::map<std::string,std::shared_ptr<ResultEvaluation>> resultEvaluations;

	public:
		static std::shared_ptr<DescriptorCreator> getDescriptorCreator(std::string s);
		static std::shared_ptr<KeypointCreator> getKeypointCreator(std::string s);
		static std::shared_ptr<Matching> getMatching(std::string s);
		static std::shared_ptr<PoseEstimator> getPoseEstimator(std::string s);
		static std::shared_ptr<ResultEvaluation> getResultEvaluation(std::string s);
};



#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_CLASSLOADMANAGER_H_ */
