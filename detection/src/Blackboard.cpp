/*
 * Blackboard.cpp
 *
 *  Created on: Nov 26, 2018
 *      Author: philipf
 */
#include "../include/Blackboard.h"
#include "../include/keypoint/VoxelGridCreator.h"
#include "../include/descriptor/RSDCreator.h"
#include "../include/descriptor/CShotCreator.h"
#include "../include/descriptor/ShotCreator.h"
#include "../include/matching/ColorMatching.h"
#include "../include/pose_estimator/GCClustering.h"
#include "../include/pose_estimator/HoughClustering.h"
#include "../include/evaluation/CorrespondenceVoting.h"
#include "../include/segmentation/SegmentsCreator.h"
#include "../include/Segment.h"
#include "../include/evaluation/Guesser.h"
#include "../include/Model.h"
#include "../include/RosParameterManager.h"
#include "../include/RosParameter.h"

RosParameterManager Blackboard::parameterManager;

RosParameter<double>::Ptr Blackboard::rfRadius;
RosParameter<int>::Ptr Blackboard::normalRadius;
RosParameter<bool>::Ptr Blackboard::debugDescriptors;

Blackboard::Blackboard()
{
	//The pointers should point to some default object.
	keypointCreator = std::make_shared<VoxelGridCreator>();
	descriptorCreator = std::make_shared<ShotCreator>();
	segmentation = std::make_shared<SegmentsCreator>();
	poseEstimator = std::make_shared<GCClustering>();
	matching = std::make_shared<ColorMatching>();
	resultEvaluation = std::make_shared<CorrespondenceVoting>();
	guesser = std::make_shared<Guesser>();
	counter = 0;

	Blackboard::normalRadius = RosParameterManager::createParam<int> (20,ParamType::INT,1,100,"/normals/radius");
	Blackboard::rfRadius = RosParameterManager::createParam<double> (30.0,ParamType::DOUBLE,0.0,100.0,"/refFrame/radius");
	Blackboard::debugDescriptors = RosParameterManager::createParam<bool> (false,ParamType::BOOL,"/debug/descriptorDebugMessages");
}
